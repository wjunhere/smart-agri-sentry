#!/usr/bin/env python3
"""Web remote control node.

Flask-based HTTP API for manual robot control, mode switching, and demo stack
start/stop orchestration. Serves the v2 remote control page at port 5000.
"""

import os
import subprocess
import threading
import time
import math
from pathlib import Path

import rclpy
import yaml
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sentry_interfaces.msg import MissionStatus
from sentry_interfaces.srv import SetCropType
from std_srvs.srv import SetBool

# Defer Flask import to avoid import issues when not running
_app = None


def _stack_script_env(base_env=None):
    """Environment used when frontend-owned stack scripts run.

    The web node and rosbridge are the operator control plane. Frontend-triggered
    stack start/stop must preserve them, otherwise the browser would kill the
    server that is handling the button click.
    """
    env = dict(base_env or os.environ)
    env['SENTRY_PRESERVE_WEB'] = '1'
    env['ENABLE_WEB'] = 'false'
    env['ENABLE_VISION'] = 'true'
    env['ENABLE_ADVISORY'] = 'true'
    return env


def _mission_status_is_complete(msg: MissionStatus) -> bool:
    return (
        getattr(msg, 'state', '') == 'PATROL'
        and getattr(msg, 'total_wps', 0) > 0
        and getattr(msg, 'current_wp_idx', 0) >= getattr(msg, 'total_wps', 0)
    )


def _validate_vision_inference_mode(mode: str) -> str:
    if mode not in ('triggered', 'independent'):
        raise ValueError(f'Invalid vision inference mode: {mode}')
    return mode




def _validate_waypoints(waypoints):
    clean = []
    if not isinstance(waypoints, list) or not waypoints:
        raise ValueError('waypoints must be a non-empty list')
    for i, wp in enumerate(waypoints):
        if not isinstance(wp, dict):
            raise ValueError(f'waypoint {i} must be an object')
        try:
            x = float(wp['x'])
            y = float(wp['y'])
            yaw = float(wp['yaw'])
        except (KeyError, TypeError, ValueError) as exc:
            raise ValueError(f'waypoint {i} must contain numeric x, y, yaw') from exc
        if not all(math.isfinite(v) for v in (x, y, yaw)):
            raise ValueError(f'waypoint {i} contains non-finite values')
        clean.append({'x': x, 'y': y, 'yaw': yaw})
    return clean


def _read_waypoints_file(path: Path):
    with path.open('r', encoding='utf-8') as f:
        data = yaml.safe_load(f) or {}
    return _validate_waypoints(data.get('waypoints', []))


def _write_waypoints_file(path: Path, waypoints):
    text = '# Serpentine coverage waypoints edited from web frontend\n'
    text += 'waypoints:\n'
    for wp in waypoints:
        text += (
            f"  - {{x: {wp['x']:.4f}, y: {wp['y']:.4f}, "
            f"yaw: {wp['yaw']:.4f}}}\n"
        )
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(text, encoding='utf-8')

class WebRemoteNode(Node):
    def __init__(self):
        super().__init__('web_remote_node')
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.mode_srv = self.create_client(SetBool, '/set_auto_mode')
        self.crop_type_client = self.create_client(SetCropType, '/set_crop_type')
        self.mission_status_sub = self.create_subscription(
            MissionStatus, '/mission/status', self.on_mission_status, 10)

        self.declare_parameter('max_linear', 0.5)
        self.declare_parameter('max_angular', 1.0)
        self.declare_parameter(
            'stack_start_script',
            '/home/sunrise/dev_ws/scripts/rdk/start_robot_stack.sh')
        self.declare_parameter(
            'stack_stop_script',
            '/home/sunrise/dev_ws/scripts/rdk/stop_robot_stack.sh')
        self.declare_parameter('stack_script_timeout_sec', 180.0)
        self.max_linear = self.get_parameter('max_linear').value
        self.max_angular = self.get_parameter('max_angular').value
        self.stack_start_script = self.get_parameter('stack_start_script').value
        self.stack_stop_script = self.get_parameter('stack_stop_script').value
        self.stack_script_timeout = float(
            self.get_parameter('stack_script_timeout_sec').value)

        # Mission control starts in MANUAL; keep the web state aligned.
        self.mode = 'MANUAL'
        self.linear = 0.0
        self.angular = 0.0
        self.lock = threading.Lock()
        self.stack_lock = threading.Lock()
        self.last_cmd_time = time.time()
        self.TIMEOUT = 0.5
        self.frontend_started_auto = False
        self.completion_stop_started = False
        self.stack_ready = False
        self.last_stack_output = ''
        self.vision_inference_mode = 'triggered'
        self.vision_diagnosis_proc = None
        self.vision_diagnosis_log = None
        self.timer = self.create_timer(0.05, self.timer_cb)

    def timer_cb(self):
        with self.lock:
            now = time.time()
            if self.mode == 'MANUAL' and (now - self.last_cmd_time) > self.TIMEOUT:
                self.linear = 0.0
                self.angular = 0.0
            if self.mode == 'MANUAL':
                twist = Twist()
                twist.linear.x = self.linear
                twist.angular.z = self.angular
                self.cmd_pub.publish(twist)
            # AUTO: do not publish, Nav2 owns /cmd_vel

    def on_mission_status(self, msg: MissionStatus):
        if not _mission_status_is_complete(msg):
            return
        with self.lock:
            should_stop = self.mode == 'AUTO' and not self.completion_stop_started
            if should_stop:
                self.completion_stop_started = True
        if should_stop:
            self.get_logger().info(
                'Mission completed from frontend AUTO session; stopping stack')
            threading.Thread(
                target=self.stop_stack,
                kwargs={'reason': 'mission_complete'},
                daemon=True).start()

    def set_mode_auto(self, auto: bool) -> bool:
        if not self.mode_srv.service_is_ready():
            self.get_logger().error('/set_auto_mode service not available')
            return False
        req = SetBool.Request()
        req.data = auto
        future = self.mode_srv.call_async(req)
        future.add_done_callback(
            lambda f: self._on_mode_response(f, auto))
        with self.lock:
            self.mode = 'AUTO' if auto else 'MANUAL'
            self.frontend_started_auto = auto
            if auto:
                self.completion_stop_started = False
            else:
                self.linear = 0.0
                self.angular = 0.0
                self.last_cmd_time = time.time()
        self.get_logger().info(f"Switched to {self.mode}")
        return True

    def _on_mode_response(self, future, auto: bool):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info(
                    f"/set_auto_mode {'AUTO' if auto else 'MANUAL'} accepted: "
                    f"{response.message}")
            else:
                self.get_logger().warn(
                    f"/set_auto_mode {'AUTO' if auto else 'MANUAL'} rejected: "
                    f"{response.message}")
        except Exception as e:
            self.get_logger().error(f"/set_auto_mode call failed: {e}")

    def set_velocity(self, linear: float, angular: float):
        linear = max(-self.max_linear, min(self.max_linear, linear))
        angular = max(-self.max_angular, min(self.max_angular, angular))
        with self.lock:
            self.linear = linear
            self.angular = angular
            self.last_cmd_time = time.time()

    def emergency_stop(self):
        if self.mode_srv.service_is_ready():
            req = SetBool.Request()
            req.data = False
            future = self.mode_srv.call_async(req)
            future.add_done_callback(self._on_stop_response)
        with self.lock:
            self.mode = 'MANUAL'
            self.frontend_started_auto = False
            self.linear = 0.0
            self.angular = 0.0
            self.last_cmd_time = time.time()
        self.get_logger().warn('EMERGENCY STOP triggered')

    def _on_stop_response(self, future):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info(
                    f"/set_auto_mode stop accepted: {response.message}")
            else:
                self.get_logger().warn(
                    f"/set_auto_mode stop rejected: {response.message}")
        except Exception as e:
            self.get_logger().error(f"/set_auto_mode stop call failed: {e}")

    def _run_stack_script(self, script_path: str):
        path = Path(script_path)
        if not path.exists():
            return False, f'Script not found: {path}'
        try:
            result = subprocess.run(
                ['bash', str(path)],
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                timeout=self.stack_script_timeout,
                env=_stack_script_env())
        except subprocess.TimeoutExpired as exc:
            output = exc.stdout or ''
            return False, f'Script timed out: {path}\n{output}'
        except Exception as exc:
            return False, f'Script failed to start: {path}: {exc}'
        output = result.stdout or ''
        self.last_stack_output = output[-4000:]
        if result.returncode != 0:
            return False, output
        return True, output

    def _wait_for_mode_service(self, timeout_sec=10.0) -> bool:
        if hasattr(self.mode_srv, 'wait_for_service'):
            return bool(self.mode_srv.wait_for_service(timeout_sec=timeout_sec))
        deadline = time.time() + timeout_sec
        while time.time() < deadline:
            if self.mode_srv.service_is_ready():
                return True
            time.sleep(0.1)
        return False

    def start_stack_and_auto(self):
        with self.stack_lock:
            self.get_logger().info('Frontend requested robot stack start')
            output = 'Robot stack already preheated; switching to AUTO.'
            with self.lock:
                stack_ready = self.stack_ready
            if not stack_ready or not self.mode_srv.service_is_ready():
                ok, output = self._run_stack_script(self.stack_start_script)
                if not ok:
                    self.get_logger().error(f'start_robot_stack failed: {output[-1000:]}')
                    return False, output
                with self.lock:
                    self.stack_ready = True
            if not self._wait_for_mode_service(timeout_sec=10.0):
                return False, '/set_auto_mode service not available after stack start'
            if not self.set_mode_auto(True):
                return False, '/set_auto_mode rejected AUTO request'
            return True, output

    def preheat_stack(self):
        with self.stack_lock:
            self.get_logger().info('Frontend requested robot stack preheat')
            ok, output = self._run_stack_script(self.stack_start_script)
            if not ok:
                self.get_logger().error(f'start_robot_stack preheat failed: {output[-1000:]}')
                with self.lock:
                    self.stack_ready = False
                return False, output
            if not self._wait_for_mode_service(timeout_sec=10.0):
                with self.lock:
                    self.stack_ready = False
                return False, '/set_auto_mode service not available after stack preheat'
            with self.lock:
                self.stack_ready = True
                self.mode = 'MANUAL'
                self.frontend_started_auto = False
                self.completion_stop_started = False
            return True, output

    def stop_stack(self, reason='frontend'):
        with self.stack_lock:
            self.get_logger().info(f'Frontend requested robot stack stop: {reason}')
            self.emergency_stop()
            self._stop_independent_diagnosis_locked()
            ok, output = self._run_stack_script(self.stack_stop_script)
            with self.lock:
                self.mode = 'MANUAL'
                self.frontend_started_auto = False
                self.completion_stop_started = False
                self.stack_ready = False
                self.linear = 0.0
                self.angular = 0.0
                self.last_cmd_time = time.time()
            if not ok:
                self.get_logger().error(f'stop_robot_stack failed: {output[-1000:]}')
                return False, output
            return True, output

    def set_vision_inference_mode(self, mode: str):
        mode = _validate_vision_inference_mode(mode)
        with self.stack_lock:
            if mode == 'independent':
                ok, message = self._start_independent_diagnosis_locked()
            else:
                ok, message = self._stop_independent_diagnosis_locked()
            if ok:
                with self.lock:
                    self.vision_inference_mode = mode
            return ok, message

    def _start_independent_diagnosis_locked(self):
        proc = self.vision_diagnosis_proc
        if proc is not None and proc.poll() is None:
            return True, 'independent diagnosis already running'

        log_path = Path('/tmp/vision_diagnosis_node_independent.log')
        try:
            if self.vision_diagnosis_log is not None:
                self.vision_diagnosis_log.close()
            self.vision_diagnosis_log = log_path.open('a', encoding='utf-8')
            crop_type = os.environ.get('CROP_TYPE', 'tomato')
            model_path = os.environ.get(
                'VISION_DIAGNOSIS_MODEL_PATH',
                '/home/sunrise/dev_ws/models/quantization/'
                'tomato_mobilenetv3_output/'
                'tomato_mobilenetv3_bayese_224x224_nv12.bin')
            cmd = [
                'ros2', 'run', 'sentry_vision', 'vision_diagnosis_node',
                '--ros-args',
                '-p', f'crop_type:={crop_type}',
                '-p', f'model_path:={model_path}',
                '-p', 'input_size:=224',
            ]
            self.vision_diagnosis_proc = subprocess.Popen(
                cmd,
                stdout=self.vision_diagnosis_log,
                stderr=subprocess.STDOUT,
                text=True,
                env=os.environ.copy(),
                start_new_session=True,
            )
        except Exception as exc:
            self.vision_diagnosis_proc = None
            return False, f'failed to start independent diagnosis: {exc}'

        self.get_logger().info(
            f'Started independent vision diagnosis node; log={log_path}')
        return True, 'independent diagnosis started'

    def _stop_independent_diagnosis_locked(self):
        proc = self.vision_diagnosis_proc
        if proc is not None and proc.poll() is None:
            try:
                os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
            except Exception:
                proc.terminate()
            try:
                proc.wait(timeout=3.0)
            except subprocess.TimeoutExpired:
                try:
                    os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
                except Exception:
                    proc.kill()
                proc.wait(timeout=2.0)
            self.get_logger().info('Stopped independent vision diagnosis node')
        # ros2 run may leave the Python entry-point child alive after the
        # wrapper exits; clean that exact independent diagnosis command too.
        pattern = '/sentry_vision/lib/sentry_vision/vision_diagnosis_node --ros-args'
        subprocess.run(
            ['pkill', '-TERM', '-f', pattern],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            timeout=2.0)
        self.vision_diagnosis_proc = None
        if self.vision_diagnosis_log is not None:
            self.vision_diagnosis_log.close()
            self.vision_diagnosis_log = None
        return True, 'independent diagnosis stopped'

    def get_status(self):
        with self.lock:
            now = time.time()
            return {
                'mode': self.mode,
                'linear': self.linear,
                'angular': self.angular,
                'timeout': (self.mode == 'MANUAL' and
                           (now - self.last_cmd_time) > self.TIMEOUT),
                'service_ready': self.mode_srv.service_is_ready(),
                'frontend_started_auto': self.frontend_started_auto,
                'completion_stop_started': self.completion_stop_started,
                'stack_ready': self.stack_ready,
                'vision_inference_mode': self.vision_inference_mode,
            }

    def destroy_node(self):
        with self.stack_lock:
            self._stop_independent_diagnosis_locked()
        super().destroy_node()


def _get_app(node: WebRemoteNode):
    """Lazy Flask app creation."""
    global _app
    if _app is not None:
        return _app

    from flask import Flask, request, jsonify, send_from_directory
    from ament_index_python.packages import get_package_share_directory
    _app = Flask(__name__)
    SHARE_DIR = Path(get_package_share_directory('sentry_mission'))
    STATIC_DIR = SHARE_DIR / 'static'
    STATIC_V2_DIR = SHARE_DIR / 'static_v2'
    WAYPOINTS_FILE = SHARE_DIR / 'config' / 'waypoints.yaml'
    SOURCE_WAYPOINTS_FILE = Path('/home/sunrise/dev_ws/src/sentry_mission/config/waypoints.yaml')

    @_app.route('/')
    def index():
        return send_from_directory(str(STATIC_V2_DIR), 'index.html')

    @_app.route('/old')
    def old_index():
        return send_from_directory(str(STATIC_DIR), 'index.html')

    @_app.route('/<path:filename>')
    def v2_static(filename):
        return send_from_directory(str(STATIC_V2_DIR), filename)

    @_app.route('/mode', methods=['POST'])
    def set_mode():
        data = request.get_json()
        auto = data.get('auto', False)
        ok = node.set_mode_auto(auto)
        return jsonify({
            'status': 'ok' if ok else 'error',
            'mode': 'AUTO' if auto else 'MANUAL'
        })

    @_app.route('/stack/start', methods=['POST'])
    def stack_start():
        ok, output = node.start_stack_and_auto()
        return jsonify({
            'status': 'ok' if ok else 'error',
            'mode': 'AUTO' if ok else node.mode,
            'stack_ready': node.stack_ready,
            'message': output[-2000:],
        }), 200 if ok else 500

    @_app.route('/stack/preheat', methods=['POST'])
    def stack_preheat():
        ok, output = node.preheat_stack()
        return jsonify({
            'status': 'ok' if ok else 'error',
            'mode': 'MANUAL',
            'stack_ready': node.stack_ready,
            'message': output[-2000:],
        }), 200 if ok else 500

    @_app.route('/stack/stop', methods=['POST'])
    def stack_stop():
        ok, output = node.stop_stack(reason='frontend')
        return jsonify({
            'status': 'ok' if ok else 'error',
            'mode': 'MANUAL',
            'stack_ready': node.stack_ready,
            'message': output[-2000:],
        }), 200 if ok else 500

    @_app.route('/stop', methods=['POST'])
    def stop():
        node.emergency_stop()
        return jsonify({'status': 'stopped', 'mode': 'MANUAL'})

    @_app.route('/control', methods=['POST'])
    def control():
        data = request.get_json()
        linear = data.get('linear', 0.0)
        angular = data.get('angular', 0.0)
        node.set_velocity(linear, angular)
        return jsonify({'status': 'ok'})

    @_app.route('/status', methods=['GET'])
    def status():
        return jsonify(node.get_status())

    @_app.route('/vision/inference-mode', methods=['GET'])
    def get_vision_inference_mode():
        return jsonify({
            'status': 'ok',
            'mode': node.get_status()['vision_inference_mode'],
        })

    @_app.route('/vision/inference-mode', methods=['POST'])
    def set_vision_inference_mode():
        data = request.get_json() or {}
        mode = data.get('mode', 'triggered')
        try:
            ok, message = node.set_vision_inference_mode(mode)
        except ValueError as exc:
            return jsonify({'status': 'error', 'message': str(exc)}), 400
        return jsonify({
            'status': 'ok' if ok else 'error',
            'mode': node.get_status()['vision_inference_mode'],
            'message': message,
        }), 200 if ok else 500


    @_app.route('/waypoints', methods=['GET'])
    def get_waypoints():
        try:
            waypoints = _read_waypoints_file(WAYPOINTS_FILE)
        except Exception as exc:
            return jsonify({'status': 'error', 'message': str(exc)}), 500
        return jsonify({'status': 'ok', 'waypoints': waypoints})

    @_app.route('/waypoints', methods=['POST'])
    def set_waypoints():
        data = request.get_json() or {}
        try:
            waypoints = _validate_waypoints(data.get('waypoints'))
            _write_waypoints_file(WAYPOINTS_FILE, waypoints)
            if SOURCE_WAYPOINTS_FILE.exists() and SOURCE_WAYPOINTS_FILE != WAYPOINTS_FILE:
                _write_waypoints_file(SOURCE_WAYPOINTS_FILE, waypoints)
        except Exception as exc:
            return jsonify({'status': 'error', 'message': str(exc)}), 400
        return jsonify({'status': 'ok', 'waypoints': waypoints})
    @_app.route('/crop_type', methods=['POST'])
    def set_crop_type():
        data = request.get_json()
        crop = data.get('crop_type', '')
        if not node.crop_type_client.wait_for_service(timeout_sec=1.0):
            return jsonify({'status': 'error', 'message': 'Service unavailable'})

        req = SetCropType.Request()
        req.crop_type = crop
        future = node.crop_type_client.call_async(req)
        event = threading.Event()
        result = {}

        def done_cb(fut):
            try:
                result['response'] = fut.result()
            except Exception as e:
                result['error'] = str(e)
            finally:
                event.set()

        future.add_done_callback(done_cb)
        if not event.wait(timeout=2.0):
            return jsonify({'status': 'error', 'message': 'Request timed out'})
        if result.get('error'):
            return jsonify({'status': 'error', 'message': result['error']})
        resp = result.get('response')
        if resp is not None and resp.success:
            return jsonify({'status': 'ok', 'message': resp.message})
        return jsonify({'status': 'error', 'message': resp.message if resp else 'Unknown error'})

    # Shared mock diagnosis mode (cross-client sync)
    _app.config['mock_diagnosis_mode'] = 'real'

    @_app.route('/mock-diagnosis-mode', methods=['GET'])
    def get_mock_mode():
        return jsonify({'mode': _app.config['mock_diagnosis_mode']})

    @_app.route('/mock-diagnosis-mode', methods=['POST'])
    def set_mock_mode():
        data = request.get_json()
        mode = data.get('mode', 'real')
        if mode not in ('real', 'healthy', 'early_blight'):
            return jsonify({'status': 'error', 'message': f'Invalid mode: {mode}'})
        _app.config['mock_diagnosis_mode'] = mode
        return jsonify({'status': 'ok', 'mode': mode})

    return _app


def _start_flask(node: WebRemoteNode):
    app = _get_app(node)
    app.run(host='0.0.0.0', port=5000, debug=False, use_reloader=False, threaded=True)


def main(args=None):
    rclpy.init(args=args)
    node = WebRemoteNode()
    flask_thread = threading.Thread(target=_start_flask, args=(node,), daemon=True)
    flask_thread.start()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
