#!/usr/bin/env python3
"""Web remote control node.

Flask-based HTTP API for manual robot control, mode switching, and demo stack
start/stop orchestration. Serves the v2 remote control page at port 5000.
"""

import os
import signal
import subprocess
import threading
import time
import math
from pathlib import Path

import rclpy
import yaml
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from geometry_msgs.msg import Twist
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String
from sentry_interfaces.msg import (MissionStatus, PlantDetection, Diagnosis,
                                   FusionResult, AdvisoryAction, ForecastAlert)
from sentry_mission.batch_recorder import BatchRecorder, draw_bbox_on_jpeg
from sentry_mission.cruise_history import CruiseHistoryStore
from sentry_interfaces.srv import SetCropType
from std_srvs.srv import SetBool

# Defer Flask import to avoid import issues when not running
_app = None
CRUISE_SPEED_MIN = 0.05
CRUISE_SPEED_MAX = 0.35
DEFAULT_CRUISE_SPEED = 0.18
DEFAULT_FIXED_POINT_RADIUS = 0.20
TOMATO_DISEASE_CLASSES = frozenset((
    'late_blight',
    'healthy',
    'early_blight',
    'bacterial_spot',
    'leaf_mold',
    'septoria_leaf_spot',
    'tomato_yellow_leaf_curl_virus',
))


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
    env['CAMERA_BACKEND'] = 'mipi'
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


def _validate_cruise_speed(speed) -> float:
    speed = float(speed)
    if not CRUISE_SPEED_MIN <= speed <= CRUISE_SPEED_MAX:
        raise ValueError(
            f'Cruise speed must be between {CRUISE_SPEED_MIN:.2f} and '
            f'{CRUISE_SPEED_MAX:.2f} m/s')
    return speed


def _validate_fixed_point_stops(stops) -> list:
    if not isinstance(stops, list):
        raise ValueError('fixed_point_stops must be a list')

    clean = []
    for index, stop in enumerate(stops):
        if not isinstance(stop, dict):
            raise ValueError(f'fixed_point_stops[{index}] must be an object')
        try:
            x = float(stop['x'])
            y = float(stop['y'])
            radius = float(stop.get('radius', DEFAULT_FIXED_POINT_RADIUS))
            disease_class = str(stop['disease_class'])
        except (KeyError, TypeError, ValueError) as exc:
            raise ValueError(
                f'fixed_point_stops[{index}] must contain x, y, radius, and disease_class'
            ) from exc
        if not all(math.isfinite(value) for value in (x, y, radius)) or radius <= 0.0:
            raise ValueError(
                f'fixed_point_stops[{index}] has invalid coordinates or radius')
        if disease_class not in TOMATO_DISEASE_CLASSES:
            raise ValueError(f'fixed_point_stops[{index}] has invalid disease_class')
        clean.append({
            'x': x,
            'y': y,
            'radius': radius,
            'disease_class': disease_class,
        })
    return clean


def _read_mission_params(path: Path) -> dict:
    with path.open('r', encoding='utf-8') as f:
        data = yaml.safe_load(f) or {}
    if not isinstance(data, dict):
        raise ValueError('mission parameters must be an object')
    return {
        **data,
        'cruise_speed': _validate_cruise_speed(
            data.get('cruise_speed', DEFAULT_CRUISE_SPEED)),
        'fixed_point_stops': _validate_fixed_point_stops(
            data.get('fixed_point_stops', [])),
    }


def _write_mission_params(path: Path, params: dict) -> None:
    normalized = {
        **params,
        'cruise_speed': _validate_cruise_speed(
            params.get('cruise_speed', DEFAULT_CRUISE_SPEED)),
        'fixed_point_stops': _validate_fixed_point_stops(
            params.get('fixed_point_stops', [])),
    }
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        '# Mission parameters saved from web frontend and loaded during preheat\n'
        + yaml.safe_dump(normalized, allow_unicode=False, sort_keys=False),
        encoding='utf-8')


def _read_cruise_speed_file(path: Path) -> float:
    return _read_mission_params(path)['cruise_speed']


def _write_cruise_speed_file(path: Path, speed) -> None:
    try:
        params = _read_mission_params(path)
    except FileNotFoundError:
        params = {}
    params['cruise_speed'] = speed
    _write_mission_params(path, params)


def _write_fixed_point_stops_file(path: Path, stops) -> list:
    clean_stops = _validate_fixed_point_stops(stops)
    try:
        params = _read_mission_params(path)
    except FileNotFoundError:
        params = {}
    params['fixed_point_stops'] = clean_stops
    _write_mission_params(path, params)
    return clean_stops




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
        self.cmd_pub = self.create_publisher(Twist, '/sentry/cmd_vel', 10)
        self.mode_srv = self.create_client(SetBool, '/set_auto_mode')
        self.crop_type_client = self.create_client(SetCropType, '/set_crop_type')
        # Latched crop-type channel for the monitoring plane: the streaming
        # vision_diagnosis_node reloads its model when this changes. Works
        # without mission_control_node (unlike the /set_crop_type service).
        self.crop_type_pub = self.create_publisher(
            String, '/vision/crop_type',
            QoSProfile(depth=1,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.current_crop_type = os.environ.get('CROP_TYPE', 'tomato')
        self.crop_type_pub.publish(String(data=self.current_crop_type))
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
        self.declare_parameter(
            'camera_start_script',
            '/home/sunrise/dev_ws/scripts/rdk/start_camera_stack.sh')
        self.declare_parameter(
            'camera_stop_script',
            '/home/sunrise/dev_ws/scripts/rdk/stop_camera_stack.sh')
        self.declare_parameter(
            'inference_start_script',
            '/home/sunrise/dev_ws/scripts/rdk/start_inference_stack.sh')
        self.declare_parameter(
            'inference_stop_script',
            '/home/sunrise/dev_ws/scripts/rdk/stop_inference_stack.sh')
        self.declare_parameter('stack_script_timeout_sec', 180.0)
        self.declare_parameter('capture_dir', '/home/sunrise/dev_ws/images')
        self.max_linear = self.get_parameter('max_linear').value
        self.max_angular = self.get_parameter('max_angular').value
        self.stack_start_script = self.get_parameter('stack_start_script').value
        self.stack_stop_script = self.get_parameter('stack_stop_script').value
        self.camera_start_script = self.get_parameter(
            'camera_start_script').value
        self.camera_stop_script = self.get_parameter(
            'camera_stop_script').value
        self.inference_start_script = self.get_parameter(
            'inference_start_script').value
        self.inference_stop_script = self.get_parameter(
            'inference_stop_script').value
        self.stack_script_timeout = float(
            self.get_parameter('stack_script_timeout_sec').value)
        self.capture_dir = self.get_parameter('capture_dir').value

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
        self.camera_ready = False
        self.inference_ready = False
        self.cruise_speed = 0.18
        self.last_stack_output = ''
        self.vision_inference_mode = 'triggered'
        self.vision_diagnosis_proc = None
        self.vision_diagnosis_log = None
        # Demo support: >0 means robot-stack scripts start with
        # MOCK_HISTORY_HOURS (fusion LWD backfill). Set via /api/settings.
        self.mock_history_hours = 0.0
        # None | 'preheat' | 'start' | 'shutdown' — guards async stack ops
        self.stack_operation = None
        self.vision_pause_srv = self.create_client(
            SetBool, '/vision/plant_detector/pause')
        self._watchdog_fail_count = 0
        self._last_watchdog_restart = 0.0
        self.latest_camera_jpeg = None
        self.camera_image_sub = self.create_subscription(
            CompressedImage, '/out/compressed', self._on_camera_image, 1)
        self.latest_plant = None
        self.latest_plant_time = 0.0
        self._last_mission_state = None
        self.batch_recorder = BatchRecorder()
        self.history_store = CruiseHistoryStore(
            os.path.expanduser('~/.local/state/sentry/cruise_history'))
        self.plant_sub = self.create_subscription(
            PlantDetection, '/vision/plant_detected',
            self._on_plant_detected, 10)
        self.diagnosis_sub = self.create_subscription(
            Diagnosis, '/vision/diagnosis', self._on_diagnosis, 10)
        self.fusion_sub = self.create_subscription(
            FusionResult, '/fusion/diagnosis', self._on_fusion_history, 10)
        self.advisory_sub = self.create_subscription(
            AdvisoryAction, '/advisory/action', self._on_advisory_history, 10)
        self.alert_sub = self.create_subscription(
            ForecastAlert, '/forecast/alert', self._on_alert_history, 10)
        self.timer = self.create_timer(0.05, self.timer_cb)
        self.watchdog_timer = self.create_timer(5.0, self._watchdog_tick)

    def _on_camera_image(self, msg: CompressedImage):
        with self.lock:
            self.latest_camera_jpeg = bytes(msg.data)

    def _on_plant_detected(self, msg):
        # Latch positive frames only. While the car brakes the plant
        # leaves the frame, so the detector publishes negatives before the
        # STOPPED status tick reaches us; clearing on a negative frame
        # would silently drop the snapshot. The 2 s freshness check in
        # _record_detection_snapshot is the guard instead.
        if msg.detected:
            with self.lock:
                self.latest_plant = (list(msg.bbox), float(msg.confidence))
                self.latest_plant_time = time.time()

    def _on_diagnosis(self, msg):
        if getattr(msg, 'class_id', 0) == 254:
            return
        self.batch_recorder.on_diagnosis(
            msg.disease_class, float(msg.confidence))
        self.history_store.add_diagnosis(msg.disease_class, float(msg.confidence))

    def _on_fusion_history(self, msg):
        self.history_store.add_event('fusion', {
            'risk_score': float(msg.risk_score), 'alert_level': int(msg.alert_level),
            'mode': msg.mode, 'confidence': float(msg.confidence),
            'lwd_hours': float(msg.lwd_hours),
            'vision_term': float(msg.vision_term),
            'env_term': float(msg.env_term),
            'interaction_term': float(msg.interaction_term),
            'evidence_chain': list(msg.evidence_chain)})

    def _on_advisory_history(self, msg):
        self.history_store.add_event('advisory', {
            'action_type': msg.action_type, 'description': msg.description,
            'priority': msg.priority, 'steps': list(msg.steps)})

    def _on_alert_history(self, msg):
        self.history_store.add_event('alert', {
            'active': bool(msg.active), 'alert_type': msg.alert_type,
            'probability': float(msg.probability), 'description': msg.description,
            'hours_ahead': int(msg.hours_ahead)})

    def _record_detection_snapshot(self):
        with self.lock:
            plant = self.latest_plant
            plant_time = self.latest_plant_time
            jpeg = self.latest_camera_jpeg
        if plant is None or (time.time() - plant_time) > 2.0:
            self.get_logger().info(
                'Detection snapshot skipped: no recent plant detection')
            return  # 固定点停车且无有效检测框 -> 不记录
        if not jpeg:
            self.get_logger().warn('Detection snapshot skipped: no frame')
            return
        bbox, conf = plant
        try:
            snap = draw_bbox_on_jpeg(jpeg, bbox, f'Plant {conf * 100:.0f}%')
        except Exception as exc:
            self.get_logger().warn(f'bbox draw failed: {exc}')
            snap = jpeg
        self.batch_recorder.on_stop_trigger(bbox, conf, snap)
        self.history_store.add_detection(bbox, conf, snap)
        self.get_logger().info(
            f'Detection snapshot recorded (conf={conf:.2f})')

    def capture_camera_image(self):
        with self.lock:
            image_data = self.latest_camera_jpeg
        if not image_data:
            return False, 'No camera frame is available yet.'

        capture_dir = Path(self.capture_dir)
        capture_dir.mkdir(parents=True, exist_ok=True)
        timestamp_ms = int(time.time() * 1000)
        filename = f'capture_{timestamp_ms}.jpg'
        (capture_dir / filename).write_bytes(image_data)
        self.get_logger().info(f'Captured camera frame: {capture_dir / filename}')
        return True, filename

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
        state = getattr(msg, 'state', '')
        prev_state = self._last_mission_state
        self._last_mission_state = state
        if prev_state == 'PATROL' and state == 'STOPPED':
            self._record_detection_snapshot()
        if state == 'MANUAL':
            self.batch_recorder.on_mode_change('MANUAL')
            self.history_store.finish('manual')
        elif state and prev_state == 'MANUAL':
            self.history_store.start(self.current_crop_type)
        # Mirror mission state into the control-plane mode both ways: in
        # every non-MANUAL state mission_control or Nav2 owns /cmd_vel, so
        # the joystick watchdog must stay silent. A MANUAL status racing
        # set_mode_auto must not leave the mode stuck in MANUAL, or the
        # watchdog floods /cmd_vel with zeros for the whole cruise.
        with self.lock:
            if state and state != 'MANUAL':
                self.mode = 'AUTO'
            elif state == 'MANUAL':
                self.mode = 'MANUAL'
                self.frontend_started_auto = False
                self.completion_stop_started = False
        if not _mission_status_is_complete(msg):
            return
        with self.lock:
            should_stop = self.mode == 'AUTO' and not self.completion_stop_started
            if should_stop:
                self.completion_stop_started = True
        if should_stop:
            self.get_logger().info(
                'Mission completed from frontend AUTO session; stopping cruise')
            threading.Thread(
                target=self.stop_cruise,
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
        self.batch_recorder.on_mode_change('AUTO' if auto else 'MANUAL')
        if auto:
            self.history_store.start(self.current_crop_type)
        else:
            self.history_store.finish('manual')
        self._set_vision_paused(not auto)
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
        self.batch_recorder.on_mode_change('MANUAL')
        self.history_store.finish('emergency_stop')
        self._set_vision_paused(True)
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

    def _set_vision_paused(self, paused: bool):
        """Gate YOLO inference with the cruise mode: run in AUTO, idle in MANUAL.

        Keeps BPU/CPU load at zero while the resident stack is idle. No-op
        when the detector is not running (stack stopped or camera stack down).
        """
        try:
            ready = self.vision_pause_srv.service_is_ready()
        except Exception:
            ready = False
        if not ready:
            return
        req = SetBool.Request()
        req.data = paused
        future = self.vision_pause_srv.call_async(req)
        future.add_done_callback(
            lambda f: self._on_vision_pause_response(f, paused))

    def _on_vision_pause_response(self, future, paused: bool):
        try:
            response = future.result()
            if not response.success:
                self.get_logger().warn(
                    f"plant_detector pause={paused} rejected: "
                    f"{response.message}")
        except Exception as exc:
            self.get_logger().error(f'plant_detector pause call failed: {exc}')

    def _watchdog_tick(self):
        """Detect a dead robot stack while it is supposed to be resident.

        Two consecutive misses of the core nodes mark the stack not-ready. If
        the robot was cruising, e-stop first, then auto-restart the stack
        (rate-limited to one restart per 10 minutes).
        """
        with self.lock:
            ready = self.stack_ready
            operation = self.stack_operation
        if not ready or operation is not None or self.stack_lock.locked():
            self._watchdog_fail_count = 0
            return
        try:
            names = {name for name, _ in self.get_node_names_and_namespaces()}
        except Exception:
            return
        missing = [n for n in ('mission_control_node', 'uart_bridge_node')
                   if n not in names]
        if not missing:
            self._watchdog_fail_count = 0
            return
        self._watchdog_fail_count += 1
        if self._watchdog_fail_count < 2:
            return
        self._watchdog_fail_count = 0
        self.get_logger().error(
            f'Watchdog: stack nodes vanished {missing}; stack not-ready')
        with self.lock:
            self.stack_ready = False
            was_auto = self.mode == 'AUTO'
        if was_auto:
            self.emergency_stop()
        now = time.time()
        if was_auto and (now - self._last_watchdog_restart) > 600.0:
            self._last_watchdog_restart = now
            self.get_logger().warn('Watchdog: auto-restarting robot stack')
            with self.lock:
                self.stack_operation = 'start'
            threading.Thread(target=self._start_worker, daemon=True).start()

    def _detect_live_stack(self) -> bool:
        """Adopt an externally started stack: if the core nodes and the mode
        service are alive, treat the stack as resident-ready instead of
        forcing a full clean restart."""
        try:
            names = {name for name, _ in self.get_node_names_and_namespaces()}
        except Exception:
            return False
        if not {'mission_control_node', 'uart_bridge_node'} <= names:
            return False
        return self.mode_srv.service_is_ready()

    def _run_stack_script(self, script_path: str):
        path = Path(script_path)
        if not path.exists():
            return False, f'Script not found: {path}'
        env = _stack_script_env()
        # Frontend MOCK toggle decides whether the fusion node starts with a
        # backfilled LWD window; scripts that don't read it are unaffected.
        env['MOCK_HISTORY_HOURS'] = str(int(self.mock_history_hours))
        try:
            result = subprocess.run(
                ['bash', str(path)],
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                timeout=self.stack_script_timeout,
                env=env)
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

    def start_vision_stack(self):
        """Clean-restart the camera stack (kill duplicates, then start fresh).

        Keeps manual motion ownership; does not touch the robot/cruise stack.
        """
        with self.stack_lock:
            self.get_logger().info('Frontend requested camera stack (re)start')
            ok, output = self._run_stack_script(self.camera_start_script)
            with self.lock:
                self.camera_ready = ok
            if not ok:
                self.get_logger().error(f'start_camera_stack failed: {output[-1000:]}')
            return ok, output

    def start_inference_stack(self):
        """Clean-restart model inference nodes (YOLO detector + pipeline)."""
        with self.stack_lock:
            self.get_logger().info('Frontend requested inference stack (re)start')
            ok, output = self._run_stack_script(self.inference_start_script)
            with self.lock:
                self.inference_ready = ok
            if not ok:
                self.get_logger().error(f'start_inference_stack failed: {output[-1000:]}')
            return ok, output

    def stop_vision_stack(self):
        """Stop the camera stack (mipi node + republisher)."""
        with self.stack_lock:
            self.get_logger().info('Frontend requested camera stack stop')
            ok, output = self._run_stack_script(self.camera_stop_script)
            with self.lock:
                self.camera_ready = False
            if not ok:
                self.get_logger().error(f'stop_camera_stack failed: {output[-1000:]}')
            return ok, output

    def stop_inference_stack(self):
        """Stop model inference nodes (YOLO detector + pipeline)."""
        with self.stack_lock:
            self.get_logger().info('Frontend requested inference stack stop')
            ok, output = self._run_stack_script(self.inference_stop_script)
            with self.lock:
                self.inference_ready = False
            if not ok:
                self.get_logger().error(f'stop_inference_stack failed: {output[-1000:]}')
            return ok, output

    def _set_remote_parameter(self, node_name: str, parameter_name: str,
                              value: float):
        from rclpy.parameter import Parameter
        from rclpy.parameter_client import AsyncParameterClient

        client = AsyncParameterClient(self, node_name)
        if not client.wait_for_services(timeout_sec=1.0):
            return False, f'{node_name} parameter service unavailable'

        future = client.set_parameters([
            Parameter(parameter_name, value=value),
        ])
        done = threading.Event()
        result = {}

        def on_done(completed):
            try:
                result['response'] = completed.result()
            except Exception as exc:
                result['error'] = str(exc)
            finally:
                done.set()

        future.add_done_callback(on_done)
        if not done.wait(timeout=2.0):
            return False, f'{node_name} parameter update timed out'
        if result.get('error'):
            return False, result['error']
        responses = result.get('response') or []
        if not responses or not all(item.successful for item in responses):
            reason = next((item.reason for item in responses
                           if not item.successful), 'rejected')
            return False, f'{node_name} rejected {parameter_name}: {reason}'
        return True, 'ok'

    def set_cruise_speed(self, speed):
        speed = _validate_cruise_speed(speed)
        with self.lock:
            stack_ready = self.stack_ready

        if not stack_ready:
            with self.lock:
                self.cruise_speed = speed
            return True, f'Cruise speed saved for preheat: {speed:.2f} m/s'

        for node_name, parameter_name in (
                ('/mission_control_node', 'cruise_speed'),
                ('/controller_server', 'FollowPath.desired_linear_vel')):
            ok, message = self._set_remote_parameter(
                node_name, parameter_name, speed)
            if not ok:
                return False, message
        with self.lock:
            self.cruise_speed = speed
        return True, f'Cruise speed set to {speed:.2f} m/s'

    def _ensure_weather_proxy(self):
        """Start tools/weather_proxy.py if the :8090 endpoint is not up."""
        import socket
        ws_root = Path(self.stack_start_script).resolve().parents[2]
        proxy = ws_root / 'tools' / 'weather_proxy.py'
        if not proxy.exists():
            self.get_logger().warn(f'Weather proxy not found: {proxy}')
            return
        try:
            with socket.create_connection(('127.0.0.1', 8090), timeout=0.5):
                return  # already running
        except OSError:
            pass
        try:
            log = open('/tmp/weather_proxy.log', 'ab')
            subprocess.Popen(
                ['python3', str(proxy)], cwd=str(ws_root),
                stdout=log, stderr=subprocess.STDOUT,
                start_new_session=True)
            self.get_logger().info('Weather proxy started on :8090')
        except Exception as exc:
            self.get_logger().warn(f'Weather proxy start failed: {exc}')

    def start_stack_and_auto(self):
        with self.stack_lock:
            self.get_logger().info('Frontend requested robot stack start')
            self._ensure_weather_proxy()
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
            self._ensure_weather_proxy()
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
            self._set_vision_paused(True)  # idle stack: no YOLO until cruise
            return True, output

    def stop_cruise(self, reason='frontend'):
        """End the cruise but keep the robot stack resident (fast path).

        Switches mission_control back to MANUAL, zeros motion and pauses
        YOLO inference; Nav2/camera/vision nodes stay warm so the next
        start is a service call instead of a full relaunch.
        """
        self.get_logger().info(
            f'Frontend requested cruise stop (stack stays resident): {reason}')
        self.emergency_stop()
        return True, 'Cruise stopped; robot stack stays resident in MANUAL.'

    def shutdown_stack(self, reason='frontend'):
        """Full teardown of the robot stack (stop script); web plane stays up."""
        with self.stack_lock:
            self.get_logger().info(f'Frontend requested robot stack shutdown: {reason}')
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

    def _start_worker(self):
        try:
            self.start_stack_and_auto()
        finally:
            with self.lock:
                self.stack_operation = None

    def _preheat_worker(self):
        try:
            self.preheat_stack()
        finally:
            with self.lock:
                self.stack_operation = None

    def _shutdown_worker(self):
        try:
            self.shutdown_stack(reason='frontend')
        finally:
            with self.lock:
                self.stack_operation = None

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
                'tomato_mobilenetv3_v5_output/'
                'tomato_mobilenetv3_v5_bayese_224x224_nv12.bin')
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
        self.vision_diagnosis_proc = None
        if self.vision_diagnosis_log is not None:
            self.vision_diagnosis_log.close()
            self.vision_diagnosis_log = None
        return True, 'independent diagnosis stopped'

    def _get_stack_states(self):
        """Live state of camera/inference stacks from the ROS graph."""
        try:
            names = set(self.get_node_names())
        except Exception:
            return False, False
        camera_running = ('mipi_camera_node' in names
                          and 'image_republisher' in names)
        inference_running = ('plant_detector_node' in names
                             and 'vision_pipeline_node' in names)
        return camera_running, inference_running

    def get_status(self):
        camera_running, inference_running = self._get_stack_states()
        stack_busy = self.stack_lock.locked()
        with self.lock:
            self.camera_ready = camera_running
            self.inference_ready = inference_running
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
                'stack_busy': stack_busy,
                'stack_operation': self.stack_operation,
                'camera_running': camera_running,
                'inference_running': inference_running,
                'cruise_speed': self.cruise_speed,
                'vision_inference_mode': self.vision_inference_mode,
                'message_unread': self.batch_recorder.unread,
            }

    # ---- Runtime settings (frontend settings panel) ----

    def _param_client(self, node_name: str, srv_type, suffix: str):
        if not hasattr(self, '_param_clients'):
            self._param_clients = {}
        key = (node_name, suffix)
        if key not in self._param_clients:
            self._param_clients[key] = self.create_client(
                srv_type, f'{node_name}/{suffix}')
        return self._param_clients[key]

    def set_ros_param(self, node_name: str, param_name: str, value):
        """Set a parameter on another node; returns (ok, detail)."""
        from rcl_interfaces.srv import SetParameters
        from rclpy.parameter import Parameter as RosParameter
        client = self._param_client(
            node_name, SetParameters, 'set_parameters')
        if not client.service_is_ready():
            return False, f'{node_name} unavailable'
        req = SetParameters.Request()
        req.parameters = [
            RosParameter(param_name, value=value).to_parameter_msg()]
        done = threading.Event()
        future = client.call_async(req)
        future.add_done_callback(lambda _f: done.set())
        if not done.wait(timeout=3.0):
            return False, 'set_parameters timeout'
        try:
            result = future.result().results[0]
            return result.successful, result.reason or 'ok'
        except Exception as exc:
            return False, str(exc)

    def get_ros_param(self, node_name: str, param_name: str):
        """Read a parameter from another node; returns (ok, value)."""
        from rcl_interfaces.srv import GetParameters
        from rcl_interfaces.msg import Parameter as ParameterMsg
        from rclpy.parameter import Parameter as RosParameter
        client = self._param_client(
            node_name, GetParameters, 'get_parameters')
        if not client.service_is_ready():
            return False, None
        req = GetParameters.Request()
        req.names = [param_name]
        done = threading.Event()
        future = client.call_async(req)
        future.add_done_callback(lambda _f: done.set())
        if not done.wait(timeout=3.0):
            return False, None
        try:
            msg = ParameterMsg(
                name=param_name, value=future.result().values[0])
            return True, RosParameter.from_parameter_msg(msg).value
        except Exception:
            return False, None

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

    # CORS：允许本地开发模式（页面由 localhost 静态服务托管）跨域调用。
    # 仅限局域网使用，放开到 * 可接受；板端托管同源访问不受影响。
    @_app.after_request
    def _add_cors_headers(resp):
        resp.headers['Access-Control-Allow-Origin'] = '*'
        resp.headers['Access-Control-Allow-Methods'] = 'GET, POST, DELETE, OPTIONS'
        resp.headers['Access-Control-Allow-Headers'] = 'Content-Type'
        return resp

    @_app.route('/', defaults={'_path': ''}, methods=['OPTIONS'])
    @_app.route('/<path:_path>', methods=['OPTIONS'])
    def _cors_preflight(_path):
        return ('', 204)

    SHARE_DIR = Path(get_package_share_directory('sentry_mission'))
    STATIC_DIR = SHARE_DIR / 'static'
    STATIC_V2_DIR = SHARE_DIR / 'static_v2'
    WAYPOINTS_FILE = SHARE_DIR / 'config' / 'waypoints.yaml'
    SOURCE_WAYPOINTS_FILE = Path('/home/sunrise/dev_ws/src/sentry_mission/config/waypoints.yaml')
    CRUISE_SPEED_FILE = SHARE_DIR / 'config' / 'mission_params.yaml'
    SOURCE_CRUISE_SPEED_FILE = Path('/home/sunrise/dev_ws/src/sentry_mission/config/mission_params.yaml')
    try:
        node.cruise_speed = _read_cruise_speed_file(CRUISE_SPEED_FILE)
    except (OSError, ValueError, yaml.YAMLError) as exc:
        node.get_logger().warning(f'Using default cruise speed: {exc}')

    # Curated runtime-tunable settings for the frontend panel. Each entry
    # maps a UI key to one or more (node, parameter) targets; detector and
    # mission thresholds must move together or triggers desync.
    SETTINGS_SCHEMA = {
        'low_light_enhancement': {
            'type': 'bool',
            'targets': [('/mipi_camera_node', 'enable_low_light_enhancement')],
        },
        'detection_confidence': {
            'type': 'float', 'min': 0.1, 'max': 0.9,
            'targets': [
                ('/plant_detector_node', 'confidence_threshold'),
                ('/mission_control_node', 'detection_confidence_threshold'),
            ],
        },
        'servo_start_side': {
            'type': 'str', 'choices': ['left', 'right'],
            'targets': [('/mission_control_node', 'servo_start_side')],
        },
        'plant_stop_offset': {
            'type': 'float', 'min': 0.0, 'max': 45.0,
            'targets': [('/mission_control_node', 'servo_plant_stop_offset_deg')],
        },
        # Web-local (no ROS target): hours of synthetic LWD history the
        # fusion node backfills when the robot stack starts via /stack/*.
        'mock_history_hours': {
            'type': 'float', 'min': 0.0, 'max': 48.0, 'targets': [],
        },
    }

    @_app.route('/api/settings', methods=['GET'])
    def api_settings_get():
        out = {}
        for key, spec in SETTINGS_SCHEMA.items():
            if not spec['targets']:
                out[key] = getattr(node, key, None)
                continue
            node_name, param = spec['targets'][0]
            ok, value = node.get_ros_param(node_name, param)
            out[key] = value if ok else None
        return jsonify(out)

    @_app.route('/api/settings', methods=['POST'])
    def api_settings_post():
        data = request.get_json(force=True) or {}
        results = {}
        for key, value in data.items():
            spec = SETTINGS_SCHEMA.get(key)
            if spec is None:
                results[key] = 'unknown setting'
                continue
            try:
                if spec['type'] == 'bool':
                    value = bool(value)
                elif spec['type'] == 'float':
                    value = max(spec['min'], min(spec['max'], float(value)))
                elif spec['type'] == 'str':
                    value = str(value)
                    if value not in spec['choices']:
                        results[key] = 'invalid choice'
                        continue
            except (TypeError, ValueError):
                results[key] = 'invalid value'
                continue
            if not spec['targets']:
                # Web-local setting: stored on the node itself.
                setattr(node, key, value)
                results[key] = 'ok'
                continue
            oks = [node.set_ros_param(n, p, value)[0]
                   for n, p in spec['targets']]
            results[key] = 'ok' if all(oks) else 'failed'
        return jsonify({'results': results})

    @_app.route('/')
    def index():
        return send_from_directory(str(STATIC_V2_DIR), 'index.html')

    @_app.route('/old')
    def old_index():
        return send_from_directory(str(STATIC_DIR), 'index.html')

    @_app.route('/api/messages')
    def api_messages():
        return jsonify(node.batch_recorder.to_dict())

    @_app.route('/api/messages/<int:batch_id>/<int:seq>/snapshot')
    def api_message_snapshot(batch_id, seq):
        from flask import Response
        jpeg = node.batch_recorder.get_snapshot(batch_id, seq)
        if jpeg is None:
            return jsonify({'error': 'not found'}), 404
        return Response(jpeg, mimetype='image/jpeg')

    @_app.route('/api/messages/read', methods=['POST'])
    def api_messages_read():
        node.batch_recorder.mark_read()
        return jsonify({'status': 'ok'})

    @_app.route('/api/messages/clear', methods=['POST'])
    def api_messages_clear():
        node.batch_recorder.clear()
        return jsonify({'status': 'ok'})
    def _history_query_args(source):
        try:
            limit = max(1, min(30, int(source.get('limit', 10))))
        except (TypeError, ValueError):
            limit = 10
        def optional_time(key):
            value = source.get(key)
            if value in (None, ''):
                return None
            try:
                return float(value)
            except (TypeError, ValueError):
                raise ValueError(f'invalid {key}')
        return (limit, optional_time('start_at'), optional_time('end_at'),
                str(source.get('crop_type', '')).strip(),
                str(source.get('disease', '')).strip())

    @_app.route('/api/history/batches', methods=['GET'])
    def api_history_batches():
        try:
            args = _history_query_args(request.args)
        except ValueError as exc:
            return jsonify({'error': str(exc)}), 400
        return jsonify({'batches': node.history_store.query(*args)})

    @_app.route('/api/history/batches/<batch_id>/snapshot/<int:seq>', methods=['GET'])
    def api_history_snapshot(batch_id, seq):
        from flask import Response
        jpeg = node.history_store.snapshot(batch_id, seq)
        if jpeg is None:
            return jsonify({'error': 'not found'}), 404
        return Response(jpeg, mimetype='image/jpeg')

    @_app.route('/api/history/batches', methods=['DELETE'])
    def api_history_clear():
        data = request.get_json(silent=True) or {}
        if data.get('confirm') is not True:
            return jsonify({'error': 'explicit confirmation required'}), 400
        try:
            _, start_at, end_at, crop_type, disease = _history_query_args(data)
        except ValueError as exc:
            return jsonify({'error': str(exc)}), 400
        removed = node.history_store.clear(start_at, end_at, crop_type, disease)
        return jsonify({'status': 'ok', 'removed': removed})

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
        with node.lock:
            ready = node.stack_ready
        if not ready and node._detect_live_stack():
            with node.lock:
                node.stack_ready = True
            ready = True
        if ready and node.mode_srv.service_is_ready():
            # Fast path: resident stack, just flip to AUTO.
            ok, output = node.start_stack_and_auto()
            return jsonify({
                'status': 'ok' if ok else 'error',
                'mode': 'AUTO' if ok else node.mode,
                'stack_ready': node.stack_ready,
                'message': output[-2000:],
            }), 200 if ok else 500
        with node.lock:
            if node.stack_operation is not None:
                return jsonify({
                    'status': 'busy',
                    'message': f"stack operation in progress: {node.stack_operation}",
                    'stack_ready': node.stack_ready,
                }), 409
            node.stack_operation = 'start'
        threading.Thread(target=node._start_worker, daemon=True).start()
        return jsonify({
            'status': 'started',
            'stack_ready': False,
            'message': 'stack start launched in background; poll /status',
        }), 202

    @_app.route('/stack/preheat', methods=['POST'])
    def stack_preheat():
        with node.lock:
            ready = node.stack_ready
        if not ready and node._detect_live_stack():
            with node.lock:
                node.stack_ready = True
            ready = True
        if ready:
            # Already resident — preheat is a no-op.
            return jsonify({
                'status': 'ok',
                'mode': 'MANUAL',
                'stack_ready': True,
                'message': 'Robot stack already resident; preheat skipped.',
            })
        with node.lock:
            if node.stack_operation is not None:
                return jsonify({
                    'status': 'busy',
                    'message': f"stack operation in progress: {node.stack_operation}",
                    'stack_ready': node.stack_ready,
                }), 409
            node.stack_operation = 'preheat'
        threading.Thread(target=node._preheat_worker, daemon=True).start()
        return jsonify({
            'status': 'started',
            'stack_ready': False,
            'message': 'stack preheat launched in background; poll /status',
        }), 202

    @_app.route('/stack/shutdown', methods=['POST'])
    def stack_shutdown():
        with node.lock:
            if node.stack_operation is not None:
                return jsonify({
                    'status': 'busy',
                    'message': f"stack operation in progress: {node.stack_operation}",
                    'stack_ready': node.stack_ready,
                }), 409
            node.stack_operation = 'shutdown'
        threading.Thread(target=node._shutdown_worker, daemon=True).start()
        return jsonify({
            'status': 'started',
            'stack_ready': node.stack_ready,
            'message': 'stack shutdown launched in background; poll /status',
        }), 202

    @_app.route('/vision/start', methods=['POST'])
    def start_vision():
        ok, output = node.start_vision_stack()
        return jsonify({
            'status': 'ok' if ok else 'error',
            'mode': node.mode,
            'stack_ready': node.stack_ready,
            'camera_ready': node.camera_ready,
            'message': output[-2000:],
        }), 200 if ok else 500

    @_app.route('/inference/start', methods=['POST'])
    def start_inference():
        ok, output = node.start_inference_stack()
        return jsonify({
            'status': 'ok' if ok else 'error',
            'mode': node.mode,
            'inference_ready': node.inference_ready,
            'message': output[-2000:],
        }), 200 if ok else 500

    @_app.route('/vision/stop', methods=['POST'])
    def stop_vision():
        ok, output = node.stop_vision_stack()
        return jsonify({
            'status': 'ok' if ok else 'error',
            'mode': node.mode,
            'camera_ready': node.camera_ready,
            'message': output[-2000:],
        }), 200 if ok else 500

    @_app.route('/inference/stop', methods=['POST'])
    def stop_inference():
        ok, output = node.stop_inference_stack()
        return jsonify({
            'status': 'ok' if ok else 'error',
            'mode': node.mode,
            'inference_ready': node.inference_ready,
            'message': output[-2000:],
        }), 200 if ok else 500

    @_app.route('/stack/stop', methods=['POST'])
    def stack_stop():
        # Fast path: end the cruise, keep the stack resident.
        ok, output = node.stop_cruise(reason='frontend')
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

    @_app.route('/camera/capture', methods=['POST'])
    def capture_camera():
        try:
            ok, message = node.capture_camera_image()
        except OSError as exc:
            return jsonify({'status': 'error', 'message': str(exc)}), 500
        return jsonify({
            'status': 'ok' if ok else 'error',
            'filename': message if ok else None,
            'message': message,
        }), 200 if ok else 503

    @_app.route('/cruise-speed', methods=['POST'])
    def set_cruise_speed():
        data = request.get_json() or {}
        try:
            speed = _validate_cruise_speed(data.get('speed'))
            _write_cruise_speed_file(CRUISE_SPEED_FILE, speed)
            if SOURCE_CRUISE_SPEED_FILE.exists() and SOURCE_CRUISE_SPEED_FILE != CRUISE_SPEED_FILE:
                _write_cruise_speed_file(SOURCE_CRUISE_SPEED_FILE, speed)
            ok, message = node.set_cruise_speed(speed)
        except (TypeError, ValueError) as exc:
            return jsonify({'status': 'error', 'message': str(exc)}), 400
        except OSError as exc:
            return jsonify({'status': 'error', 'message': str(exc)}), 500
        return jsonify({
            'status': 'ok' if ok else 'error',
            'speed': node.get_status()['cruise_speed'],
            'message': message,
        }), 200 if ok else 500

    @_app.route('/fixed-point-stops', methods=['GET'])
    def get_fixed_point_stops():
        try:
            params = _read_mission_params(CRUISE_SPEED_FILE)
        except (OSError, ValueError, yaml.YAMLError) as exc:
            return jsonify({'status': 'error', 'message': str(exc)}), 500
        return jsonify({
            'status': 'ok',
            'fixed_point_stops': params['fixed_point_stops'],
        })

    @_app.route('/fixed-point-stops', methods=['POST'])
    def set_fixed_point_stops():
        data = request.get_json() or {}
        try:
            stops = _write_fixed_point_stops_file(
                CRUISE_SPEED_FILE, data.get('fixed_point_stops'))
            if (SOURCE_CRUISE_SPEED_FILE.exists()
                    and SOURCE_CRUISE_SPEED_FILE != CRUISE_SPEED_FILE):
                _write_fixed_point_stops_file(SOURCE_CRUISE_SPEED_FILE, stops)
        except (TypeError, ValueError, OSError, yaml.YAMLError) as exc:
            return jsonify({'status': 'error', 'message': str(exc)}), 400
        return jsonify({'status': 'ok', 'fixed_point_stops': stops})

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
        valid = {'tomato', 'wheat', 'strawberry'}
        if crop not in valid:
            return jsonify({'status': 'error',
                            'message': f'Invalid crop type: {crop}. Valid: {sorted(valid)}'}), 400

        # Always update the monitoring plane (latched topic); the streaming
        # diagnosis node reloads its model on change.
        node.current_crop_type = crop
        node.crop_type_pub.publish(String(data=crop))

        # Best-effort forward to mission_control (only up during full stack).
        if node.crop_type_client.wait_for_service(timeout_sec=0.5):
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
            if resp is not None and not resp.success:
                return jsonify({'status': 'error', 'message': resp.message})
        return jsonify({'status': 'ok', 'message': f'Crop type set to {crop}'})

    # Shared mock diagnosis mode (cross-client sync)
    _app.config['mock_diagnosis_mode'] = 'real'

    @_app.route('/mock-diagnosis-mode', methods=['GET'])
    def get_mock_mode():
        return jsonify({'mode': _app.config['mock_diagnosis_mode']})

    @_app.route('/mock-diagnosis-mode', methods=['POST'])
    def set_mock_mode():
        data = request.get_json()
        mode = data.get('mode', 'real')
        if mode not in ('real', 'healthy', 'early_blight', 'leaf_mold'):
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
