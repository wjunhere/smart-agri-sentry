#!/usr/bin/env python3
"""Web remote control node.

Flask-based HTTP API for manual robot control and mode switching.
Serves a simple remote control page at port 5000.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_srvs.srv import SetBool
from sentry_interfaces.srv import SetCropType
import threading
import time
from pathlib import Path

# Defer Flask import to avoid import issues when not running
_app = None


class WebRemoteNode(Node):
    def __init__(self):
        super().__init__('web_remote_node')
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.mode_srv = self.create_client(SetBool, '/set_auto_mode')
        self.crop_type_client = self.create_client(SetCropType, '/set_crop_type')

        self.declare_parameter('max_linear', 0.5)
        self.declare_parameter('max_angular', 1.0)
        self.max_linear = self.get_parameter('max_linear').value
        self.max_angular = self.get_parameter('max_angular').value

        # Default to AUTO to match mission_control_node
        self.mode = 'AUTO'
        self.linear = 0.0
        self.angular = 0.0
        self.lock = threading.Lock()
        self.last_cmd_time = time.time()
        self.TIMEOUT = 0.5
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
            if not auto:
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
            }


def _get_app(node: WebRemoteNode):
    """Lazy Flask app creation."""
    global _app
    if _app is not None:
        return _app

    from flask import Flask, request, jsonify, send_from_directory
    _app = Flask(__name__)
    STATIC_DIR = Path(__file__).parent / 'static'

    @_app.route('/')
    def index():
        return send_from_directory(str(STATIC_DIR), 'index.html')

    @_app.route('/mode', methods=['POST'])
    def set_mode():
        data = request.get_json()
        auto = data.get('auto', False)
        ok = node.set_mode_auto(auto)
        return jsonify({
            'status': 'ok' if ok else 'error',
            'mode': 'AUTO' if auto else 'MANUAL'
        })

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

    return _app


def _start_flask(node: WebRemoteNode):
    app = _get_app(node)
    app.run(host='0.0.0.0', port=5000, debug=False, use_reloader=False)


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
