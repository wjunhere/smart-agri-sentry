import os

import rclpy
from rclpy.node import Node
import yaml

from sentry_interfaces.msg import ServoCmd
from sentry_servo.servo_driver import Servo


class ServoDriverNode(Node):
    """ROS2 node that drives PWM servos from /sentry/servo_cmd."""

    def __init__(self):
        super().__init__('servo_driver_node')
        self.declare_parameter('config_path', '')

        cfg = self._load_config(self.get_parameter('config_path').value)
        pwm_cfg = cfg.get('pwm', {})
        servos_cfg = cfg.get('servos', {})

        self.yaw = self._create_servo(servos_cfg.get('yaw', {}), pwm_cfg)
        self.pitch = self._create_servo(servos_cfg.get('pitch', {}), pwm_cfg)

        self.sub = self.create_subscription(
            ServoCmd, '/sentry/servo_cmd', self.on_servo_cmd, 10)

        self.yaw.set_angle(servos_cfg.get('yaw', {}).get('initial_angle', 90))
        self.pitch.set_angle(
            servos_cfg.get('pitch', {}).get('initial_angle', 90))

        self.get_logger().info('Servo driver node ready')

    def _default_config(self):
        return {
            'pwm': {
                'chip': 0,
                'frequency_hz': 50,
                'min_pulse_us': 500,
                'max_pulse_us': 2500,
            },
            'servos': {
                'yaw': {
                    'channel': 0,
                    'min_angle': 0,
                    'max_angle': 180,
                    'initial_angle': 90,
                },
                'pitch': {
                    'channel': 1,
                    'min_angle': 30,
                    'max_angle': 150,
                    'initial_angle': 90,
                },
            },
        }

    def _load_config(self, path):
        if not path:
            return self._default_config()
        if not os.path.isabs(path):
            candidates = [
                path,
                os.path.join(os.getcwd(), path),
                os.path.join(
                    os.path.dirname(__file__), '..', '..', '..', path),
            ]
            for c in candidates:
                if os.path.exists(c):
                    path = c
                    break
        if os.path.exists(path):
            with open(path, 'r') as f:
                return yaml.safe_load(f) or {}
        self.get_logger().warn(f'Config not found: {path}, using defaults')
        return self._default_config()

    def _create_servo(self, servo_cfg, pwm_cfg):
        return Servo(
            channel=servo_cfg.get('channel', 0),
            chip=pwm_cfg.get('chip', 0),
            freq_hz=pwm_cfg.get('frequency_hz', 50),
            min_us=pwm_cfg.get('min_pulse_us', 500),
            max_us=pwm_cfg.get('max_pulse_us', 2500),
            min_angle=servo_cfg.get('min_angle', 0),
            max_angle=servo_cfg.get('max_angle', 180),
            name=servo_cfg.get('name', 'servo'),
        )

    def on_servo_cmd(self, msg: ServoCmd):
        self.pitch.set_angle(float(msg.pitch))
        self.yaw.set_angle(float(msg.yaw))

    def destroy_node(self):
        self.yaw.disable()
        self.pitch.disable()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ServoDriverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
