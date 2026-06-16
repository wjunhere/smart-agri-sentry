#!/usr/bin/env python3
"""ROS2 keyboard node that publishes /sentry/servo_cmd."""

import argparse
import os
import select
import sys
import termios
import tty

import rclpy
from rclpy.node import Node
import yaml

from sentry_interfaces.msg import ServoCmd


def _default_config():
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
                'step_deg': 5,
            },
            'pitch': {
                'channel': 1,
                'min_angle': 30,
                'max_angle': 150,
                'initial_angle': 90,
                'step_deg': 5,
            },
        },
    }


def _load_config(path):
    if not path or not os.path.exists(path):
        return {}
    with open(path, 'r') as f:
        return yaml.safe_load(f) or {}


def _merge_config(user):
    cfg = _default_config()
    if not user:
        return cfg
    pwm = user.get('pwm', {})
    for key in cfg['pwm']:
        cfg['pwm'][key] = pwm.get(key, cfg['pwm'][key])
    for name in ('yaw', 'pitch'):
        servo = user.get('servos', {}).get(name, {})
        cfg['servos'][name].update(servo)
    return cfg


def _clamp(value, min_value, max_value):
    return max(min_value, min(max_value, float(value)))


def _set_raw_mode(fd):
    old = termios.tcgetattr(fd)
    tty.setcbreak(fd)
    return old


def _restore_mode(fd, old):
    termios.tcsetattr(fd, termios.TCSADRAIN, old)


def _getch(fd, timeout=None):
    esc_timeout = 0.5
    if not select.select([sys.stdin], [], [], timeout)[0]:
        return None
    ch = os.read(fd, 1).decode('utf-8', errors='replace')
    if ch != '\x1b':
        return ch
    if select.select([sys.stdin], [], [], esc_timeout)[0]:
        ch += os.read(fd, 1).decode('utf-8', errors='replace')
    if len(ch) == 2 and ch[1] == '[':
        if select.select([sys.stdin], [], [], esc_timeout)[0]:
            ch += os.read(fd, 1).decode('utf-8', errors='replace')
    return ch


class ServoKeyboardNode(Node):
    """Publish servo angles from keyboard input."""

    def __init__(self, config_path=None, verbose=False):
        super().__init__('servo_keyboard_node')
        self.verbose = verbose

        cfg = _merge_config(_load_config(config_path))
        self.yaw_cfg = cfg['servos']['yaw']
        self.pitch_cfg = cfg['servos']['pitch']

        self.yaw_angle = float(self.yaw_cfg['initial_angle'])
        self.pitch_angle = float(self.pitch_cfg['initial_angle'])

        self.pub = self.create_publisher(ServoCmd, '/sentry/servo_cmd', 10)
        self._publish()

        self.get_logger().info(
            'Keyboard servo node ready: '
            '←/→ yaw  ↑/↓ pitch  r=reset  q/ESC=quit')
        if self.verbose:
            self.get_logger().info(
                f'Initial: yaw={self.yaw_angle}, pitch={self.pitch_angle}')

    def _publish(self):
        msg = ServoCmd()
        msg.yaw = int(self.yaw_angle)
        msg.pitch = int(self.pitch_angle)
        self.pub.publish(msg)
        if self.verbose:
            self.get_logger().info(
                f'publish: yaw={msg.yaw}, pitch={msg.pitch}')

    def _move_yaw(self, delta):
        self.yaw_angle = _clamp(
            self.yaw_angle + delta,
            self.yaw_cfg['min_angle'],
            self.yaw_cfg['max_angle'])
        self._publish()

    def _move_pitch(self, delta):
        self.pitch_angle = _clamp(
            self.pitch_angle + delta,
            self.pitch_cfg['min_angle'],
            self.pitch_cfg['max_angle'])
        self._publish()

    def _reset(self):
        self.yaw_angle = float(self.yaw_cfg['initial_angle'])
        self.pitch_angle = float(self.pitch_cfg['initial_angle'])
        self._publish()

    def run(self):
        fd = sys.stdin.fileno()
        old_term = _set_raw_mode(fd)
        try:
            while rclpy.ok():
                ch = _getch(fd)
                if ch is None:
                    continue
                if self.verbose:
                    self.get_logger().info(f'key: {ch!r}')
                if ch.startswith('\x1b[') and len(ch) == 3:
                    key = ch[2]
                    if key == 'C':
                        self._move_yaw(self.yaw_cfg['step_deg'])
                    elif key == 'D':
                        self._move_yaw(-self.yaw_cfg['step_deg'])
                    elif key == 'A':
                        self._move_pitch(self.pitch_cfg['step_deg'])
                    elif key == 'B':
                        self._move_pitch(-self.pitch_cfg['step_deg'])
                elif ch.lower() == 'r':
                    self._reset()
                elif ch.lower() == 'q' or ch == '\x03':
                    break
        finally:
            _restore_mode(fd, old_term)


def main(args=None):
    parser = argparse.ArgumentParser(
        description='Keyboard servo publisher for RDK X5')
    parser.add_argument(
        '--config', default='config/servo_config.yaml',
        help='Path to servo_config.yaml')
    parser.add_argument(
        '--verbose', '-v', action='store_true',
        help='Print key codes and servo angles')
    app_args, _ = parser.parse_known_args()

    rclpy.init(args=args)
    node = ServoKeyboardNode(
        config_path=app_args.config, verbose=app_args.verbose)
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
