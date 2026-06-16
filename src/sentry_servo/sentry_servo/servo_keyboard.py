#!/usr/bin/env python3
"""Standalone keyboard servo controller for RDK X5."""

import argparse
import os
import select
import sys
import termios
import tty

import yaml

from sentry_servo.servo_driver import Servo


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


def _make_servo(cfg, name):
    pwm = cfg['pwm']
    servo = cfg['servos'][name]
    return Servo(
        channel=servo['channel'],
        chip=pwm['chip'],
        freq_hz=pwm['frequency_hz'],
        min_us=pwm['min_pulse_us'],
        max_us=pwm['max_pulse_us'],
        min_angle=servo['min_angle'],
        max_angle=servo['max_angle'],
        name=name,
    )


def _getch(timeout=0.05):
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setcbreak(fd)
        if select.select([sys.stdin], [], [], timeout)[0]:
            return sys.stdin.read(1)
        return None
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)


def main():
    parser = argparse.ArgumentParser(
        description='Keyboard control for RDK X5 PWM servos')
    parser.add_argument(
        '--config', default='config/servo_config.yaml',
        help='Path to servo_config.yaml')
    args = parser.parse_args()

    cfg = _merge_config(_load_config(args.config))

    yaw = _make_servo(cfg, 'yaw')
    pitch = _make_servo(cfg, 'pitch')

    yaw_cfg = cfg['servos']['yaw']
    pitch_cfg = cfg['servos']['pitch']

    yaw.set_angle(yaw_cfg['initial_angle'])
    pitch.set_angle(pitch_cfg['initial_angle'])

    print('Controls: ←/→ yaw  ↑/↓ pitch  r=reset  q/ESC=quit')

    try:
        while True:
            ch = _getch()
            if ch is None:
                continue
            if ch == '\x1b':
                bracket = _getch(0.05)
                if bracket == '[':
                    key = _getch(0.05)
                    if key == 'C':
                        yaw.set_angle(yaw.last_angle + yaw_cfg['step_deg'])
                    elif key == 'D':
                        yaw.set_angle(yaw.last_angle - yaw_cfg['step_deg'])
                    elif key == 'A':
                        pitch.set_angle(pitch.last_angle + pitch_cfg['step_deg'])
                    elif key == 'B':
                        pitch.set_angle(pitch.last_angle - pitch_cfg['step_deg'])
            elif ch.lower() == 'r':
                yaw.set_angle(yaw_cfg['initial_angle'])
                pitch.set_angle(pitch_cfg['initial_angle'])
            elif ch.lower() == 'q' or ch == '\x03':
                break
    finally:
        yaw.disable()
        pitch.disable()
        print('Servos disabled.')


if __name__ == '__main__':
    main()
