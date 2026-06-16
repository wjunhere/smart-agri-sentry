import pytest

from sentry_servo.servo_driver import Servo, ServoError


def test_angle_to_duty_ns_at_limits():
    servo = Servo(channel=0, min_angle=0, max_angle=180)
    assert servo.angle_to_duty_ns(0) == 500_000
    assert servo.angle_to_duty_ns(90) == 1_500_000
    assert servo.angle_to_duty_ns(180) == 2_500_000


def test_angle_clamping():
    servo = Servo(channel=0, min_angle=30, max_angle=150)
    assert servo.angle_to_duty_ns(0) == servo.angle_to_duty_ns(30)
    assert servo.angle_to_duty_ns(200) == servo.angle_to_duty_ns(150)


def test_custom_pulse_range():
    servo = Servo(channel=0, min_us=1000, max_us=2000)
    assert servo.angle_to_duty_ns(0) == 1_000_000
    assert servo.angle_to_duty_ns(180) == 2_000_000


def test_sysfs_path_building():
    servo = Servo(channel=1, chip=0)
    assert servo._base == '/sys/class/pwm/pwmchip0'
    assert servo._path == '/sys/class/pwm/pwmchip0/pwm1'
