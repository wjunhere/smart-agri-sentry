import pytest
import rclpy
from unittest.mock import MagicMock, patch

from sentry_servo.servo_driver_node import ServoDriverNode
from sentry_interfaces.msg import ServoCmd


@pytest.fixture(scope='module')
def ros_context():
    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.fixture
def node(ros_context):
    with patch('sentry_servo.servo_driver_node.Servo') as MockServo:
        yaw_mock = MagicMock()
        pitch_mock = MagicMock()
        MockServo.side_effect = [yaw_mock, pitch_mock]
        n = ServoDriverNode()
        yield n
        n.destroy_node()


def test_servo_cmd_sets_pitch_and_yaw(node):
    msg = ServoCmd()
    msg.pitch = 60
    msg.yaw = 120

    node.on_servo_cmd(msg)

    assert node.pitch.set_angle.call_args[0][0] == 60
    assert node.yaw.set_angle.call_args[0][0] == 120


def test_initial_angles_applied():
    with patch('sentry_servo.servo_driver_node.Servo') as MockServo:
        yaw_mock = MagicMock()
        pitch_mock = MagicMock()
        MockServo.side_effect = [yaw_mock, pitch_mock]
        node = ServoDriverNode()
        yaw_mock.set_angle.assert_called_once()
        pitch_mock.set_angle.assert_called_once()
        node.destroy_node()
