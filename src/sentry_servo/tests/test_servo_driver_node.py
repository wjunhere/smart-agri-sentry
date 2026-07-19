import pytest
import rclpy
from unittest.mock import MagicMock, patch

from sentry_servo.servo_driver_node import ServoDriverNode
from sentry_interfaces.msg import ServoCmd


DUAL_AXIS_CONFIG = {
    'pwm': {'chip': 0, 'frequency_hz': 50},
    'servos': {
        'yaw': {'channel': 0, 'initial_angle': 90},
        'pitch': {'channel': 1, 'initial_angle': 90},
    },
}


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
        with patch.object(ServoDriverNode, '_load_config',
                          return_value=DUAL_AXIS_CONFIG):
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
        with patch.object(ServoDriverNode, '_load_config',
                          return_value=DUAL_AXIS_CONFIG):
            node = ServoDriverNode()
            yaw_mock.set_angle.assert_called_once()
            pitch_mock.set_angle.assert_called_once()
            node.destroy_node()


def test_single_axis_config_ignores_pitch_commands(ros_context):
    config = {
        'pwm': {'chip': 0, 'frequency_hz': 50},
        'servos': {'yaw': {'channel': 0, 'initial_angle': 67.5}},
    }
    with patch('sentry_servo.servo_driver_node.Servo') as MockServo, \
         patch.object(ServoDriverNode, '_load_config', return_value=config):
        yaw_mock = MagicMock()
        MockServo.return_value = yaw_mock
        node = ServoDriverNode()
        msg = ServoCmd()
        msg.yaw = 80
        msg.pitch = 120

        node.on_servo_cmd(msg)

        assert node.pitch is None
        assert yaw_mock.set_angle.call_args_list[-1].args == (80.0,)
        node.destroy_node()
