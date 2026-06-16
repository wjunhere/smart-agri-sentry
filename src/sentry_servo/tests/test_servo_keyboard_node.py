import pytest
import rclpy
from unittest.mock import MagicMock

from sentry_servo.servo_keyboard_node import ServoKeyboardNode
from sentry_interfaces.msg import ServoCmd


@pytest.fixture(scope='module')
def ros_context():
    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.fixture
def node(ros_context):
    n = ServoKeyboardNode(config_path=None, verbose=False)
    n.pub = MagicMock()
    yield n
    n.destroy_node()


def test_initial_angles_published(node):
    assert node.yaw_angle == 90.0
    assert node.pitch_angle == 90.0
    assert node.pub.publish.call_count == 1
    msg = node.pub.publish.call_args[0][0]
    assert isinstance(msg, ServoCmd)
    assert msg.yaw == 90
    assert msg.pitch == 90


def test_move_yaw_clamps_and_publishes(node):
    node._move_yaw(100)
    assert node.yaw_angle == 180.0
    node._move_yaw(-300)
    assert node.yaw_angle == 0.0
    published = [call[0][0] for call in node.pub.publish.call_args_list]
    assert published[-1].yaw == 0


def test_move_pitch_respects_limits(node):
    node._move_pitch(100)
    assert node.pitch_angle == 150.0
    node._move_pitch(-300)
    assert node.pitch_angle == 30.0


def test_reset_restores_initial_angles(node):
    node._move_yaw(10)
    node._move_pitch(10)
    node._reset()
    assert node.yaw_angle == 90.0
    assert node.pitch_angle == 90.0
