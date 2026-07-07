"""Tests for keyboard_control_node cmd_vel publishing behavior."""

import pytest
import rclpy
from unittest.mock import patch, MagicMock

from sentry_mission.keyboard_control_node import KeyboardControlNode


@pytest.fixture(scope='module')
def ros_context():
    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.fixture
def node(ros_context):
    with patch.object(KeyboardControlNode, '_try_disable_mission_control'), \
         patch.object(KeyboardControlNode, '_try_restore_mission_control'):
        n = KeyboardControlNode()
        yield n
        n.destroy_node()


def test_timer_publishes_in_manual_mode(node):
    """timer_cb should publish /cmd_vel when mode is MANUAL."""
    node.mode = 'MANUAL'
    node.linear = 0.3
    node.angular = 0.1

    with patch.object(node.cmd_pub, 'publish') as mock_pub:
        node.timer_cb()

        assert mock_pub.called
        twist = mock_pub.call_args[0][0]
        assert twist.linear.x == 0.3
        assert twist.angular.z == 0.1


def test_timer_does_not_publish_in_auto_mode(node):
    """timer_cb should NOT publish /cmd_vel when mode is AUTO."""
    node.mode = 'AUTO'
    node.linear = 0.3
    node.angular = 0.1

    with patch.object(node.cmd_pub, 'publish') as mock_pub:
        node.timer_cb()

        assert not mock_pub.called


def test_timer_timeout_stops_motors(node):
    """timer_cb should zero out velocity after TIMEOUT without input."""
    import time
    node.mode = 'MANUAL'
    node.linear = 0.3
    node.last_cmd_time = time.time() - 1.0  # Expired

    with patch.object(node.cmd_pub, 'publish') as mock_pub:
        node.timer_cb()

        twist = mock_pub.call_args[0][0]
        assert twist.linear.x == 0.0
        assert twist.angular.z == 0.0
