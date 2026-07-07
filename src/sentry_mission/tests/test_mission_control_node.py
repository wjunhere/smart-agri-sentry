"""Tests for mission_control_node state machine."""

import pytest
import rclpy
from unittest.mock import patch, MagicMock

from sentry_mission.mission_control_node import MissionControlNode


@pytest.fixture(scope='module')
def ros_context():
    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.fixture
def node(ros_context):
    with patch('nav2_simple_commander.robot_navigator.BasicNavigator'):
        n = MissionControlNode()
        yield n
        n.destroy_node()


def test_resume_does_not_publish_cruise_speed(node):
    """RESUME state should not publish non-zero cmd_vel even after delay expires."""
    node.state = 'RESUME'
    node.state_enter_time = 0.0
    node.resume_delay = 0.0  # Immediately expired

    with patch.object(node.pub_cmd, 'publish') as mock_cmd, \
         patch.object(node, '_transition') as mock_transition, \
         patch.object(node, '_send_next_waypoint') as mock_send:
        node.tick()

        # Should have published a zero Twist (not cruise_speed)
        assert mock_cmd.called
        cmd = mock_cmd.call_args[0][0]
        assert cmd.linear.x == 0.0
        assert cmd.angular.z == 0.0
