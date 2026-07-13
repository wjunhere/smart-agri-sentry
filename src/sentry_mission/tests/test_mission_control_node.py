"""Tests for mission_control_node state machine."""

import pytest
import rclpy
from unittest.mock import patch, MagicMock

from sentry_mission.mission_control_node import MissionControlNode
from nav2_simple_commander.robot_navigator import TaskResult


@pytest.fixture(scope='module')
def ros_context():
    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.fixture
def node(ros_context):
    with patch('sentry_mission.mission_control_node.BasicNavigator'):
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

        assert mock_cmd.called
        cmd = mock_cmd.call_args[0][0]
        assert cmd.linear.x == 0.0
        assert cmd.angular.z == 0.0


def test_nav2_task_failed_retries_same_waypoint(node):
    """PATROL state: Nav2 failure keeps waypoint and schedules a delayed retry."""
    node.state = 'PATROL'
    node._nav2_ready = True
    node.current_wp_idx = 1
    node.sending_goal = True
    node.waypoints = [
        {'x': 0.0, 'y': 0.0, 'yaw': 0.0},
        {'x': 1.0, 'y': 0.0, 'yaw': 0.0},
        {'x': 2.0, 'y': 0.0, 'yaw': 0.0},
    ]

    with patch.object(node.navigator, 'isTaskComplete', return_value=True), \
         patch.object(node.navigator, 'getResult', return_value=TaskResult.FAILED), \
         patch.object(node, '_send_next_waypoint') as mock_send:
        node.tick()

        assert node.current_wp_idx == 1
        assert node.sending_goal == False
        assert node._next_goal_time > 0.0
        mock_send.assert_not_called()


def test_nav2_task_succeeded_advances_waypoint(node):
    """PATROL state: successful Nav2 task advances to next waypoint."""
    node.state = 'PATROL'
    node._nav2_ready = True
    node.current_wp_idx = 1
    node.sending_goal = True
    node.waypoints = [
        {'x': 0.0, 'y': 0.0, 'yaw': 0.0},
        {'x': 1.0, 'y': 0.0, 'yaw': 0.0},
        {'x': 2.0, 'y': 0.0, 'yaw': 0.0},
    ]

    with patch.object(node.navigator, 'isTaskComplete', return_value=True), \
         patch.object(node.navigator, 'getResult', return_value=TaskResult.SUCCEEDED):
        node.tick()

    assert node.current_wp_idx == 2
    assert node.sending_goal == True  # Re-set by _send_next_waypoint


def test_auto_mode_prepares_autonomous_start(node):
    """AUTO transition should reset odometry/encoders before sending a goal."""
    from std_srvs.srv import SetBool

    node.state = 'MANUAL'
    node.saved_wp_idx = 1
    node.current_wp_idx = 0
    node.sending_goal = True

    with patch.object(node, '_prepare_autonomous_start') as mock_prepare:
        request = SetBool.Request()
        request.data = True
        response = node.set_auto_mode_cb(request, SetBool.Response())

    assert response.success is True
    assert node.state == 'PATROL'
    assert node.current_wp_idx == 1
    mock_prepare.assert_called_once()


def test_prepare_autonomous_start_calls_reset_services(node):
    """Verify autonomous start asynchronously requests odom, encoder, and EKF resets."""
    reset_odom = MagicMock()
    reset_encoder = MagicMock()
    set_pose = MagicMock()
    reset_odom.wait_for_service.return_value = True
    reset_encoder.wait_for_service.return_value = True
    set_pose.wait_for_service.return_value = True
    reset_odom.call_async.return_value.add_done_callback = MagicMock()
    reset_encoder.call_async.return_value.add_done_callback = MagicMock()
    set_pose.call_async.return_value.add_done_callback = MagicMock()
    node.reset_wheel_odom_client = reset_odom
    node.reset_encoder_client = reset_encoder
    node.set_pose_client = set_pose
    node.sending_goal = True

    with patch.object(node.pub_cmd, 'publish') as mock_stop:
        node._prepare_autonomous_start()

    assert node.sending_goal is False
    assert node._next_goal_time > 0.0
    assert mock_stop.called
    reset_odom.wait_for_service.assert_called_once()
    reset_encoder.wait_for_service.assert_called_once()
    reset_odom.call_async.assert_called_once()
    reset_encoder.call_async.assert_called_once()
    set_pose.wait_for_service.assert_called_once()
    set_pose.call_async.assert_called_once()
    pose_req = set_pose.call_async.call_args[0][0]
    assert pose_req.pose.header.frame_id == 'odom'
    assert pose_req.pose.pose.pose.position.x == 0.0
    assert pose_req.pose.pose.pose.position.y == 0.0
    assert pose_req.pose.pose.pose.orientation.w == 1.0
