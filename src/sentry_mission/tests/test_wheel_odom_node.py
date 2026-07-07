"""Tests for wheel_odom_node chassis status handling."""

import pytest
import rclpy
from unittest.mock import patch

from sentry_mission.wheel_odom_node import WheelOdomNode
from sentry_interfaces.msg import ChassisStatus


@pytest.fixture(scope='module')
def ros_context():
    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.fixture
def node(ros_context):
    n = WheelOdomNode()
    yield n
    n.destroy_node()


def _make_chassis_msg(left_pulse, right_pulse, comm_timeout=False):
    msg = ChassisStatus()
    msg.left_pulse = left_pulse
    msg.right_pulse = right_pulse
    msg.comm_timeout = comm_timeout
    return msg


def test_timeout_frame_publishes_holding_pose(node):
    """Verify comm_timeout=True frames publish a holding odom (to keep EKF alive)."""
    with patch.object(node.pub, 'publish') as mock_pub:
        node.on_chassis(_make_chassis_msg(100, 100, comm_timeout=True))
        assert mock_pub.call_count == 1
        odom = mock_pub.call_args[0][0]
        assert odom.twist.twist.linear.x == 0.0
        assert odom.twist.twist.angular.z == 0.0


def test_valid_frame_after_timeout_resets(node):
    """Verify normal frames resume odometry after timeout frames."""
    with patch.object(node, 'get_clock') as mock_clock, \
         patch.object(node.pub, 'publish') as mock_pub:
        t0 = rclpy.time.Time(seconds=1.0)
        t1 = rclpy.time.Time(seconds=1.05)
        t2 = rclpy.time.Time(seconds=1.10)
        mock_clock.return_value.now.side_effect = [t0, t1, t2]

        node.on_chassis(_make_chassis_msg(100, 100, comm_timeout=True))
        node.on_chassis(_make_chassis_msg(100, 100, comm_timeout=False))
        node.on_chassis(_make_chassis_msg(200, 200, comm_timeout=False))
        # timeout publishes holding pose (1), valid frame publishes odom (1)
        assert mock_pub.call_count == 2


def test_valid_frame_computes_odometry(node):
    """Verify two valid frames produce expected odometry."""
    times = [
        rclpy.time.Time(seconds=1.0),
        rclpy.time.Time(seconds=1.05),
    ]

    with patch.object(node, 'get_clock') as mock_clock, \
            patch.object(node.pub, 'publish') as mock_pub:
        mock_clock.return_value.now.side_effect = times

        node.on_chassis(_make_chassis_msg(100, 100))
        node.on_chassis(_make_chassis_msg(200, 200))

        assert mock_pub.call_count == 1
        odom = mock_pub.call_args[0][0]
        expected_dist = 100.0 / node.pulses_per_m
        expected_linear = expected_dist / 0.05
        assert abs(odom.pose.pose.position.x - expected_dist) < 1e-6
        assert abs(odom.pose.pose.position.y) < 1e-6
        assert abs(odom.twist.twist.linear.x - expected_linear) < 1e-6
        assert abs(odom.twist.twist.angular.z) < 1e-6
        assert abs(odom.pose.pose.orientation.z) < 1e-6
        assert abs(odom.pose.pose.orientation.w - 1.0) < 1e-6


def test_publish_odom_uses_passed_dt(node):
    """Verify _publish_odom uses the caller-provided dt, not a hardcoded value."""
    node.x = node.y = node.theta = 0.0

    with patch.object(node.pub, 'publish') as mock_pub:
        node._publish_odom(
            node.get_clock().now(), dl=0.1, dr=0.1, d_center=0.1, d_theta=0.0, dt=0.1)

        odom = mock_pub.call_args[0][0]
        # d_center=0.1m, dt=0.1s → linear.x = 1.0 m/s
        assert abs(odom.twist.twist.linear.x - 1.0) < 1e-6


def test_publish_odom_rejects_zero_dt(node):
    """Verify _publish_odom with zero dt uses 0 for twist instead of dividing by zero."""
    node.x = node.y = node.theta = 0.0

    with patch.object(node.pub, 'publish') as mock_pub:
        node._publish_odom(
            node.get_clock().now(), dl=0.1, dr=0.1, d_center=0.1, d_theta=0.0, dt=0.0)

        odom = mock_pub.call_args[0][0]
        assert odom.twist.twist.linear.x == 0.0


def test_timeout_updates_last_time(node):
    """Verify comm_timeout updates last_time to prevent dt explosion on recovery."""
    t1 = rclpy.time.Time(seconds=1.0)
    t2 = rclpy.time.Time(seconds=5.0)  # Simulates 4-second gap

    with patch.object(node, 'get_clock') as mock_clock, \
            patch.object(node.pub, 'publish') as mock_pub:
        mock_clock.return_value.now.side_effect = [t1]

        # Send initial valid frame to prime last_left/last_right/last_time
        node.last_left = 100
        node.last_right = 100
        node.last_time = t1

        # Send timeout frame at t2
        mock_clock.return_value.now.side_effect = [t2]
        node.on_chassis(_make_chassis_msg(100, 100, comm_timeout=True))

        # After timeout, last_time should be t2 (not t1)
        assert node.last_time == t2
