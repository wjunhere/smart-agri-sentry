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


def test_timeout_frame_skips_odometry(node):
    """Verify comm_timeout=True frames do not publish odometry."""
    with patch.object(node.pub, 'publish') as mock_pub:
        node.on_chassis(_make_chassis_msg(100, 100, comm_timeout=True))
        assert not mock_pub.called


def test_valid_frame_after_timeout_resets(node):
    """Verify normal frames resume odometry after timeout frames."""
    with patch.object(node.pub, 'publish') as mock_pub:
        node.on_chassis(_make_chassis_msg(100, 100, comm_timeout=True))
        node.on_chassis(_make_chassis_msg(100, 100, comm_timeout=False))
        node.on_chassis(_make_chassis_msg(200, 200, comm_timeout=False))
        assert mock_pub.call_count == 1


def test_valid_frame_computes_odometry(node):
    """Verify two valid frames produce expected odometry."""
    times = [
        rclpy.time.Time(seconds=1.0),
        rclpy.time.Time(seconds=1.05),
    ]

    with patch.object(node, 'get_clock') as mock_clock, \
            patch.object(node.pub, 'publish') as mock_pub:
        mock_clock.return_value.now.side_effect = times

        node.on_chassis(_make_chassis_msg(11035, 11035))
        node.on_chassis(_make_chassis_msg(22070, 22070))

        assert mock_pub.call_count == 1
        odom = mock_pub.call_args[0][0]
        # Both wheels moved 1m forward => x=1, theta=0
        assert abs(odom.pose.pose.position.x - 1.0) < 1e-6
        assert abs(odom.pose.pose.position.y) < 1e-6
        assert abs(odom.twist.twist.linear.x - 20.0) < 1e-6
        assert abs(odom.twist.twist.angular.z) < 1e-6
        assert abs(odom.pose.pose.orientation.z) < 1e-6
        assert abs(odom.pose.pose.orientation.w - 1.0) < 1e-6
