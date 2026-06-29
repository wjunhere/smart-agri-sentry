import pytest
import rclpy
from unittest.mock import patch

from sentry_sensors.uart_bridge_node import UartBridgeNode


@pytest.fixture(scope='module')
def ros_context():
    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.fixture
def node(ros_context):
    with patch('sentry_sensors.uart_bridge_node.serial.Serial'):
        n = UartBridgeNode()
        yield n
        n.destroy_node()


def test_servo_subscription_disabled_by_default(node):
    assert not hasattr(node, 'sub_servo')


def test_servo_subscription_enabled_when_configured(ros_context):
    with patch('sentry_sensors.uart_bridge_node.serial.Serial'):
        n = UartBridgeNode(parameter_overrides=[
            rclpy.parameter.Parameter(
                'forward_servo_cmd', rclpy.Parameter.Type.BOOL, True),
        ])
        assert hasattr(n, 'sub_servo')
        n.destroy_node()


def test_cmd_vel_conversion(node):
    """Verify Twist is converted to differential-drive wheel speeds."""
    import struct
    from geometry_msgs.msg import Twist

    node.wheel_base = 0.23
    msg = Twist()
    msg.linear.x = 0.5
    msg.angular.z = 0.0

    with patch.object(node.ser, 'write') as mock_write:
        node.on_cmd_vel(msg)
        assert mock_write.called
        frame = mock_write.call_args[0][0]
        assert frame[0:2] == b'\xaa\x55'
        assert frame[2] == 0x81
        left, right = struct.unpack('<hh', frame[4:8])
        assert left == 500
        assert right == 500
