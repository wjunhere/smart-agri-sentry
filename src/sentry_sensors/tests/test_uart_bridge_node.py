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
        assert frame[3] == 4
        left, right = struct.unpack('<hh', frame[4:8])
        assert left == 500
        assert right == 500


def test_cmd_vel_turn_in_place(node):
    """Verify pure rotation produces opposite wheel speeds."""
    import struct
    from geometry_msgs.msg import Twist

    node.wheel_base = 0.23
    msg = Twist()
    msg.linear.x = 0.0
    msg.angular.z = 1.0

    with patch.object(node.ser, 'write') as mock_write:
        node.on_cmd_vel(msg)
        frame = mock_write.call_args[0][0]
        left, right = struct.unpack('<hh', frame[4:8])
        # v_left = -w*L/2, v_right = +w*L/2
        assert left == -115
        assert right == 115


def test_cmd_vel_reverse(node):
    """Verify negative linear speed produces negative wheel speeds."""
    import struct
    from geometry_msgs.msg import Twist

    node.wheel_base = 0.23
    msg = Twist()
    msg.linear.x = -0.3
    msg.angular.z = 0.0

    with patch.object(node.ser, 'write') as mock_write:
        node.on_cmd_vel(msg)
        frame = mock_write.call_args[0][0]
        left, right = struct.unpack('<hh', frame[4:8])
        assert left == -300
        assert right == -300


def test_cmd_vel_saturates_to_int16(node):
    """Verify wheel speeds are clamped to int16 range."""
    import struct
    from geometry_msgs.msg import Twist

    node.wheel_base = 0.23
    msg = Twist()
    msg.linear.x = 50.0  # 50000 mm/s, exceeds int16 max
    msg.angular.z = 0.0

    with patch.object(node.ser, 'write') as mock_write:
        node.on_cmd_vel(msg)
        frame = mock_write.call_args[0][0]
        left, right = struct.unpack('<hh', frame[4:8])
        assert left == 32767
        assert right == 32767

    msg.linear.x = -50.0  # -50000 mm/s, below int16 min
    with patch.object(node.ser, 'write') as mock_write:
        node.on_cmd_vel(msg)
        frame = mock_write.call_args[0][0]
        left, right = struct.unpack('<hh', frame[4:8])
        assert left == -32768
        assert right == -32768


def test_cmd_vel_no_serial(node):
    """Verify on_cmd_vel returns early when serial is unavailable."""
    from geometry_msgs.msg import Twist

    node.ser = None
    msg = Twist()
    msg.linear.x = 0.5
    # encode_frame should not be invoked without serial; no exception means success.
    node.on_cmd_vel(msg)


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
