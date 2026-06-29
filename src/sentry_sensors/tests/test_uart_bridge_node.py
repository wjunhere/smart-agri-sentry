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


def test_decode_chassis_status_signed_pulses():
    """Verify 19-byte chassis status parses left/right pulses as signed int32.

    Using a negative right_pulse ensures the decoder uses 'i' (signed) rather
    than 'I' (unsigned); with 'I' the value would be misread as ~4.29e9.
    """
    from sentry_sensors.uart_bridge_node import encode_frame, decode_chassis_frame
    import struct

    payload = struct.pack('<hhHBiiI',
                          500, -300, 1234, 0x04,
                          100000, -100000, 0x12345678)
    frame = encode_frame(0x03, payload)
    data = decode_chassis_frame(frame)
    assert data is not None
    assert data['left_speed'] == 0.5
    assert data['right_speed'] == -0.3
    assert data['battery_voltage'] == 12.34
    assert data['alarm_bits'] == 0x04
    assert data['left_pulse'] == 100000
    assert data['right_pulse'] == -100000
    assert data['encoder_timestamp'] == 0x12345678


def test_decode_chassis_status_negative_one_pulse():
    """Verify -1 pulse decodes correctly (boundary for signed vs unsigned)."""
    from sentry_sensors.uart_bridge_node import encode_frame, decode_chassis_frame
    import struct

    payload = struct.pack('<hhHBiiI',
                          0, 0, 0, 0,
                          -1, -1, 0)
    frame = encode_frame(0x03, payload)
    data = decode_chassis_frame(frame)
    assert data is not None
    assert data['left_pulse'] == -1
    assert data['right_pulse'] == -1
