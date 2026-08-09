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
    from sentry_sensors.uart_bridge_node import TYPE_MODE_CMD, TYPE_MOTION_CMD

    node.wheel_base = 0.23
    msg = Twist()
    msg.linear.x = 0.5
    msg.angular.z = 0.0

    with patch.object(node.ser, 'write') as mock_write:
        node.on_cmd_vel(msg)
        assert mock_write.called
        assert mock_write.call_count == 2

        mode_frame = mock_write.call_args_list[0][0][0]
        assert mode_frame[0:2] == b'\xaa\x55'
        assert mode_frame[2] == TYPE_MODE_CMD
        assert mode_frame[3] == 1
        assert mode_frame[4] == 0x02

        frame = mock_write.call_args_list[1][0][0]
        assert frame[0:2] == b'\xaa\x55'
        assert frame[2] == TYPE_MOTION_CMD
        assert frame[3] == 4
        left, right = struct.unpack('<hh', frame[4:8])
        assert left == 500
        assert right == 500


def test_cmd_vel_mode_frame_not_repeated(node):
    """Verify repeated Twist commands do not spam mode frames."""
    from geometry_msgs.msg import Twist
    from sentry_sensors.uart_bridge_node import TYPE_MOTION_CMD

    msg = Twist()
    msg.linear.x = 0.2

    with patch.object(node.ser, 'write') as mock_write:
        node.on_cmd_vel(msg)
        node.on_cmd_vel(msg)

    frame_types = [call[0][0][2] for call in mock_write.call_args_list]
    assert frame_types == [0x83, TYPE_MOTION_CMD, TYPE_MOTION_CMD]


def test_cmd_vel_applies_track_speed_scale(node):
    """Verify per-track trim can compensate mechanical speed mismatch."""
    import struct
    from geometry_msgs.msg import Twist

    node.left_speed_scale = 1.0
    node.right_speed_scale = 0.95

    msg = Twist()
    msg.linear.x = 0.5

    with patch.object(node.ser, 'write') as mock_write:
        node.on_cmd_vel(msg)
        frame = mock_write.call_args[0][0]
        left, right = struct.unpack('<hh', frame[4:8])

    assert left == 500
    assert right == 475


def test_cmd_vel_can_swap_wheel_commands(node):
    """Verify hardware deployments can swap left/right command channels."""
    import struct
    from geometry_msgs.msg import Twist

    node.left_speed_scale = 1.0
    node.right_speed_scale = 0.9
    node.swap_wheel_commands = True

    msg = Twist()
    msg.linear.x = 0.5

    with patch.object(node.ser, 'write') as mock_write:
        node.on_cmd_vel(msg)
        frame = mock_write.call_args[0][0]
        left, right = struct.unpack('<hh', frame[4:8])

    assert left == 450
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


def test_cmd_vel_ignores_non_finite(node):
    """Verify non-finite Twist values do not crash the node or write to UART."""
    from geometry_msgs.msg import Twist

    with patch.object(node.ser, 'write') as mock_write:
        for bad_val in [float('nan'), float('inf'), float('-inf')]:
            msg = Twist()
            msg.linear.x = bad_val
            msg.angular.z = 0.0
            node.on_cmd_vel(msg)
            assert not mock_write.called

            msg = Twist()
            msg.linear.x = 0.0
            msg.angular.z = bad_val
            node.on_cmd_vel(msg)
            assert not mock_write.called


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


def test_chassis_timeout_publishes_timeout_flag(node):
    """Verify no frame for >1s publishes ChassisStatus with comm_timeout=True."""
    from rclpy.duration import Duration
    from sentry_interfaces.msg import ChassisStatus

    node.last_chassis_time = node.get_clock().now() - Duration(seconds=2)
    node.chassis_timed_out = False

    with patch.object(node.pub_chassis, 'publish') as mock_pub:
        node.check_chassis_timeout()
        assert mock_pub.called
        published = mock_pub.call_args[0][0]
        assert isinstance(published, ChassisStatus)
        assert published.comm_timeout is True
        assert published.left_pulse == 0
        assert published.right_pulse == 0


def test_valid_chassis_frame_resets_timeout(node):
    """Verify a valid chassis frame clears timed-out state and comm_timeout=False."""
    from sentry_sensors.uart_bridge_node import encode_frame
    from sentry_interfaces.msg import ChassisStatus
    import struct

    payload = struct.pack('<hhHBiiI',
                          500, -300, 1234, 0x04,
                          100000, -100000, 0x12345678)
    frame = encode_frame(0x03, payload)
    node.chassis_timed_out = True

    with patch.object(node.pub_chassis, 'publish') as mock_pub:
        node.handle_frame(frame)
        assert node.chassis_timed_out is False
        published = mock_pub.call_args[0][0]
        assert isinstance(published, ChassisStatus)
        assert published.comm_timeout is False


def test_invalid_chassis_frame_is_discarded(node):
    """Verify a corrupted chassis frame is not published."""
    from sentry_sensors.uart_bridge_node import encode_frame
    import struct

    payload = struct.pack('<hhHBiiI',
                          500, -300, 1234, 0x04,
                          100000, -100000, 0x12345678)
    frame = bytearray(encode_frame(0x03, payload))
    # Corrupt the payload so CRC fails
    frame[-3] ^= 0xFF

    with patch.object(node.pub_chassis, 'publish') as mock_pub:
        node.handle_frame(bytes(frame))
        assert not mock_pub.called


def test_swap_encoder_channels_swaps_pulses_and_speeds(ros_context):
    """With swap_encoder_channels=True the published ChassisStatus maps the
    firmware's crossed channels back onto physical left/right."""
    from sentry_sensors.uart_bridge_node import encode_frame
    import struct

    with patch('sentry_sensors.uart_bridge_node.serial.Serial'):
        n = UartBridgeNode(parameter_overrides=[
            rclpy.parameter.Parameter(
                'swap_encoder_channels', rclpy.Parameter.Type.BOOL, True),
        ])
    payload = struct.pack('<hhHBiiI',
                          500, -300, 1234, 0x04,
                          100000, -100000, 0x12345678)
    frame = encode_frame(0x03, payload)
    with patch.object(n.pub_chassis, 'publish') as mock_pub:
        n.handle_frame(frame)
    published = mock_pub.call_args[0][0]
    assert published.left_speed == -0.3
    assert published.right_speed == 0.5
    assert published.left_pulse == -100000
    assert published.right_pulse == 100000
    assert published.battery_voltage == 12.34
    n.destroy_node()


def test_swap_encoder_channels_off_by_default(node):
    """Default wiring assumption: firmware left = physical left, no swap."""
    from sentry_sensors.uart_bridge_node import encode_frame
    import struct

    assert node.swap_encoder_channels is False
    payload = struct.pack('<hhHBiiI',
                          500, -300, 1234, 0x04,
                          100000, -100000, 0x12345678)
    frame = encode_frame(0x03, payload)
    with patch.object(node.pub_chassis, 'publish') as mock_pub:
        node.handle_frame(frame)
    published = mock_pub.call_args[0][0]
    assert published.left_pulse == 100000
    assert published.right_pulse == -100000


def test_prolonged_timeout_reopens_serial(node):
    """After chassis_reopen_after_sec without frames the port is reopened."""
    from rclpy.duration import Duration

    node.last_chassis_time = node.get_clock().now() - Duration(seconds=30)
    node._last_reopen_monotonic = 0.0
    with patch('sentry_sensors.uart_bridge_node.serial.Serial') as mock_serial, \
         patch.object(node.pub_chassis, 'publish'):
        node.check_chassis_timeout()
        assert mock_serial.called  # port reopened


def test_reopen_has_cooldown(node):
    """A second prolonged timeout within the window must not reopen again."""
    import time as _time
    from rclpy.duration import Duration

    node.last_chassis_time = node.get_clock().now() - Duration(seconds=30)
    node._last_reopen_monotonic = _time.monotonic()  # just reopened
    with patch('sentry_sensors.uart_bridge_node.serial.Serial') as mock_serial, \
         patch.object(node.pub_chassis, 'publish'):
        node.check_chassis_timeout()
        assert not mock_serial.called
