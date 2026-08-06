import struct

import pytest
import rclpy
from unittest.mock import patch, MagicMock

from sentry_sensors.lora_bridge_node import (
    crc16_ccitt,
    decode_environment_payload,
    decode_cj702_payload,
    LoraBridgeNode,
    MOCK_BASELINE,
    MOCK_JITTER,
    FRAME_TYPE_DATA,
    STATUS_NODE_OK,
    STATUS_SRC_ACTIVE,
)


@pytest.fixture(scope='module')
def ros_context():
    rclpy.init()
    yield
    rclpy.shutdown()


def _build_frame(payload: bytes, frame_type=FRAME_TYPE_DATA,
                 status=STATUS_NODE_OK | STATUS_SRC_ACTIVE, seq=0):
    header = bytes([0xAA, frame_type | status, seq, 0x00, len(payload)])
    frame = header + payload
    return frame + struct.pack('>H', crc16_ccitt(frame[1:]))


def _build_env_frame(values, **kwargs):
    return _build_frame(struct.pack('>HHHHHHHHHHHH', *values), **kwargs)


def _build_cj702_frame(values, **kwargs):
    return _build_frame(struct.pack('>HHHHHHH', *values), **kwargs)


def test_crc16_ccitt_empty():
    assert crc16_ccitt(b'') == 0xFFFF


def test_crc16_ccitt_known_vector():
    # CRC-16/CCITT-FALSE of "123456789" is 0x29B1
    assert crc16_ccitt(b'123456789') == 0x29B1


def test_decode_environment_payload_valid():
    payload = struct.pack('>HHHHHHHHHHHH', *[
        303, 20, 120, 35, 50,          # co2, hcho, tvoc, pm25, pm10
        2700, 6000,                    # air_temp, air_humidity
        2500, 5000,                    # soil_temp, soil_humidity
        150,                           # ec
        7000,                          # leaf_wetness
        2800,                          # leaf_temp
    ])
    data = decode_environment_payload(payload)
    assert data['air_co2'] == 303.0
    assert data['hcho'] == 20.0
    assert data['tvoc'] == 120.0
    assert data['pm25'] == 35.0
    assert data['pm10'] == 50.0
    assert data['air_temp'] == 27.0
    assert data['air_humidity'] == 60.0
    assert data['soil_temp'] == 25.0
    assert data['soil_humidity'] == 50.0
    assert data['ec'] == 150.0
    assert data['leaf_wetness'] == 70.0
    assert data['leaf_temp'] == 28.0


def test_decode_environment_payload_negative_temperatures():
    payload = struct.pack('>HHHHHHHHHHHH', *[
        0, 0, 0, 0, 0,
        0xF63C, 0,      # -25.00 C
        0xF63C, 0,      # -25.00 C
        0,
        0,
        0xF63C,         # -25.00 C
    ])
    data = decode_environment_payload(payload)
    assert data['air_temp'] == -25.0
    assert data['soil_temp'] == -25.0
    assert data['leaf_temp'] == -25.0


def test_decode_environment_payload_wrong_length():
    assert decode_environment_payload(b'\x00' * 10) is None


def test_decode_cj702_payload_valid():
    payload = struct.pack('>HHHHHHH', *[
        450,   # co2 ppm
        15,    # hcho raw
        80,    # tvoc ppb
        25,    # pm25 ug/m3
        40,    # pm10 ug/m3
        2650,  # air_temp 26.5 C
        5500,  # air_humidity 55.0% RH
    ])
    data = decode_cj702_payload(payload)
    assert data['air_co2'] == 450.0
    assert data['hcho'] == 15.0
    assert data['tvoc'] == 80.0
    assert data['pm25'] == 25.0
    assert data['pm10'] == 40.0
    assert data['air_temp'] == 26.5
    assert data['air_humidity'] == 55.0
    assert data['soil_temp'] == 0.0
    assert data['soil_humidity'] == 0.0
    assert data['ec'] == 0.0
    assert data['leaf_wetness'] == 0.0
    assert data['leaf_temp'] == 0.0


def test_decode_cj702_payload_negative_temp():
    payload = struct.pack('>HHHHHHH', *[
        0, 0, 0, 0, 0,
        0xF63C,  # -25.0 C
        0,       # humidity
    ])
    data = decode_cj702_payload(payload)
    assert data['air_temp'] == -25.0


def test_decode_cj702_payload_wrong_length():
    assert decode_cj702_payload(b'\x00' * 10) is None


def test_handle_frame_real_capture(node):
    """Real frame captured from the fixed node over LoRa (2026-08-06)."""
    frame = bytes.fromhex(
        'aa 13 17 00 18 01 db 00 07 00 2f 00 05 00 06 09 63'
        ' 14 52 00 00 01 d5 00 00 01 e7 00 00 b9 b3')
    node.pub_env.publish = MagicMock()
    node._handle_frame(frame)
    node.pub_env.publish.assert_called_once()
    msg = node.pub_env.publish.call_args[0][0]
    assert msg.air_co2 == 475.0
    assert msg.air_temp == pytest.approx(24.03)
    assert msg.air_humidity == pytest.approx(52.02)
    assert msg.soil_humidity == pytest.approx(4.69)
    assert msg.data_source == 'FIXED_LORA'


def test_handle_frame_cj702(node):
    """14-byte CJ702 payload should be parsed and published."""
    frame = _build_cj702_frame([450, 15, 80, 25, 40, 2650, 5500])
    node.pub_env.publish = MagicMock()
    node._handle_frame(frame)
    node.pub_env.publish.assert_called_once()
    msg = node.pub_env.publish.call_args[0][0]
    assert msg.air_co2 == 450.0
    assert msg.soil_temp == 0.0
    assert msg.data_source == 'FIXED_LORA'


def test_handle_frame_error(node):
    """1-byte payload is an error frame: logged, not published."""
    frame = _build_frame(bytes([0x42]))
    node.pub_env.publish = MagicMock()
    node._handle_frame(frame)
    node.pub_env.publish.assert_not_called()


@pytest.fixture
def node(ros_context):
    with patch('sentry_sensors.lora_bridge_node.serial.Serial'):
        n = LoraBridgeNode()
        yield n
        n.destroy_node()


def test_node_creates_publisher(node):
    assert node.pub_env.topic_name == '/sensor/environment_fixed'


def test_node_default_port(node):
    assert node.get_parameter('uart_port').value == '/dev/lora'


def test_handle_frame_crc_mismatch(node):
    frame = bytearray(_build_env_frame([0] * 12))
    frame[-1] ^= 0xFF  # corrupt CRC
    node.pub_env.publish = MagicMock()
    node._handle_frame(bytes(frame))
    node.pub_env.publish.assert_not_called()


def test_handle_frame_unknown_type(node):
    frame = _build_frame(bytes(24), frame_type=0x20)
    node.pub_env.publish = MagicMock()
    node._handle_frame(frame)
    node.pub_env.publish.assert_not_called()


def test_mock_baseline_covers_all_fields():
    """MOCK_BASELINE and MOCK_JITTER should have the same keys."""
    assert set(MOCK_BASELINE.keys()) == set(MOCK_JITTER.keys())
    required = {'air_co2', 'hcho', 'tvoc', 'pm25', 'pm10',
                'air_temp', 'air_humidity', 'soil_temp', 'soil_humidity',
                'ec', 'leaf_wetness', 'leaf_temp'}
    assert set(MOCK_BASELINE.keys()) == required


def test_generate_mock_data(node):
    data = node._generate_mock_data()
    for key, base in MOCK_BASELINE.items():
        jitter = MOCK_JITTER[key]
        assert base - jitter <= data[key] <= base + jitter


def test_mock_tick_publishes(node):
    node.pub_env.publish = MagicMock()
    node._mock_tick()
    node.pub_env.publish.assert_called_once()
    assert node._mock_frame_count == 1
