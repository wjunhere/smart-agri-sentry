import struct

import pytest
import rclpy
from unittest.mock import patch, MagicMock

from sentry_sensors.lora_bridge_node import (
    crc8_maxim,
    decode_environment_frame,
    decode_cj702_frame,
    LoraBridgeNode,
    MOCK_BASELINE,
    MOCK_JITTER,
)


@pytest.fixture(scope='module')
def ros_context():
    rclpy.init()
    yield
    rclpy.shutdown()


def _build_frame(values):
    payload = struct.pack('>HHHHHHHHHHHH', *values)
    header = bytes([0xAA, 0x55, 0x01, 0x01, len(payload)])
    frame = header + payload
    return frame + bytes([crc8_maxim(frame)])


def test_crc8_maxim_empty():
    assert crc8_maxim(b'') == 0x00


def test_crc8_maxim_header_only():
    # Computed with current CRC-8/MAXIM implementation (no reflection)
    assert crc8_maxim(b'\xaa\x55') == 0x9A


def test_decode_environment_frame_valid():
    frame = _build_frame([
        303, 20, 120, 35, 50,          # co2, hcho, tvoc, pm25, pm10
        2700, 6000,                    # air_temp, air_humidity
        2500, 5000,                    # soil_temp, soil_humidity
        150,                           # ec
        7000,                          # leaf_wetness
        2800,                          # leaf_temp
    ])
    data = decode_environment_frame(frame)
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


def test_decode_environment_frame_negative_temperatures():
    frame = _build_frame([
        0, 0, 0, 0, 0,
        0xF63C, 0,      # -25.00 C
        0xF63C, 0,      # -25.00 C
        0,
        0,
        0xF63C,         # -25.00 C
    ])
    data = decode_environment_frame(frame)
    assert data['air_temp'] == -25.0
    assert data['soil_temp'] == -25.0
    assert data['leaf_temp'] == -25.0


def test_decode_environment_frame_wrong_length():
    assert decode_environment_frame(b'\xaa\x55\x01\x01\x00\x00') is None


def _build_cj702_frame(values):
    """Build a 14-byte CJ702 frame with header, payload, CRC."""
    payload = struct.pack('>HHHHHHH', *values)
    header = bytes([0xAA, 0x55, 0x01, 0x01, len(payload)])
    frame = header + payload
    return frame + bytes([crc8_maxim(frame)])


def test_decode_cj702_frame_valid():
    frame = _build_cj702_frame([
        450,   # co2 ppm
        15,    # hcho raw
        80,    # tvoc ppb
        25,    # pm25 ug/m3
        40,    # pm10 ug/m3
        2650,  # air_temp 26.5 C
        5500,  # air_humidity 55.0% RH
    ])
    data = decode_cj702_frame(frame)
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


def test_decode_cj702_frame_negative_temp():
    frame = _build_cj702_frame([
        0, 0, 0, 0, 0,
        0xF63C,  # -25.0 C
        0,       # humidity
    ])
    data = decode_cj702_frame(frame)
    assert data['air_temp'] == -25.0


def test_decode_cj702_frame_wrong_length():
    assert decode_cj702_frame(b'\xaa\x55\x01\x01\x00\x00') is None


def test_handle_frame_cj702(node):
    """14-byte CJ702 frame should be parsed and published."""
    frame = _build_cj702_frame([450, 15, 80, 25, 40, 2650, 5500])
    node.pub_env.publish = MagicMock()
    node._handle_frame(frame)
    node.pub_env.publish.assert_called_once()
    msg = node.pub_env.publish.call_args[0][0]
    assert msg.air_co2 == 450.0
    assert msg.soil_temp == 0.0
    assert msg.data_source == 'FIXED_LORA'


@pytest.fixture
def node(ros_context):
    with patch('sentry_sensors.lora_bridge_node.serial.Serial'):
        n = LoraBridgeNode()
        yield n
        n.destroy_node()


def test_node_creates_publisher(node):
    assert node.pub_env.topic_name == '/sensor/environment_fixed'


def test_handle_frame_crc_mismatch(node):
    bad_frame = bytes([0xAA, 0x55, 0x01, 0x01, 24]) + bytes(24) + bytes([0xFF])
    node.pub_env.publish = MagicMock()
    node._handle_frame(bad_frame)
    node.pub_env.publish.assert_not_called()


def test_handle_frame_unknown_msg_type(node):
    payload = bytes(24)
    header = bytes([0xAA, 0x55, 0x01, 0xAB, len(payload)])
    frame = header + payload
    frame += bytes([crc8_maxim(frame)])
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
