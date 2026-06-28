import struct

import pytest
import rclpy
from unittest.mock import patch, MagicMock

from sentry_sensors.lora_bridge_node import (
    crc8_maxim,
    decode_environment_frame,
    LoraBridgeNode,
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
