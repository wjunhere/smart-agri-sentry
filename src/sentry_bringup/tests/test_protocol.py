import pytest
from sentry_bringup.uart_bridge_node import crc16_ccitt, encode_frame, decode_sensor_frame


def test_crc16_ccitt_known():
    data = bytes([0x01, 0x00, 0x01])
    crc = crc16_ccitt(data)
    assert isinstance(crc, int)
    assert 0 <= crc <= 0xFFFF


def test_encode_frame_structure():
    frame = encode_frame(0x01, bytes([0x00] * 24))
    assert len(frame) == 30
    assert frame[0:2] == b'\xaa\x55'
    assert frame[2] == 0x01
    assert frame[3] == 24


def test_decode_sensor_frame_valid():
    import struct
    payload = struct.pack('<IhHHhHHHHHH',
                          1000, 250, 600, 400,
                          200, 550, 100, 50, 30, 40, 65)
    frame = encode_frame(0x01, payload)
    result = decode_sensor_frame(frame)
    assert result is not None
    assert result['timestamp_ms'] == 1000
    assert abs(result['air_temp'] - 25.0) < 0.01
    assert abs(result['soil_ph'] - 6.5) < 0.01


def test_decode_sensor_frame_bad_crc():
    frame = bytearray(encode_frame(0x01, bytes([0x00] * 24)))
    frame[-1] ^= 0xFF
    result = decode_sensor_frame(bytes(frame))
    assert result is None
