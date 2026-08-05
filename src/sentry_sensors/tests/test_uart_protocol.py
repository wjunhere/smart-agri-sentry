"""Protocol-level tests for the UART chassis bridge."""

import struct

from sentry_sensors.uart_bridge_node import (
    crc16_ccitt,
    decode_chassis_frame,
    encode_frame,
)


def test_crc16_ccitt_known():
    data = bytes([0x01, 0x00, 0x01])
    crc = crc16_ccitt(data)
    assert isinstance(crc, int)
    assert 0 <= crc <= 0xFFFF


def test_encode_frame_structure():
    frame = encode_frame(0x03, bytes([0x00] * 7))
    assert len(frame) == 13
    assert frame[0:2] == b'\xaa\x55'
    assert frame[2] == 0x03
    assert frame[3] == 7


def test_encode_frame_structure_legacy_sensor():
    # Legacy 0x01 sensor frames still encode correctly (frame format is
    # type-agnostic); decoding them is no longer supported by the bridge.
    frame = encode_frame(0x01, bytes([0x00] * 24))
    assert len(frame) == 30
    assert frame[0:2] == b'\xaa\x55'
    assert frame[2] == 0x01
    assert frame[3] == 24


def test_decode_chassis_frame_valid_legacy():
    # Legacy 7-byte chassis frame: left/right speed (mm/s), battery (0.01V),
    # alarm bits.
    payload = struct.pack('<hhHB', 250, -250, 1260, 3)
    result = decode_chassis_frame(encode_frame(0x03, payload))
    assert result is not None
    assert abs(result['left_speed'] - 0.25) < 1e-6
    assert abs(result['right_speed'] + 0.25) < 1e-6
    assert abs(result['battery_voltage'] - 12.6) < 1e-6
    assert result['alarm_bits'] == 3


def test_decode_chassis_frame_valid_extended():
    # Extended 19-byte chassis frame adds pulse counters and timestamp.
    payload = struct.pack('<hhHBiiI', 250, -250, 1260, 3, 100, 200, 12345)
    result = decode_chassis_frame(encode_frame(0x03, payload))
    assert result is not None
    assert result['left_pulse'] == 100
    assert result['right_pulse'] == 200
    assert result['encoder_timestamp'] == 12345


def test_decode_chassis_frame_bad_crc():
    frame = bytearray(encode_frame(0x03, bytes([0x00] * 7)))
    frame[-1] ^= 0xFF
    assert decode_chassis_frame(bytes(frame)) is None


def test_decode_chassis_frame_wrong_type():
    # Non-chassis frames (e.g. deprecated 0x01 sensor frames) are rejected.
    frame = encode_frame(0x01, bytes([0x00] * 24))
    assert decode_chassis_frame(frame) is None
