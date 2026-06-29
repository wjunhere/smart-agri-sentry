import struct
import sys


def crc16_ccitt(data: bytes) -> int:
    """CRC16-CCITT (polynomial 0x1021, initial value 0xFFFF)."""
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = (crc << 1) ^ 0x1021
            else:
                crc <<= 1
        crc &= 0xFFFF
    return crc


def test_crc16_ccitt_known():
    """Verify against body 0x81 0x04 0x00 0x00 0x00 0x00."""
    body = bytes([0x81, 0x04, 0x00, 0x00, 0x00, 0x00])
    result = crc16_ccitt(body)
    assert result == 0x1696, f"Expected 0x1696, got 0x{result:04X}"


def test_crc_matches_stm32_logic():
    """Verify frame body 0x03 0x13 + 19 zeros produces valid CRC."""
    body = bytes([0x03, 0x13]) + bytes(19)
    result = crc16_ccitt(body)
    # Just ensure it runs without error and produces a 16-bit value
    assert 0 <= result <= 0xFFFF
    assert result == 0x6C64, f"Expected 0x6C64, got 0x{result:04X}"


def pack_chassis_status(left_speed_mm_s: int, right_speed_mm_s: int,
                        battery_x100: int, alarm_bits: int,
                        left_pulse: int, right_pulse: int,
                        timestamp_ms: int) -> bytes:
    """Pack a 25-byte chassis status frame matching the C implementation."""
    header = bytes([0xAA, 0x55])
    type_byte = bytes([0x03])
    len_byte = bytes([0x13])
    data = struct.pack('<hhHBiiI',
                         left_speed_mm_s, right_speed_mm_s,
                         battery_x100, alarm_bits,
                         left_pulse, right_pulse,
                         timestamp_ms)
    crc = crc16_ccitt(type_byte + len_byte + data)
    crc_bytes = bytes([crc >> 8, crc & 0xFF])
    return header + type_byte + len_byte + data + crc_bytes


def test_pack_chassis_status():
    """Verify pack_chassis_status produces a correct 25-byte frame."""
    frame = pack_chassis_status(
        left_speed_mm_s=100,
        right_speed_mm_s=-50,
        battery_x100=1234,
        alarm_bits=0x01,
        left_pulse=100000,
        right_pulse=-200000,
        timestamp_ms=0xDEADBEEF,
    )

    assert len(frame) == 25, f"Expected frame length 25, got {len(frame)}"
    assert frame[0] == 0xAA and frame[1] == 0x55, "Header mismatch"
    assert frame[2] == 0x03, "TYPE mismatch"
    assert frame[3] == 0x13, "LEN mismatch"

    # Verify CRC is computed over bytes 2..23 (TYPE + LEN + DATA, length 21)
    crc_computed = crc16_ccitt(frame[2:23])
    crc_received = (frame[23] << 8) | frame[24]
    assert crc_computed == crc_received, (
        f"CRC mismatch: computed=0x{crc_computed:04X}, received=0x{crc_received:04X}"
    )

    # Verify little-endian data fields
    assert struct.unpack_from('<h', frame, 4)[0] == 100, "left_speed mismatch"
    assert struct.unpack_from('<h', frame, 6)[0] == -50, "right_speed mismatch"
    assert struct.unpack_from('<h', frame, 8)[0] == 1234, "battery mismatch"
    assert frame[10] == 0x01, "alarm_bits mismatch"
    assert struct.unpack_from('<i', frame, 11)[0] == 100000, "left_pulse mismatch"
    assert struct.unpack_from('<i', frame, 15)[0] == -200000, "right_pulse mismatch"
    assert struct.unpack_from('<I', frame, 19)[0] == 0xDEADBEEF, "timestamp mismatch"


if __name__ == "__main__":
    test_crc16_ccitt_known()
    test_crc_matches_stm32_logic()
    test_pack_chassis_status()
    print("All protocol tests OK")
