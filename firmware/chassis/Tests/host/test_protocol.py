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


if __name__ == "__main__":
    test_crc16_ccitt_known()
    test_crc_matches_stm32_logic()
    print("CRC tests OK")
