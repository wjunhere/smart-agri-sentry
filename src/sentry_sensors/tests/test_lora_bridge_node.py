import pytest
import rclpy
from unittest.mock import patch, MagicMock

from sentry_sensors.lora_bridge_node import (
    crc8_maxim,
    decode_environment_frame,
    LoraBridgeNode,
)


def test_crc8_maxim_empty():
    assert crc8_maxim(b'') == 0x00


def test_crc8_maxim_header_only():
    # Known vector computed offline: CRC8/MAXIM of [0xAA, 0x55]
    assert crc8_maxim(b'\xaa\x55') == 0x8C
