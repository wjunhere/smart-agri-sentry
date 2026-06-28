import struct

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
import serial

from sentry_interfaces.msg import Environment


FRAME_HEADER = b'\xaa\x55'
CRC8_POLY = 0x31
CRC8_INIT = 0x00
MSG_TYPE_ENV = 0x01
MSG_TYPE_ERROR = 0xFF
PAYLOAD_LEN_ENV = 24


def crc8_maxim(data: bytes) -> int:
    crc = CRC8_INIT
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 0x80:
                crc = ((crc << 1) ^ CRC8_POLY) & 0xFF
            else:
                crc = (crc << 1) & 0xFF
    return crc


def _to_int16(value: int) -> int:
    return value if value < 0x8000 else value - 0x10000


def decode_environment_frame(frame: bytes):
    """Decode a validated environment frame into a dict of floats."""
    if len(frame) != 5 + PAYLOAD_LEN_ENV + 1:
        return None
    payload = frame[5:5 + PAYLOAD_LEN_ENV]
    values = struct.unpack('>HHHHHHHHHHHH', payload)
    (co2, hcho, tvoc, pm25, pm10, air_temp_raw, air_humidity_raw,
     soil_temp_raw, soil_humidity_raw, ec_raw,
     leaf_wetness_raw, leaf_temp_raw) = values
    return {
        'air_co2': float(co2),
        'hcho': float(hcho),
        'tvoc': float(tvoc),
        'pm25': float(pm25),
        'pm10': float(pm10),
        'air_temp': _to_int16(air_temp_raw) / 100.0,
        'air_humidity': air_humidity_raw / 100.0,
        'soil_temp': _to_int16(soil_temp_raw) / 100.0,
        'soil_humidity': soil_humidity_raw / 100.0,
        'ec': float(ec_raw),
        'leaf_wetness': leaf_wetness_raw / 100.0,
        'leaf_temp': _to_int16(leaf_temp_raw) / 100.0,
    }
