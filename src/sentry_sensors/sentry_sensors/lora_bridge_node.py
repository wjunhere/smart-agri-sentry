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
PAYLOAD_LEN_CJ702 = 14


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


def decode_cj702_frame(frame: bytes):
    """Decode a validated CJ702 air-sensor frame (14-byte payload) into a dict."""
    if len(frame) != 5 + PAYLOAD_LEN_CJ702 + 1:
        return None
    payload = frame[5:5 + PAYLOAD_LEN_CJ702]
    values = struct.unpack('>HHHHHHH', payload)
    (co2, hcho, tvoc, pm25, pm10, air_temp_raw, air_humidity_raw) = values
    return {
        'air_co2': float(co2),
        'hcho': float(hcho),
        'tvoc': float(tvoc),
        'pm25': float(pm25),
        'pm10': float(pm10),
        'air_temp': _to_int16(air_temp_raw) / 100.0,
        'air_humidity': air_humidity_raw / 100.0,
        'soil_temp': 0.0,
        'soil_humidity': 0.0,
        'ec': 0.0,
        'leaf_wetness': 0.0,
        'leaf_temp': 0.0,
    }


class LoraBridgeNode(Node):
    def __init__(self, **kwargs):
        super().__init__('lora_bridge_node', **kwargs)
        self.declare_parameter('uart_port', '/dev/ttyACM0')
        self.declare_parameter('baudrate', 9600)
        port = self.get_parameter('uart_port').value
        baud = self.get_parameter('baudrate').value

        try:
            self.ser = serial.Serial(port, baud, timeout=0.01)
            self.get_logger().info(f'LoRa UART open: {port} @ {baud}')
        except serial.SerialException as e:
            self.get_logger().error(f'Failed to open LoRa UART: {e}')
            self.ser = None

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.pub_env = self.create_publisher(
            Environment, '/sensor/environment_fixed', qos)

        self.timer_rx = self.create_timer(0.01, self.rx_tick)
        self.rx_buf = bytearray()
        self._reconnect_tick = 0

    def _try_reconnect(self):
        port = self.get_parameter('uart_port').value
        baud = self.get_parameter('baudrate').value
        try:
            self.ser = serial.Serial(port, baud, timeout=0.01)
            self.get_logger().info(f'LoRa UART reopened: {port} @ {baud}')
            return True
        except (serial.SerialException, OSError) as e:
            return False

    def rx_tick(self):
        if self.ser is None:
            self._reconnect_tick += 1
            if self._reconnect_tick >= 300:
                self._reconnect_tick = 0
                self._try_reconnect()
            return
        try:
            if self.ser.in_waiting:
                self.rx_buf.extend(self.ser.read(self.ser.in_waiting))
        except (serial.SerialException, OSError) as e:
            self.get_logger().error(f'LoRa UART read error: {e}')
            self.rx_buf.clear()
            try:
                self.ser.close()
            except Exception:
                pass
            self.ser = None
            self._reconnect_tick = 0
            return

        while True:
            idx = self.rx_buf.find(FRAME_HEADER)
            if idx < 0:
                if len(self.rx_buf) > 512:
                    self.rx_buf.clear()
                break
            if len(self.rx_buf) < idx + 5:
                break
            payload_len = self.rx_buf[idx + 4]
            total = 5 + payload_len + 1
            if len(self.rx_buf) < idx + total:
                break
            frame = bytes(self.rx_buf[idx:idx + total])
            self.rx_buf = self.rx_buf[idx + total:]
            self._handle_frame(frame)

    def _handle_frame(self, frame: bytes):
        msg_type = frame[3]
        payload_len = frame[4]
        if msg_type == MSG_TYPE_ENV:
            if crc8_maxim(frame[:-1]) != frame[-1]:
                self.get_logger().warn('CRC mismatch on environment frame')
                return
            if payload_len == PAYLOAD_LEN_ENV:
                data = decode_environment_frame(frame)
            elif payload_len == PAYLOAD_LEN_CJ702:
                data = decode_cj702_frame(frame)
            else:
                self.get_logger().warn(
                    f'Unexpected payload length: {payload_len}')
                return
            if data is None:
                return
            self._publish_environment(data)
        elif msg_type == MSG_TYPE_ERROR:
            error_code = frame[5] if payload_len >= 1 else None
            self.get_logger().warn(f'LoRa error frame: code={error_code}')
        else:
            self.get_logger().warn(f'Unknown msg_type: {msg_type}')

    def _publish_environment(self, data: dict):
        msg = Environment()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'fixed_env'
        msg.air_temp = data['air_temp']
        msg.air_humidity = data['air_humidity']
        msg.air_co2 = data['air_co2']
        msg.soil_temp = data['soil_temp']
        msg.soil_humidity = data['soil_humidity']
        msg.leaf_wetness = data['leaf_wetness']
        msg.hcho = data['hcho']
        msg.tvoc = data['tvoc']
        msg.pm25 = data['pm25']
        msg.pm10 = data['pm10']
        msg.leaf_temp = data['leaf_temp']
        msg.ec = data['ec']
        msg.data_source = 'FIXED_LORA'
        self.pub_env.publish(msg)

    def destroy_node(self):
        if self.ser and self.ser.is_open:
            self.ser.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = LoraBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
