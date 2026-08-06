import random
import struct

import rclpy
from rclpy.node import Node
import serial

from sentry_interfaces.msg import Environment


# Frame format (matches stm32_cj702_lora_opt_v2 lora_frame.c):
#   [0]     SYNC  = 0xAA
#   [1]     TYPE  = frame_type(high nibble) | status(low nibble)
#   [2]     SEQ   = incrementing sequence number
#   [3]     FLAG  = 0x00
#   [4]     LEN   = payload length
#   [5..N]  PAYLOAD
#   [N+1..] CRC16-CCITT (poly 0x1021, init 0xFFFF) over bytes [1..N], big-endian
SYNC_BYTE = 0xAA
FRAME_TYPE_MASK = 0xF0
FRAME_TYPE_DATA = 0x10
STATUS_NODE_OK = 0x01
STATUS_SRC_ACTIVE = 0x02
FRAME_OVERHEAD = 7  # sync(1) + type(1) + seq(1) + flag(1) + len(1) + crc16(2)
PAYLOAD_LEN_ENV = 24
PAYLOAD_LEN_CJ702 = 14
PAYLOAD_LEN_ERROR = 1


def crc16_ccitt(data: bytes) -> int:
    """CRC-16/CCITT-FALSE: poly 0x1021, init 0xFFFF."""
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


def _to_int16(value: int) -> int:
    return value if value < 0x8000 else value - 0x10000


def decode_environment_payload(payload: bytes):
    """Decode a 24-byte environment payload into a dict of floats."""
    if len(payload) != PAYLOAD_LEN_ENV:
        return None
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


def decode_cj702_payload(payload: bytes):
    """Decode a 14-byte CJ702 air-sensor payload into a dict."""
    if len(payload) != PAYLOAD_LEN_CJ702:
        return None
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


MOCK_BASELINE = {
    'air_co2': 420.0,
    'hcho': 0.03,
    'tvoc': 120.0,
    'pm25': 25.0,
    'pm10': 45.0,
    'air_temp': 28.5,
    'air_humidity': 65.0,
    'soil_temp': 24.0,
    'soil_humidity': 55.0,
    'ec': 200.0,
    'leaf_wetness': 30.0,
    'leaf_temp': 27.0,
}

MOCK_JITTER = {
    'air_co2': 15.0,
    'hcho': 0.01,
    'tvoc': 30.0,
    'pm25': 5.0,
    'pm10': 8.0,
    'air_temp': 0.8,
    'air_humidity': 3.0,
    'soil_temp': 0.3,
    'soil_humidity': 2.0,
    'ec': 10.0,
    'leaf_wetness': 5.0,
    'leaf_temp': 0.5,
}


class LoraBridgeNode(Node):
    def __init__(self, **kwargs):
        super().__init__('lora_bridge_node', **kwargs)
        self.declare_parameter('uart_port', '/dev/lora')
        self.declare_parameter('baudrate', 9600)
        self.declare_parameter('use_mock', False)
        self._use_mock = self.get_parameter('use_mock').value

        if self._use_mock:
            self.get_logger().info('LoRa bridge in MOCK mode — generating synthetic sensor data')
            self.ser = None
        else:
            port = self.get_parameter('uart_port').value
            baud = self.get_parameter('baudrate').value
            try:
                self.ser = serial.Serial(port, baud, timeout=0.01)
                self.get_logger().info(f'LoRa UART open: {port} @ {baud}')
            except serial.SerialException as e:
                self.get_logger().error(f'Failed to open LoRa UART: {e}')
                self.ser = None

        # Reliable QoS: only 1 msg/min, and it keeps default-QoS subscribers
        # (miniprogram bridge, advisory, data logger, rosbridge web frontend)
        # compatible — BEST_EFFORT here would silently starve them.
        self.pub_env = self.create_publisher(
            Environment, '/sensor/environment_fixed', 10)

        if self._use_mock:
            self.timer_mock = self.create_timer(5.0, self._mock_tick)
        else:
            self.timer_rx = self.create_timer(0.01, self.rx_tick)
        self.rx_buf = bytearray()
        self._reconnect_tick = 0
        self._mock_frame_count = 0

    def _try_reconnect(self):
        port = self.get_parameter('uart_port').value
        baud = self.get_parameter('baudrate').value
        try:
            self.ser = serial.Serial(port, baud, timeout=0.01)
            self.get_logger().info(f'LoRa UART reopened: {port} @ {baud}')
            return True
        except (serial.SerialException, OSError):
            return False

    def _generate_mock_data(self):
        data = {}
        for key, base in MOCK_BASELINE.items():
            jitter = MOCK_JITTER[key] * (random.random() * 2 - 1)
            data[key] = round(base + jitter, 2)
        return data

    def _mock_tick(self):
        self._mock_frame_count += 1
        data = self._generate_mock_data()
        self._publish_environment(data)
        self.get_logger().info(
            f'[LORA RX #{self._mock_frame_count}] '
            f'CO2={data["air_co2"]}ppm HCHO={data["hcho"]}mg/m³ TVOC={data["tvoc"]}ppb '
            f'PM2.5={data["pm25"]}ug/m³ PM10={data["pm10"]}ug/m³ '
            f'Air={data["air_temp"]}°C {data["air_humidity"]}%RH '
            f'Soil={data["soil_temp"]}°C {data["soil_humidity"]}%RH EC={data["ec"]}uS/cm '
            f'Leaf={data["leaf_temp"]}°C wet={data["leaf_wetness"]}%')

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
            idx = self.rx_buf.find(bytes([SYNC_BYTE]))
            if idx < 0:
                if len(self.rx_buf) > 512:
                    self.rx_buf.clear()
                break
            if len(self.rx_buf) < idx + 5:
                break
            payload_len = self.rx_buf[idx + 4]
            total = 5 + payload_len + 2
            if len(self.rx_buf) < idx + total:
                break
            frame = bytes(self.rx_buf[idx:idx + total])
            self.rx_buf = self.rx_buf[idx + total:]
            self._handle_frame(frame)

    def _handle_frame(self, frame: bytes):
        if crc16_ccitt(frame[1:-2]) != struct.unpack('>H', frame[-2:])[0]:
            self.get_logger().warn('CRC16 mismatch on LoRa frame')
            return
        frame_type = frame[1] & FRAME_TYPE_MASK
        status = frame[1] & 0x0F
        seq = frame[2]
        payload = frame[5:-2]
        if frame_type != FRAME_TYPE_DATA:
            self.get_logger().warn(f'Unknown frame type: 0x{frame_type:02x}')
            return
        if len(payload) == PAYLOAD_LEN_ERROR:
            self.get_logger().warn(
                f'LoRa error frame: seq={seq} code={payload[0]}')
            return
        if len(payload) == PAYLOAD_LEN_ENV:
            data = decode_environment_payload(payload)
        elif len(payload) == PAYLOAD_LEN_CJ702:
            data = decode_cj702_payload(payload)
        else:
            self.get_logger().warn(
                f'Unexpected payload length: {len(payload)}')
            return
        if data is None:
            return
        if not (status & STATUS_NODE_OK):
            self.get_logger().warn(
                f'LoRa node status flags: 0x{status:02x} (seq={seq})')
        self._publish_environment(data)

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
