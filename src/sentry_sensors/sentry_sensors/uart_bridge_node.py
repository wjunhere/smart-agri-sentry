import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
import serial
import struct

from geometry_msgs.msg import Twist
from sentry_interfaces.msg import (
    Environment, SoilNutrition, ChassisStatus, ServoCmd, ChassisConfig)
from std_srvs.srv import Trigger


# ---- Protocol Constants ----
FRAME_HEADER = b'\xaa\x55'
TYPE_SENSOR = 0x01
TYPE_CHASSIS = 0x03
TYPE_MOTION_CMD = 0x81
TYPE_SERVO_CMD = 0x82
TYPE_MODE_CMD = 0x83
TYPE_CONFIG_CMD = 0x84
TYPE_RESET_ENCODER = 0x85

MODE_STANDBY = 0x00
MODE_REMOTE = 0x01
MODE_AUTO = 0x02


# ---- CRC16-CCITT (0x1021, init 0xFFFF) ----
def crc16_ccitt(data: bytes) -> int:
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


def encode_frame(frame_type: int, payload: bytes) -> bytes:
    length = len(payload)
    body = bytes([frame_type, length]) + payload
    crc = crc16_ccitt(body)
    return FRAME_HEADER + body + struct.pack('>H', crc)


def decode_sensor_frame(frame: bytes):
    if len(frame) < 6:
        return None
    if frame[0:2] != FRAME_HEADER:
        return None
    frame_type = frame[2]
    length = frame[3]
    if len(frame) != 4 + length + 2:
        return None
    body = frame[2:4 + length]
    payload = frame[4:4 + length]
    rx_crc = struct.unpack('>H', frame[4 + length:4 + length + 2])[0]
    if crc16_ccitt(body) != rx_crc:
        return None
    if frame_type != TYPE_SENSOR:
        return None
    if length != 24:
        return None
    (ts, at, ah, ac, st, sh, sec, sn, sp, sk, sph) = struct.unpack(
        '<IhHHhHHHHHH', payload)
    return {
        'timestamp_ms': ts,
        'air_temp': at / 10.0,
        'air_humi': ah / 10.0,
        'air_co2': ac,
        'soil_temp': st / 10.0,
        'soil_humi': sh / 10.0,
        'soil_ec': sec,
        'soil_n': sn,
        'soil_p': sp,
        'soil_k': sk,
        'soil_ph': sph / 10.0,
    }


def decode_chassis_frame(frame: bytes):
    if len(frame) < 6:
        return None
    if frame[0:2] != FRAME_HEADER:
        return None
    frame_type = frame[2]
    length = frame[3]
    if len(frame) != 4 + length + 2:
        return None
    body = frame[2:4 + length]
    payload = frame[4:4 + length]
    rx_crc = struct.unpack('>H', frame[4 + length:4 + length + 2])[0]
    if crc16_ccitt(body) != rx_crc:
        return None
    if frame_type != TYPE_CHASSIS:
        return None
    # 兼容旧版 (7 bytes) 和新版 (19 bytes)
    if length == 7:
        (ls, rs, bv, alarm) = struct.unpack('<hhHB', payload)
        return {
            'left_speed': ls / 1000.0,
            'right_speed': rs / 1000.0,
            'battery_voltage': bv / 100.0,
            'alarm_bits': alarm,
            'left_pulse': 0,
            'right_pulse': 0,
            'encoder_timestamp': 0,
        }
    elif length == 19:
        (ls, rs, bv, alarm, lp, rp, ts) = struct.unpack('<hhHBiiI', payload)
        return {
            'left_speed': ls / 1000.0,
            'right_speed': rs / 1000.0,
            'battery_voltage': bv / 100.0,
            'alarm_bits': alarm,
            'left_pulse': lp,
            'right_pulse': rp,
            'encoder_timestamp': ts,
        }
    else:
        return None


class UartBridgeNode(Node):
    def __init__(self, **kwargs):
        super().__init__('uart_bridge_node', **kwargs)
        self.declare_parameter('uart_port', '/dev/ttyS1')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('forward_servo_cmd', False)
        self.declare_parameter('wheel_base', 0.23)
        self.declare_parameter('left_speed_scale', 1.0)
        self.declare_parameter('right_speed_scale', 1.0)
        self.declare_parameter('swap_wheel_commands', False)
        # The two encoder channels on the current chassis wiring are crossed:
        # the firmware's "left" pulse counter is attached to the physical
        # right wheel and vice versa. Swap them here so downstream odometry
        # sees physical-correct sides (d_theta = (right-left)/wheel_base
        # then matches the real turn direction).
        self.declare_parameter('swap_encoder_channels', False)
        self.declare_parameter('chassis_timeout_sec', 1.0)
        # If no valid chassis frame arrives for this long, close and reopen
        # the serial port: a wedged kernel-side UART path (driver/FIFO
        # state surviving a killed process) recovers this way without a
        # stack restart. Also a diagnostic: if reopening restores comm,
        # the fault was on the RDK side, not the STM32.
        self.declare_parameter('chassis_reopen_after_sec', 5.0)
        self.declare_parameter('motion_mode', MODE_AUTO)
        self.declare_parameter('min_effective_linear_speed', 0.08)  # m/s, boost floor
        self.uart_port = self.get_parameter('uart_port').value
        self.baud = self.get_parameter('baudrate').value
        forward_servo = self.get_parameter('forward_servo_cmd').value
        self.wheel_base = self.get_parameter('wheel_base').value
        self.left_speed_scale = self.get_parameter('left_speed_scale').value
        self.right_speed_scale = self.get_parameter('right_speed_scale').value
        self.swap_wheel_commands = self.get_parameter(
            'swap_wheel_commands').value
        self.swap_encoder_channels = self.get_parameter(
            'swap_encoder_channels').value
        self.chassis_timeout_sec = self.get_parameter(
            'chassis_timeout_sec').value
        self.chassis_reopen_after = self.get_parameter(
            'chassis_reopen_after_sec').value
        self._last_reopen_monotonic = 0.0
        self.motion_mode = int(self.get_parameter('motion_mode').value)
        self.min_effective_v = self.get_parameter(
            'min_effective_linear_speed').value
        self._last_sent_mode = None

        self._open_serial()

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.pub_env = self.create_publisher(
            Environment, '/sensor/environment_mobile', qos)
        self.pub_soil = self.create_publisher(
            SoilNutrition, '/sensor/soil_nutrition', qos)
        self.pub_chassis = self.create_publisher(
            ChassisStatus, '/sentry/chassis/status', qos)

        self.sub_cmd_vel = self.create_subscription(
            Twist, '/sentry/cmd_vel', self.on_cmd_vel, 10)
        self.sub_config = self.create_subscription(
            ChassisConfig, '/sentry/chassis/config', self.on_chassis_config, 10)
        if forward_servo:
            self.sub_servo = self.create_subscription(
                ServoCmd, '/sentry/servo_cmd', self.on_servo, 10)
            self.get_logger().info('ServoCmd forwarding to STM32 enabled')
        else:
            self.get_logger().info(
                'ServoCmd forwarding disabled; assuming direct RDK X5 PWM')

        # Encoder reset service
        self.srv_reset_enc = self.create_service(
            Trigger, '/sentry/reset_encoder', self.reset_encoder_cb)

        self.timer_rx = self.create_timer(0.01, self.rx_tick)
        self.rx_buf = bytearray()

        self.last_chassis_time = self.get_clock().now()
        self.chassis_timed_out = False
        self.timer_chassis_timeout = self.create_timer(
            1.0, self.check_chassis_timeout)

    def rx_tick(self):
        if self.ser is None or not self.ser.is_open:
            return
        try:
            try:
                waiting = self.ser.in_waiting
            except OSError:
                # Some UART drivers (e.g. RDK X5 dw-apb-uart) do not
                # support the TIOCINQ ioctl used by in_waiting.
                waiting = None
            if waiting is None:
                chunk = self.ser.read(256)
            elif waiting:
                chunk = self.ser.read(waiting)
            else:
                chunk = b''
            if chunk:
                self.rx_buf.extend(chunk)
        except serial.SerialException as e:
            self.get_logger().error(f'UART read error: {e}')
            return

        while True:
            idx = self.rx_buf.find(FRAME_HEADER)
            if idx < 0:
                if len(self.rx_buf) > 256:
                    self.rx_buf.clear()
                break
            if len(self.rx_buf) < idx + 4:
                break
            length = self.rx_buf[idx + 3]
            total = 4 + length + 2
            if len(self.rx_buf) < idx + total:
                break
            frame = bytes(self.rx_buf[idx:idx + total])
            self.rx_buf = self.rx_buf[idx + total:]
            self.handle_frame(frame)

    def handle_frame(self, frame: bytes):
        frame_type = frame[2]
        if frame_type == TYPE_SENSOR:
            data = decode_sensor_frame(frame)
            if data:
                now = self.get_clock().now().to_msg()
                env = Environment()
                env.header.stamp = now
                env.air_temp = data['air_temp']
                env.air_humidity = data['air_humi']
                env.air_co2 = float(data['air_co2'])
                env.soil_temp = data['soil_temp']
                env.soil_humidity = data['soil_humi']
                env.leaf_wetness = 0.0  # not available from mobile sensor
                env.data_source = 'MOBILE'
                self.pub_env.publish(env)

                soil = SoilNutrition()
                soil.header.stamp = now
                soil.nitrogen = float(data['soil_n'])
                soil.phosphorus = float(data['soil_p'])
                soil.potassium = float(data['soil_k'])
                soil.ph = data['soil_ph']
                soil.ec = float(data['soil_ec'])
                self.pub_soil.publish(soil)
            else:
                self.get_logger().debug(
                    f'Invalid sensor frame discarded: {frame.hex()}')
        elif frame_type == TYPE_CHASSIS:
            data = decode_chassis_frame(frame)
            if data:
                msg = ChassisStatus()
                if self.swap_encoder_channels:
                    msg.left_speed = data['right_speed']
                    msg.right_speed = data['left_speed']
                    msg.left_pulse = data['right_pulse']
                    msg.right_pulse = data['left_pulse']
                else:
                    msg.left_speed = data['left_speed']
                    msg.right_speed = data['right_speed']
                    msg.left_pulse = data['left_pulse']
                    msg.right_pulse = data['right_pulse']
                msg.battery_voltage = data['battery_voltage']
                msg.alarm_bits = data['alarm_bits']
                msg.encoder_timestamp = data['encoder_timestamp']
                msg.comm_timeout = False
                self.pub_chassis.publish(msg)
                self.last_chassis_time = self.get_clock().now()
                if self.chassis_timed_out:
                    self.get_logger().info('Chassis status frame recovered')
                    self.chassis_timed_out = False
            else:
                self.get_logger().debug(
                    f'Invalid chassis frame discarded: {frame.hex()}')

    def _open_serial(self):
        try:
            self.ser = serial.Serial(self.uart_port, self.baud, timeout=0)
            self.get_logger().info(
                f'UART open: {self.uart_port} @ {self.baud}')
        except serial.SerialException as e:
            self.get_logger().error(f'Failed to open UART: {e}')
            self.ser = None

    def _reopen_serial(self):
        try:
            if self.ser is not None:
                self.ser.close()
        except serial.SerialException:
            pass
        self._open_serial()

    def check_chassis_timeout(self):
        elapsed = (
            self.get_clock().now() - self.last_chassis_time).nanoseconds / 1e9
        if elapsed > self.chassis_timeout_sec:
            if not self.chassis_timed_out:
                self.get_logger().warning(
                    f'No chassis status frame for {elapsed:.1f}s')
                self.chassis_timed_out = True
            if (elapsed > self.chassis_reopen_after
                    and time.monotonic() - self._last_reopen_monotonic
                        >= self.chassis_reopen_after):
                self._last_reopen_monotonic = time.monotonic()
                self.get_logger().warning(
                    f'Reopening UART after {elapsed:.1f}s without frames '
                    '(if this restores comm, the wedge was RDK-side)')
                self._reopen_serial()
                # Give the fresh port a full window before the next reopen.
                self.last_chassis_time = self.get_clock().now()
            msg = ChassisStatus()
            msg.left_speed = float('nan')
            msg.right_speed = float('nan')
            msg.battery_voltage = float('nan')
            msg.alarm_bits = 0
            msg.left_pulse = 0
            msg.right_pulse = 0
            msg.encoder_timestamp = 0
            msg.comm_timeout = True
            self.pub_chassis.publish(msg)

    def on_cmd_vel(self, msg: Twist):
        if self.ser is None or not self.ser.is_open:
            return

        v = msg.linear.x
        w = msg.angular.z
        if not (math.isfinite(v) and math.isfinite(w)):
            self.get_logger().warning(
                f'Ignoring non-finite Twist: linear.x={v}, angular.z={w}')
            return

        # Minimum effective speed boost: if |v| is non-zero but below
        # min_effective_v, scale it up to overcome static friction dead-zone.
        abs_v = abs(v)
        if 0.0 < abs_v < self.min_effective_v:
            scale = self.min_effective_v / abs_v
            v = v * scale
            w = w * scale  # scale angular proportionally to keep curvature
            self.get_logger().debug(
                f'Boosting cmd_vel: {msg.linear.x:.3f}→{v:.3f} m/s '
                f'(min={self.min_effective_v:.3f})')

        self._send_mode_if_needed(self.motion_mode)

        left_m_s = v - w * self.wheel_base / 2.0
        right_m_s = v + w * self.wheel_base / 2.0
        left_m_s *= self.left_speed_scale
        right_m_s *= self.right_speed_scale

        left_mm_s = max(-32768, min(32767, int(left_m_s * 1000)))
        right_mm_s = max(-32768, min(32767, int(right_m_s * 1000)))
        if self.swap_wheel_commands:
            left_mm_s, right_mm_s = right_mm_s, left_mm_s

        payload = struct.pack('<hh', left_mm_s, right_mm_s)
        frame = encode_frame(TYPE_MOTION_CMD, payload)
        try:
            self.ser.write(frame)
        except serial.SerialException as e:
            self.get_logger().error(f'UART write error: {e}')

    def _send_mode_if_needed(self, mode: int):
        if self._last_sent_mode == mode:
            return
        payload = bytes([mode & 0xFF])
        frame = encode_frame(TYPE_MODE_CMD, payload)
        try:
            self.ser.write(frame)
            self._last_sent_mode = mode
            self.get_logger().info(f'Sent chassis mode: {mode}')
        except serial.SerialException as e:
            self.get_logger().error(f'UART write error: {e}')

    def on_chassis_config(self, msg: ChassisConfig):
        if self.ser is None or not self.ser.is_open:
            return
        if not math.isfinite(msg.value):
            self.get_logger().warning(
                f'Ignoring non-finite config value for param_id={msg.param_id}')
            return
        payload = bytes([msg.param_id]) + struct.pack('<f', msg.value)
        frame = encode_frame(TYPE_CONFIG_CMD, payload)
        try:
            self.ser.write(frame)
            self.get_logger().debug(
                f'Sent config param_id={msg.param_id} value={msg.value}')
        except serial.SerialException as e:
            self.get_logger().error(f'UART write error: {e}')

    def on_servo(self, msg: ServoCmd):
        if self.ser is None or not self.ser.is_open:
            return
        payload = struct.pack('<BB', msg.pitch, msg.yaw)
        frame = encode_frame(TYPE_SERVO_CMD, payload)
        try:
            self.ser.write(frame)
        except serial.SerialException as e:
            self.get_logger().error(f'UART write error: {e}')

    def reset_encoder_cb(self, request, response):
        if self.ser is None or not self.ser.is_open:
            response.success = False
            response.message = 'UART not open'
            return response
        payload = bytes([0x00])  # reserved
        frame = encode_frame(TYPE_RESET_ENCODER, payload)
        try:
            self.ser.write(frame)
            self.get_logger().info('Sent encoder reset command')
            response.success = True
            response.message = 'Encoder reset command sent'
        except serial.SerialException as e:
            response.success = False
            response.message = f'UART write error: {e}'
        return response

    def destroy_node(self):
        if self.ser and self.ser.is_open:
            self.ser.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = UartBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
