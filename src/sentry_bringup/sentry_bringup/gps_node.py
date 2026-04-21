import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
import serial


def nmea_to_decimal(coord: str, direction: str) -> float:
    if not coord:
        return 0.0
    try:
        if direction in ('N', 'S'):
            degrees = int(coord[:2])
            minutes = float(coord[2:])
        else:
            degrees = int(coord[:3])
            minutes = float(coord[3:])
        decimal = degrees + minutes / 60.0
        if direction in ('S', 'W'):
            decimal = -decimal
        return decimal
    except (ValueError, IndexError):
        return 0.0


def nmea_checksum(sentence: str) -> bool:
    if '*' not in sentence:
        return False
    body, cs = sentence.split('*')
    calc = 0
    for c in body[1:]:
        calc ^= ord(c)
    try:
        return calc == int(cs, 16)
    except ValueError:
        return False


def parse_nmea_line(line: str):
    line = line.strip()
    if not line.startswith('$'):
        return None
    if not nmea_checksum(line):
        return None
    parts = line.split('*')[0].split(',')
    sentence = parts[0][3:]
    if sentence == 'GGA':
        if len(parts) < 10:
            return None
        try:
            fix = int(parts[6]) if parts[6] else 0
            sats = int(parts[7]) if parts[7] else 0
            hdop = float(parts[8]) if parts[8] else 99.9
            alt = float(parts[9]) if parts[9] else 0.0
            return {
                'type': 'GGA',
                'lat': nmea_to_decimal(parts[2], parts[3]),
                'lon': nmea_to_decimal(parts[4], parts[5]),
                'fix_quality': fix,
                'num_satellites': sats,
                'hdop': hdop,
                'altitude': alt,
            }
        except (ValueError, IndexError):
            return None
    elif sentence == 'RMC':
        if len(parts) < 10:
            return None
        try:
            status = parts[2]
            if status != 'A':
                return None
            speed = float(parts[7]) if parts[7] else 0.0
            track = float(parts[8]) if parts[8] else 0.0
            return {
                'type': 'RMC',
                'lat': nmea_to_decimal(parts[3], parts[4]),
                'lon': nmea_to_decimal(parts[5], parts[6]),
                'speed_knots': speed,
                'track_angle': track,
            }
        except (ValueError, IndexError):
            return None
    return None


class GpsNode(Node):
    def __init__(self):
        super().__init__('gps_node')
        self.declare_parameter('uart_port', '/dev/ttyS6')
        self.declare_parameter('baudrate', 9600)
        port = self.get_parameter('uart_port').value
        baud = self.get_parameter('baudrate').value

        try:
            self.ser = serial.Serial(port, baud, timeout=0.1)
            self.get_logger().info(f'GPS UART open: {port} @ {baud}')
        except serial.SerialException as e:
            self.get_logger().error(f'Failed to open GPS UART: {e}')
            self.ser = None

        self.pub = self.create_publisher(NavSatFix, '/sentry/gps/fix', 10)
        self.timer = self.create_timer(0.1, self.tick)
        self.last_gga = None
        self.last_rmc = None

    def tick(self):
        if self.ser is None or not self.ser.is_open:
            return
        try:
            while self.ser.in_waiting:
                line = self.ser.readline().decode('ascii', errors='ignore')
                data = parse_nmea_line(line)
                if data is None:
                    continue
                if data['type'] == 'GGA':
                    self.last_gga = data
                    self.publish_fix()
                elif data['type'] == 'RMC':
                    self.last_rmc = data
        except serial.SerialException as e:
            self.get_logger().error(f'GPS read error: {e}')

    def publish_fix(self):
        if self.last_gga is None:
            return
        gga = self.last_gga
        msg = NavSatFix()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'gps'
        msg.latitude = gga['lat']
        msg.longitude = gga['lon']
        msg.altitude = gga['altitude']
        if gga['fix_quality'] == 0:
            msg.status.status = -1
        else:
            msg.status.status = 0
        msg.position_covariance_type = 0
        self.pub.publish(msg)

    def destroy_node(self):
        if self.ser and self.ser.is_open:
            self.ser.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = GpsNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
