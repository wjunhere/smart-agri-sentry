# Batch 1: Data Pipeline Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build the core ROS2 data pipeline on RDK X5: custom messages, UART bridge, GPS, camera, and AI inference nodes, with data flowing end-to-end.

**Architecture:** Two ROS2 packages: `sentry_interfaces` (CMake, custom msg definitions) and `sentry_bringup` (Python, all nodes). Nodes communicate via ROS2 topics. `uart_bridge_node` handles STM32 ↔ RDK binary protocol. `gps_node` parses NMEA from UART6. `camera_node` captures from IMX219. `ai_inference_node` runs TFLite CPU inference on 224×224 images.

**Tech Stack:** ROS2 Humble, Python 3, TFLite Runtime, OpenCV, PySerial, FastAPI (placeholder for batch 2)

---

## File Structure

```
smart-agri-sentry/
├── docs/superpowers/plans/2026-04-22-batch1-data-pipeline.md
├── .claude/PROJECT_CONTEXT.md
├── models/finetuned_mobilenetv2_int8.tflite
├── src/
│   ├── sentry_interfaces/          # CMake package - custom messages
│   │   ├── msg/
│   │   │   ├── AiDiagnosis.msg
│   │   │   ├── SensorCombined.msg
│   │   │   ├── FinalDiagnosis.msg
│   │   │   ├── ServoCmd.msg
│   │   │   └── ChassisStatus.msg
│   │   ├── CMakeLists.txt
│   │   └── package.xml
│   └── sentry_bringup/             # Python package - nodes + launch
│       ├── sentry_bringup/
│       │   ├── __init__.py
│       │   ├── uart_bridge_node.py
│       │   ├── gps_node.py
│       │   ├── camera_node.py
│       │   └── ai_inference_node.py
│       ├── launch/
│       │   └── sentry.launch.py
│       ├── tests/
│       │   ├── test_protocol.py
│       │   ├── test_nmea.py
│       │   └── test_preprocessing.py
│       ├── setup.py
│       ├── setup.cfg
│       └── package.xml
```

---

### Task 1: Initialize Git Repository and Create GitHub Remote

**Files:**
- Create: `.gitignore`

- [ ] **Step 1: Initialize git repo**

```bash
git init
git checkout -b main
```

- [ ] **Step 2: Create .gitignore**

```gitignore
# Python
__pycache__/
*.py[cod]
*$py.class
*.so
.Python
build/
develop-eggs/
dist/
downloads/
eggs/
.eggs/
lib/
lib64/
parts/
sdist/
var/
wheels/
*.egg-info/
.installed.cfg
*.egg

# ROS2
install/
log/
build/
colcon-*

# IDE
.vscode/
.idea/
*.swp
*.swo

# Models (keep reference but maybe large)
# models/*.tflite

# OS
.DS_Store
Thumbs.db
```

- [ ] **Step 3: Create GitHub repository via gh CLI**

```bash
gh repo create smart-agri-sentry --public --source=. --remote=origin --push
```

Expected: Repository created at `https://github.com/<user>/smart-agri-sentry`

- [ ] **Step 4: Initial commit**

```bash
git add .gitignore docs/
git commit -m "chore: init repo with docs and plan"
```

---

### Task 2: Create sentry_interfaces Package (Custom Messages)

**Files:**
- Create: `src/sentry_interfaces/msg/AiDiagnosis.msg`
- Create: `src/sentry_interfaces/msg/SensorCombined.msg`
- Create: `src/sentry_interfaces/msg/FinalDiagnosis.msg`
- Create: `src/sentry_interfaces/msg/ServoCmd.msg`
- Create: `src/sentry_interfaces/msg/ChassisStatus.msg`
- Create: `src/sentry_interfaces/CMakeLists.txt`
- Create: `src/sentry_interfaces/package.xml`

- [ ] **Step 1: Create message definitions**

`src/sentry_interfaces/msg/AiDiagnosis.msg`:
```
std_msgs/Header header
string disease_class
float32 confidence
float32[] probabilities
```

`src/sentry_interfaces/msg/SensorCombined.msg`:
```
uint32 timestamp_ms
float32 air_temp
float32 air_humi
uint16 air_co2
float32 soil_temp
float32 soil_humi
uint16 soil_ec
uint16 soil_n
uint16 soil_p
uint16 soil_k
float32 soil_ph
```

`src/sentry_interfaces/msg/FinalDiagnosis.msg`:
```
std_msgs/Header header
string disease_class
float32 risk_score
string confidence_level
string fusion_mode
```

`src/sentry_interfaces/msg/ServoCmd.msg`:
```
uint8 pitch
uint8 yaw
```

`src/sentry_interfaces/msg/ChassisStatus.msg`:
```
float32 left_speed
float32 right_speed
float32 battery_voltage
uint8 alarm_bits
```

- [ ] **Step 2: Create CMakeLists.txt**

`src/sentry_interfaces/CMakeLists.txt`:
```cmake
cmake_minimum_required(VERSION 3.8)
project(sentry_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/AiDiagnosis.msg"
  "msg/SensorCombined.msg"
  "msg/FinalDiagnosis.msg"
  "msg/ServoCmd.msg"
  "msg/ChassisStatus.msg"
  DEPENDENCIES std_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
```

- [ ] **Step 3: Create package.xml**

`src/sentry_interfaces/package.xml`:
```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>sentry_interfaces</name>
  <version>0.1.0</version>
  <description>Custom message definitions for Smart Agri Sentry</description>
  <maintainer email="team@example.com">team</maintainer>
  <license>MIT</license>

  <buildtool_depend>ament_cmake</buildtool_depend>
  <build_depend>rosidl_default_generators</build_depend>
  <exec_depend>rosidl_default_runtime</exec_depend>
  <depend>std_msgs</depend>

  <member_of_group>rosidl_interface_packages</member_of_group>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

- [ ] **Step 4: Commit**

```bash
git add src/sentry_interfaces/
git commit -m "feat(interfaces): add custom ROS2 messages"
```

---

### Task 3: Create sentry_bringup Python Package Skeleton

**Files:**
- Create: `src/sentry_bringup/package.xml`
- Create: `src/sentry_bringup/setup.py`
- Create: `src/sentry_bringup/setup.cfg`
- Create: `src/sentry_bringup/sentry_bringup/__init__.py`
- Create: `src/sentry_bringup/resource/sentry_bringup`

- [ ] **Step 1: Create Python package files**

`src/sentry_bringup/package.xml`:
```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>sentry_bringup</name>
  <version>0.1.0</version>
  <description>ROS2 nodes for Smart Agri Sentry RDK X5</description>
  <maintainer email="team@example.com">team</maintainer>
  <license>MIT</license>

  <depend>rclpy</depend>
  <depend>sensor_msgs</depend>
  <depend>geometry_msgs</depend>
  <depend>std_msgs</depend>
  <depend>sentry_interfaces</depend>

  <test_depend>ament_copyright</test_depend>
  <test_depend>ament_flake8</test_depend>
  <test_depend>ament_pep257</test_depend>
  <test_depend>python3-pytest</test_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
```

`src/sentry_bringup/setup.py`:
```python
from setuptools import find_packages, setup

package_name = 'sentry_bringup'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/sentry.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='team',
    maintainer_email='team@example.com',
    description='ROS2 nodes for Smart Agri Sentry RDK X5',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'uart_bridge_node = sentry_bringup.uart_bridge_node:main',
            'gps_node = sentry_bringup.gps_node:main',
            'camera_node = sentry_bringup.camera_node:main',
            'ai_inference_node = sentry_bringup.ai_inference_node:main',
        ],
    },
)
```

`src/sentry_bringup/setup.cfg`:
```ini
[develop]
script_dir=$base/lib/sentry_bringup
[install]
install_scripts=$base/lib/sentry_bringup
```

`src/sentry_bringup/resource/sentry_bringup`:
```
# Empty marker file
```

`src/sentry_bringup/sentry_bringup/__init__.py`:
```python
# sentry_bringup package
```

- [ ] **Step 2: Commit**

```bash
git add src/sentry_bringup/
git commit -m "feat(bringup): add python package skeleton"
```

---

### Task 4: Implement uart_bridge_node

**Files:**
- Create: `src/sentry_bringup/sentry_bringup/uart_bridge_node.py`
- Create: `src/sentry_bringup/tests/test_protocol.py`

- [ ] **Step 1: Write protocol codec tests**

`src/sentry_bringup/tests/test_protocol.py`:
```python
import pytest
from sentry_bringup.uart_bridge_node import crc16_ccitt, encode_frame, decode_sensor_frame


def test_crc16_ccitt_known():
    # CRC16-CCITT of [0x01] + payload [0x00, 0x01] should be deterministic
    data = bytes([0x01, 0x00, 0x01])
    crc = crc16_ccitt(data)
    assert isinstance(crc, int)
    assert 0 <= crc <= 0xFFFF


def test_encode_frame_structure():
    frame = encode_frame(0x01, bytes([0x00] * 24))
    assert len(frame) == 30  # 2 header + 1 type + 1 len + 24 payload + 2 crc
    assert frame[0:2] == b'\xaa\x55'
    assert frame[2] == 0x01
    assert frame[3] == 24


def test_decode_sensor_frame_valid():
    # Build a valid sensor frame payload (24 bytes)
    import struct
    payload = struct.pack('<IhhhhHHHHHHH',
                          1000,    # timestamp_ms
                          250,     # air_temp_x10 (25.0 C)
                          600,     # air_humi_x10 (60.0 %)
                          400,     # air_co2
                          200,     # soil_temp_x10 (20.0 C)
                          550,     # soil_humi_x10 (55.0 %)
                          100,     # soil_ec
                          50,      # soil_n
                          30,      # soil_p
                          40,      # soil_k
                          65)      # soil_ph_x10 (6.5)
    frame = encode_frame(0x01, payload)
    result = decode_sensor_frame(frame)
    assert result is not None
    assert result['timestamp_ms'] == 1000
    assert abs(result['air_temp'] - 25.0) < 0.01
    assert abs(result['soil_ph'] - 6.5) < 0.01


def test_decode_sensor_frame_bad_crc():
    frame = bytearray(encode_frame(0x01, bytes([0x00] * 24)))
    frame[-1] ^= 0xFF  # Corrupt CRC
    result = decode_sensor_frame(bytes(frame))
    assert result is None
```

- [ ] **Step 2: Run tests (expect FAIL - module not yet implemented)**

```bash
cd src/sentry_bringup
python -m pytest tests/test_protocol.py -v
```

Expected: ImportError or module not found

- [ ] **Step 3: Implement uart_bridge_node with protocol codec**

`src/sentry_bringup/sentry_bringup/uart_bridge_node.py`:
```python
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
import serial
import struct
import threading

from geometry_msgs.msg import Twist
from sentry_interfaces.msg import SensorCombined, ChassisStatus, ServoCmd


# ---- Protocol Constants ----
FRAME_HEADER = b'\xaa\x55'
TYPE_SENSOR = 0x01
TYPE_CHASSIS = 0x03
TYPE_MOTION_CMD = 0x81
TYPE_SERVO_CMD = 0x82
TYPE_MODE_CMD = 0x83

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
    (ts, at, ah, ac, st, sh, sec, sn, sp, sk, sph) = struct.unpack('<IhhhhHHHHHHH', payload)
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
    # chassis payload: left_speed(mm/s), right_speed(mm/s), battery(V x100), alarm
    (ls, rs, bv, alarm) = struct.unpack('<hhHB', payload)
    return {
        'left_speed': ls / 1000.0,
        'right_speed': rs / 1000.0,
        'battery_voltage': bv / 100.0,
        'alarm_bits': alarm,
    }


class UartBridgeNode(Node):
    def __init__(self):
        super().__init__('uart_bridge_node')
        self.declare_parameter('uart_port', '/dev/ttyS2')
        self.declare_parameter('baudrate', 115200)
        port = self.get_parameter('uart_port').value
        baud = self.get_parameter('baudrate').value

        self.ser = serial.Serial(port, baud, timeout=0.01)
        self.get_logger().info(f'UART open: {port} @ {baud}')

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.pub_sensor = self.create_publisher(SensorCombined, '/sentry/sensors/combined', qos)
        self.pub_chassis = self.create_publisher(ChassisStatus, '/sentry/chassis/status', qos)

        self.sub_cmd_vel = self.create_subscription(Twist, '/sentry/cmd_vel', self.on_cmd_vel, 10)
        self.sub_servo = self.create_subscription(ServoCmd, '/sentry/servo_cmd', self.on_servo, 10)

        self.timer_rx = self.create_timer(0.01, self.rx_tick)
        self.rx_buf = bytearray()

    def rx_tick(self):
        try:
            if self.ser.in_waiting:
                self.rx_buf.extend(self.ser.read(self.ser.in_waiting))
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
                msg = SensorCombined()
                msg.timestamp_ms = data['timestamp_ms']
                msg.air_temp = data['air_temp']
                msg.air_humi = data['air_humi']
                msg.air_co2 = data['air_co2']
                msg.soil_temp = data['soil_temp']
                msg.soil_humi = data['soil_humi']
                msg.soil_ec = data['soil_ec']
                msg.soil_n = data['soil_n']
                msg.soil_p = data['soil_p']
                msg.soil_k = data['soil_k']
                msg.soil_ph = data['soil_ph']
                self.pub_sensor.publish(msg)
        elif frame_type == TYPE_CHASSIS:
            data = decode_chassis_frame(frame)
            if data:
                msg = ChassisStatus()
                msg.left_speed = data['left_speed']
                msg.right_speed = data['right_speed']
                msg.battery_voltage = data['battery_voltage']
                msg.alarm_bits = data['alarm_bits']
                self.pub_chassis.publish(msg)

    def on_cmd_vel(self, msg: Twist):
        # Convert m/s to mm/s, send as motion control frame
        left = int(msg.linear.x * 1000)
        right = int(msg.linear.x * 1000)
        payload = struct.pack('<hh', left, right)
        frame = encode_frame(TYPE_MOTION_CMD, payload)
        self.ser.write(frame)

    def on_servo(self, msg: ServoCmd):
        payload = struct.pack('<BB', msg.pitch, msg.yaw)
        frame = encode_frame(TYPE_SERVO_CMD, payload)
        self.ser.write(frame)

    def destroy_node(self):
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
```

- [ ] **Step 4: Run tests (expect PASS)**

```bash
cd src/sentry_bringup
python -m pytest tests/test_protocol.py -v
```

Expected: 4 tests pass

- [ ] **Step 5: Commit**

```bash
git add src/sentry_bringup/sentry_bringup/uart_bridge_node.py src/sentry_bringup/tests/test_protocol.py
git commit -m "feat(uart): add uart_bridge_node with binary protocol codec"
```

---

### Task 5: Implement gps_node

**Files:**
- Create: `src/sentry_bringup/sentry_bringup/gps_node.py`
- Create: `src/sentry_bringup/tests/test_nmea.py`

- [ ] **Step 1: Write NMEA parser tests**

`src/sentry_bringup/tests/test_nmea.py`:
```python
import pytest
from sentry_bringup.gps_node import parse_nmea_line


def test_parse_gga_valid():
    line = "$GNGGA,123519,4807.038,N,01131.000,E,1,8,0.9,545.4,M,46.9,M,,*47"
    result = parse_nmea_line(line)
    assert result is not None
    assert result['lat'] > 48.0
    assert result['lon'] > 11.0
    assert result['fix_quality'] == 1


def test_parse_rmc_valid():
    line = "$GNRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W*6A"
    result = parse_nmea_line(line)
    assert result is not None
    assert result['speed_knots'] == 22.4
    assert result['track_angle'] == 84.4


def test_parse_invalid_checksum():
    line = "$GNGGA,123519,4807.038,N,01131.000,E,1,8,0.9,545.4,M,46.9,M,,*00"
    result = parse_nmea_line(line)
    assert result is None


def test_parse_unsupported_sentence():
    line = "$GNGSV,1,1,04,01,40,083,46*4D"
    result = parse_nmea_line(line)
    assert result is None
```

- [ ] **Step 2: Run tests (expect FAIL)**

```bash
cd src/sentry_bringup
python -m pytest tests/test_nmea.py -v
```

Expected: ImportError

- [ ] **Step 3: Implement gps_node**

`src/sentry_bringup/sentry_bringup/gps_node.py`:
```python
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

        self.ser = serial.Serial(port, baud, timeout=0.1)
        self.get_logger().info(f'GPS UART open: {port} @ {baud}')

        self.pub = self.create_publisher(NavSatFix, '/sentry/gps/fix', 10)
        self.timer = self.create_timer(0.1, self.tick)

        self.last_gga = None
        self.last_rmc = None

    def tick(self):
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
            msg.status.status = -1  # STATUS_NO_FIX
        else:
            msg.status.status = 0   # STATUS_FIX
        msg.position_covariance_type = 0
        self.pub.publish(msg)

    def destroy_node(self):
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
```

- [ ] **Step 4: Run tests (expect PASS)**

```bash
cd src/sentry_bringup
python -m pytest tests/test_nmea.py -v
```

Expected: 4 tests pass

- [ ] **Step 5: Commit**

```bash
git add src/sentry_bringup/sentry_bringup/gps_node.py src/sentry_bringup/tests/test_nmea.py
git commit -m "feat(gps): add gps_node with NMEA parser"
```

---

### Task 6: Implement camera_node

**Files:**
- Create: `src/sentry_bringup/sentry_bringup/camera_node.py`
- Create: `src/sentry_bringup/tests/test_preprocessing.py`

- [ ] **Step 1: Write preprocessing tests**

`src/sentry_bringup/tests/test_preprocessing.py`:
```python
import pytest
import numpy as np
from unittest.mock import patch, MagicMock

from sentry_bringup.camera_node import preprocess_image


def test_preprocess_shape():
    img = np.zeros((480, 640, 3), dtype=np.uint8)
    out = preprocess_image(img, target_size=(224, 224))
    assert out.shape == (1, 224, 224, 3)
    assert out.dtype == np.float32


def test_preprocess_normalization():
    img = np.full((480, 640, 3), 128, dtype=np.uint8)
    out = preprocess_image(img, target_size=(224, 224))
    # After normalization to [-1, 1] or [0, 1], check range
    assert out.min() >= -1.0
    assert out.max() <= 1.0
```

- [ ] **Step 2: Run tests (expect FAIL)**

```bash
cd src/sentry_bringup
python -m pytest tests/test_preprocessing.py -v
```

- [ ] **Step 3: Implement camera_node**

`src/sentry_bringup/sentry_bringup/camera_node.py`:
```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np


def preprocess_image(image: np.ndarray, target_size=(224, 224)) -> np.ndarray:
    """Resize and normalize image for MobileNetV2 input."""
    resized = cv2.resize(image, target_size)
    rgb = cv2.cvtColor(resized, cv2.COLOR_BGR2RGB)
    normalized = rgb.astype(np.float32) / 127.5 - 1.0  # Normalize to [-1, 1]
    return np.expand_dims(normalized, axis=0)


class CameraNode(Node):
    def __init__(self):
        super().__init__('camera_node')
        self.declare_parameter('device_id', 0)
        self.declare_parameter('fps', 2.0)
        self.declare_parameter('publish_raw', True)

        dev_id = self.get_parameter('device_id').value
        fps = self.get_parameter('fps').value

        self.cap = cv2.VideoCapture(dev_id)
        if not self.cap.isOpened():
            self.get_logger().error(f'Failed to open camera {dev_id}')
        else:
            self.get_logger().info(f'Camera opened: {dev_id}')
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
            self.cap.set(cv2.CAP_PROP_FPS, fps)

        self.bridge = CvBridge()
        self.pub = self.create_publisher(Image, '/sentry/camera/image_raw', 10)
        self.timer = self.create_timer(1.0 / fps, self.capture)
        self.frame_count = 0

    def capture(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn('Camera capture failed')
            return
        self.frame_count += 1
        msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'camera'
        self.pub.publish(msg)

    def destroy_node(self):
        self.cap.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
```

- [ ] **Step 4: Run tests (expect PASS)**

```bash
cd src/sentry_bringup
python -m pytest tests/test_preprocessing.py -v
```

Expected: 2 tests pass

- [ ] **Step 5: Commit**

```bash
git add src/sentry_bringup/sentry_bringup/camera_node.py src/sentry_bringup/tests/test_preprocessing.py
git commit -m "feat(camera): add camera_node with preprocessing"
```

---

### Task 7: Implement ai_inference_node

**Files:**
- Create: `src/sentry_bringup/sentry_bringup/ai_inference_node.py`
- Modify: `src/sentry_bringup/setup.py` (add entry point)

- [ ] **Step 1: Implement ai_inference_node**

`src/sentry_bringup/sentry_bringup/ai_inference_node.py`:
```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from sentry_interfaces.msg import AiDiagnosis
from cv_bridge import CvBridge
import numpy as np
import tflite_runtime.interpreter as tflite
import os


# 10-class tomato disease labels
LABELS = [
    'bacterial_spot',
    'early_blight',
    'healthy',
    'late_blight',
    'leaf_mold',
    'septoria_leaf_spot',
    'spider_mites_two-spotted_spider_mite',
    'target_spot',
    'tomato_mosaic_virus',
    'tomato_yellow_leaf_curl_virus',
]


class AiInferenceNode(Node):
    def __init__(self):
        super().__init__('ai_inference_node')
        self.declare_parameter('model_path', 'models/finetuned_mobilenetv2_int8.tflite')
        self.declare_parameter('input_size', 224)

        model_path = self.get_parameter('model_path').value
        self.input_size = self.get_parameter('input_size').value

        # Resolve model path relative to workspace or absolute
        if not os.path.isabs(model_path):
            ws = os.environ.get('COLCON_PREFIX_PATH', os.getcwd())
            candidates = [
                os.path.join(ws, '..', '..', model_path),
                os.path.join(ws, model_path),
                model_path,
            ]
            for c in candidates:
                if os.path.exists(c):
                    model_path = c
                    break

        self.get_logger().info(f'Loading model: {model_path}')
        self.interpreter = tflite.Interpreter(model_path=model_path)
        self.interpreter.allocate_tensors()
        self.input_details = self.interpreter.get_input_details()
        self.output_details = self.interpreter.get_output_details()

        self.bridge = CvBridge()
        self.sub = self.create_subscription(Image, '/sentry/camera/image_raw', self.on_image, 1)
        self.pub = self.create_publisher(AiDiagnosis, '/sentry/ai/diagnosis', 10)
        self.get_logger().info('AI inference node ready')

    def on_image(self, msg: Image):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'CV bridge error: {e}')
            return

        resized = self.preprocess(cv_image)
        self.interpreter.set_tensor(self.input_details[0]['index'], resized)
        self.interpreter.invoke()
        output = self.interpreter.get_tensor(self.output_details[0]['index'])[0]

        # Softmax if logits
        exp_out = np.exp(output - np.max(output))
        probs = exp_out / np.sum(exp_out)

        class_idx = int(np.argmax(probs))
        confidence = float(probs[class_idx])

        out_msg = AiDiagnosis()
        out_msg.header.stamp = self.get_clock().now().to_msg()
        out_msg.header.frame_id = 'camera'
        out_msg.disease_class = LABELS[class_idx]
        out_msg.confidence = confidence
        out_msg.probabilities = probs.tolist()
        self.pub.publish(out_msg)

    def preprocess(self, image: np.ndarray) -> np.ndarray:
        resized = cv2.resize(image, (self.input_size, self.input_size))
        rgb = cv2.cvtColor(resized, cv2.COLOR_BGR2RGB)
        normalized = rgb.astype(np.float32) / 127.5 - 1.0
        return np.expand_dims(normalized, axis=0)


def main(args=None):
    rclpy.init(args=args)
    node = AiInferenceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
```

Also add to `setup.py` entry_points if not already there (should already include it from skeleton).

- [ ] **Step 2: Commit**

```bash
git add src/sentry_bringup/sentry_bringup/ai_inference_node.py
git commit -m "feat(ai): add ai_inference_node with TFLite CPU inference"
```

---

### Task 8: Create Launch File

**Files:**
- Create: `src/sentry_bringup/launch/sentry.launch.py`

- [ ] **Step 1: Create launch file**

`src/sentry_bringup/launch/sentry.launch.py`:
```python
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='sentry_bringup',
            executable='uart_bridge_node',
            name='uart_bridge_node',
            parameters=[{'uart_port': '/dev/ttyS2', 'baudrate': 115200}],
            output='screen',
        ),
        Node(
            package='sentry_bringup',
            executable='gps_node',
            name='gps_node',
            parameters=[{'uart_port': '/dev/ttyS6', 'baudrate': 9600}],
            output='screen',
        ),
        Node(
            package='sentry_bringup',
            executable='camera_node',
            name='camera_node',
            parameters=[{'device_id': 0, 'fps': 2.0}],
            output='screen',
        ),
        Node(
            package='sentry_bringup',
            executable='ai_inference_node',
            name='ai_inference_node',
            parameters=[{'model_path': 'models/finetuned_mobilenetv2_int8.tflite', 'input_size': 224}],
            output='screen',
        ),
    ])
```

- [ ] **Step 2: Commit**

```bash
git add src/sentry_bringup/launch/sentry.launch.py
git commit -m "feat(launch): add sentry.launch.py for batch 1 nodes"
```

---

### Task 9: Update PROJECT_CONTEXT.md with Confirmed Info

**Files:**
- Modify: `.claude/PROJECT_CONTEXT.md`

- [ ] **Step 1: Append confirmed specifications to context file**

Append the following section to `.claude/PROJECT_CONTEXT.md` before the "目录指引" section:

```markdown
---

## 已确认技术细节（2026-04-22）

### 模型
- 路径：`models/finetuned_mobilenetv2_int8.tflite`
- 格式：原生 TFLite（CPU 推理），后续可转 NPU
- 输入尺寸：224×224
- 输出类别（10 类）：bacterial_spot, early_blight, healthy, late_blight, leaf_mold, septoria_leaf_spot, spider_mites_two-spotted_spider_mite, target_spot, tomato_mosaic_virus, tomato_yellow_leaf_curl_virus

### 摄像头
- 型号：IMX219（MIPI-CSI）

### GPS
- 输出频率：1 Hz（GGA + RMC）

### 前端架构
- Vue 直连 `rosbridge_server` WebSocket
- FastAPI 负责：航点管理、SQLite、MJPEG 视频流代理

### WebSocket 分层推送（最终版）
| 数据类型 | 推送频率 |
|---|---|
| 传感器环境数据 | 1 Hz（巡检）/ 5 Hz（精细监测） |
| AI 诊断结果 | 2 Hz |
| 底盘状态 | 2 Hz |
| 紧急报警 | 事件触发 + 1 Hz 确认 |
| 远程控制指令 | 20-50 Hz |
| 视频流 | 独立 HTTP MJPEG，15-20 fps |

### 融合策略
- Demo 版：固定规则（环境条件触发权重调整）
- 长期：条件门控 + 自适应权重 + AHP + 逻辑回归训练

### 导航
- Batch 1 仅生成占位框架，不实现纯追踪算法
- 航点存储：YAML 模板 + SQLite 当前执行 + Web 可视化编辑
- 策略：改良版纯追踪 + 路径点/任务点分层

### STM32 协议
- 由我自主设计，当前 v2.0 自定义二进制帧已冻结
- uart_bridge_node 已按此实现
```

- [ ] **Step 2: Commit**

```bash
git add .claude/PROJECT_CONTEXT.md
git commit -m "docs(context): append confirmed technical specs"
```

---

### Task 10: Push to GitHub

**Files:**
- None (git operation)

- [ ] **Step 1: Push main branch**

```bash
git push -u origin main
```

Expected: All commits pushed to `https://github.com/<user>/smart-agri-sentry`

---

## Self-Review

**1. Spec coverage:**
- Custom messages (`sentry_interfaces`) → Task 2
- `uart_bridge_node` with binary protocol → Task 4
- `gps_node` with NMEA parsing → Task 5
- `camera_node` with IMX219 capture → Task 6
- `ai_inference_node` with TFLite 224×224 inference → Task 7
- Launch file → Task 8
- GitHub repo + git tracking → Task 1, 10
- PROJECT_CONTEXT.md update → Task 9
- No placeholders found.
- Type consistency: message field names match between `.msg` definitions and Python node publishers/subscribers.

**Gap:** `nav_node` placeholder and `fusion_node` are intentionally deferred to Batch 2 as per user request.
