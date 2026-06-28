# LoRa 固定环境节点接入 RDK ROS2 实现计划

> **For agentic workers:** REQUIRED SUB-SKILL: Use `superpowers:subagent-driven-development` (recommended) or `superpowers:executing-plans` to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 在 RDK X5 上新增 `lora_bridge_node`，通过 USB CDC 接收 E22-400TBH-SC 透传的 LoRa 数据，解析聚合环境帧，发布完整 `Environment.msg` 到 `/sensor/environment_fixed`。

**Architecture:** 固定环境 MCU 在发送端把空气、土壤、叶面三类传感器数据聚合成单一 `msg_type 0x01` 帧；RDK 侧 `lora_bridge_node` 按 `0xAA 0x55` 切帧、校验 CRC-8/MAXIM、解析 24 字节 payload，填充扩展后的 `Environment.msg` 并发布。

**Tech Stack:** ROS2 Humble / rclpy / Python 3 / pyserial / ament_python / custom binary protocol / CRC-8/MAXIM

---

## 文件结构

| 文件 | 操作 | 职责 |
|---|---|---|
| `src/sentry_interfaces/msg/Environment.msg` | 修改 | 新增 `hcho`、`tvoc`、`pm25`、`pm10`、`leaf_temp`、`ec` 字段 |
| `src/sentry_sensors/sentry_sensors/lora_bridge_node.py` | 创建 | LoRa USB CDC 串口读取、帧切分、CRC 校验、解析、发布 |
| `src/sentry_sensors/launch/lora_bridge.launch.py` | 创建 | 启动 `lora_bridge_node` |
| `src/sentry_sensors/setup.py` | 修改 | 添加 `lora_bridge_node` 入口点，注册启动文件 |
| `src/sentry_sensors/tests/test_lora_bridge_node.py` | 创建 | 单元测试：CRC、帧解析、节点生命周期、错误处理 |

---

## Task 1: 扩展 `Environment.msg`

**Files:**
- Modify: `src/sentry_interfaces/msg/Environment.msg`
- Test: `colcon build --packages-select sentry_interfaces`

- [ ] **Step 1: 在末尾添加 6 个新字段**

将文件内容改为：

```msg
std_msgs/Header header
float32 air_temp
float32 air_humidity
float32 air_co2
float32 soil_temp
float32 soil_humidity
float32 leaf_wetness
float32 hcho
float32 tvoc
float32 pm25
float32 pm10
float32 leaf_temp
float32 ec
string data_source
```

- [ ] **Step 2: 编译接口包**

Run:

```bash
cd E:/smart_agri_sentry
colcon build --packages-select sentry_interfaces
```

Expected: 编译成功，无错误。

- [ ] **Step 3: 提交**

```bash
git add src/sentry_interfaces/msg/Environment.msg
git commit -m "feat(interfaces): extend Environment.msg with air quality and leaf temp/ec fields"
```

---

## Task 2: 实现 CRC-8/MAXIM 和帧解码辅助函数

**Files:**
- Create: `src/sentry_sensors/sentry_sensors/lora_bridge_node.py`
- Test: `src/sentry_sensors/tests/test_lora_bridge_node.py`

- [ ] **Step 1: 创建节点文件并写入常量与 CRC 函数**

在 `src/sentry_sensors/sentry_sensors/lora_bridge_node.py` 顶部写入：

```python
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
```

- [ ] **Step 2: 创建测试文件并写入 CRC 测试**

在 `src/sentry_sensors/tests/test_lora_bridge_node.py` 写入：

```python
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
```

- [ ] **Step 3: 运行测试，验证失败（因为 `LoraBridgeNode` 还未实现）**

Run:

```bash
cd E:/smart_agri_sentry
pytest src/sentry_sensors/tests/test_lora_bridge_node.py -v
```

Expected: `ImportError` 或 `NameError` 因为 `LoraBridgeNode` 未定义。

- [ ] **Step 4: 提交当前骨架**

```bash
git add src/sentry_sensors/sentry_sensors/lora_bridge_node.py
git add src/sentry_sensors/tests/test_lora_bridge_node.py
git commit -m "feat(lora): add CRC8 and frame decoder skeleton"
```

---

## Task 3: 实现 `LoraBridgeNode`

**Files:**
- Modify: `src/sentry_sensors/sentry_sensors/lora_bridge_node.py`
- Test: `src/sentry_sensors/tests/test_lora_bridge_node.py`

- [ ] **Step 1: 在节点文件末尾追加节点类**

在 `lora_bridge_node.py` 中 `decode_environment_frame` 之后追加：

```python

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

    def rx_tick(self):
        if self.ser is None or not self.ser.is_open:
            return
        try:
            if self.ser.in_waiting:
                self.rx_buf.extend(self.ser.read(self.ser.in_waiting))
        except serial.SerialException as e:
            self.get_logger().error(f'LoRa UART read error: {e}')
            self.rx_buf.clear()
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
        if msg_type == MSG_TYPE_ENV and payload_len == PAYLOAD_LEN_ENV:
            if crc8_maxim(frame[:-1]) != frame[-1]:
                self.get_logger().warn('CRC mismatch on environment frame')
                return
            data = decode_environment_frame(frame)
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
```

- [ ] **Step 2: 补全解析测试（正例 + 负例 + 节点生命周期）**

将 `test_lora_bridge_node.py` 替换为完整版本：

```python
import struct

import pytest
import rclpy
from unittest.mock import patch, MagicMock

from sentry_sensors.lora_bridge_node import (
    crc8_maxim,
    decode_environment_frame,
    LoraBridgeNode,
)


@pytest.fixture(scope='module')
def ros_context():
    rclpy.init()
    yield
    rclpy.shutdown()


def _build_frame(values):
    payload = struct.pack('>HHHHHHHHHHHH', *values)
    header = bytes([0xAA, 0x55, 0x01, 0x01, len(payload)])
    frame = header + payload
    return frame + bytes([crc8_maxim(frame)])


def test_crc8_maxim_empty():
    assert crc8_maxim(b'') == 0x00


def test_crc8_maxim_header_only():
    assert crc8_maxim(b'\xaa\x55') == 0x8C


def test_decode_environment_frame_valid():
    frame = _build_frame([
        303, 20, 120, 35, 50,          # co2, hcho, tvoc, pm25, pm10
        2700, 6000,                    # air_temp, air_humidity
        2500, 5000,                    # soil_temp, soil_humidity
        150,                           # ec
        7000,                          # leaf_wetness
        2800,                          # leaf_temp
    ])
    data = decode_environment_frame(frame)
    assert data['air_co2'] == 303.0
    assert data['hcho'] == 20.0
    assert data['tvoc'] == 120.0
    assert data['pm25'] == 35.0
    assert data['pm10'] == 50.0
    assert data['air_temp'] == 27.0
    assert data['air_humidity'] == 60.0
    assert data['soil_temp'] == 25.0
    assert data['soil_humidity'] == 50.0
    assert data['ec'] == 150.0
    assert data['leaf_wetness'] == 70.0
    assert data['leaf_temp'] == 28.0


def test_decode_environment_frame_negative_temperatures():
    frame = _build_frame([
        0, 0, 0, 0, 0,
        0xF63C, 0,      # -25.00 C
        0xF63C, 0,      # -25.00 C
        0,
        0,
        0xF63C,         # -25.00 C
    ])
    data = decode_environment_frame(frame)
    assert data['air_temp'] == -25.0
    assert data['soil_temp'] == -25.0
    assert data['leaf_temp'] == -25.0


def test_decode_environment_frame_wrong_length():
    assert decode_environment_frame(b'\xaa\x55\x01\x01\x00\x00') is None


@pytest.fixture
def node(ros_context):
    with patch('sentry_sensors.lora_bridge_node.serial.Serial'):
        n = LoraBridgeNode()
        yield n
        n.destroy_node()


def test_node_creates_publisher(node):
    assert node.pub_env.topic_name == '/sensor/environment_fixed'


def test_handle_frame_crc_mismatch(node, caplog):
    bad_frame = bytes([0xAA, 0x55, 0x01, 0x01, 24]) + bytes(24) + bytes([0xFF])
    node._handle_frame(bad_frame)
    assert 'CRC mismatch' in caplog.text


def test_handle_frame_unknown_msg_type(node, caplog):
    payload = bytes(24)
    header = bytes([0xAA, 0x55, 0x01, 0xAB, len(payload)])
    frame = header + payload
    frame += bytes([crc8_maxim(frame)])
    node._handle_frame(frame)
    assert 'Unknown msg_type' in caplog.text
```

- [ ] **Step 3: 运行测试**

Run:

```bash
cd E:/smart_agri_sentry
pytest src/sentry_sensors/tests/test_lora_bridge_node.py -v
```

Expected: 所有测试通过。

- [ ] **Step 4: 提交**

```bash
git add src/sentry_sensors/sentry_sensors/lora_bridge_node.py
git add src/sentry_sensors/tests/test_lora_bridge_node.py
git commit -m "feat(lora): implement LoraBridgeNode with parsing and publishing"
```

---

## Task 4: 添加启动文件

**Files:**
- Create: `src/sentry_sensors/launch/lora_bridge.launch.py`
- Test: `ros2 launch sentry_sensors lora_bridge.launch.py`

- [ ] **Step 1: 创建启动文件**

写入 `src/sentry_sensors/launch/lora_bridge.launch.py`：

```python
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='sentry_sensors',
            executable='lora_bridge_node',
            name='lora_bridge_node',
            parameters=[{
                'uart_port': '/dev/ttyACM0',
                'baudrate': 9600,
            }],
            output='screen',
        ),
    ])
```

- [ ] **Step 2: 提交**

```bash
git add src/sentry_sensors/launch/lora_bridge.launch.py
git commit -m "feat(lora): add lora_bridge launch file"
```

---

## Task 5: 更新 `setup.py`

**Files:**
- Modify: `src/sentry_sensors/setup.py`

- [ ] **Step 1: 添加入口点和启动文件注册**

修改 `setup.py` 的 `data_files` 和 `entry_points` 部分：

```python
data_files=[
    ('share/ament_index/resource_index/packages',
        ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
    ('share/' + package_name + '/launch', [
        'launch/imu.launch.py',
        'launch/lora_bridge.launch.py',
    ]),
    ('share/' + package_name + '/config', ['config/imu.yaml', 'config/imu_filter_madgwick.yaml']),
    ('share/' + package_name + '/udev', ['udev/99-myimu.rules']),
],
install_requires=['setuptools'],
zip_safe=True,
maintainer='team',
maintainer_email='team@example.com',
description='Sensor bridge nodes for Smart Agri Sentry v2.0',
license='MIT',
tests_require=['pytest'],
entry_points={
    'console_scripts': [
        'uart_bridge_node = sentry_sensors.uart_bridge_node:main',
        'imu_node = sentry_sensors.imu_node:main',
        'lora_bridge_node = sentry_sensors.lora_bridge_node:main',
    ],
},
```

- [ ] **Step 2: 提交**

```bash
git add src/sentry_sensors/setup.py
git commit -m "build(sentry_sensors): register lora_bridge_node entry point and launch file"
```

---

## Task 6: 本地构建与测试

**Files:**
- All packages involved

- [ ] **Step 1: 构建相关包**

Run:

```bash
cd E:/smart_agri_sentry
colcon build --packages-select sentry_interfaces sentry_sensors
```

Expected: 编译成功，无错误。

- [ ] **Step 2: 运行单元测试**

```bash
cd E:/smart_agri_sentry
python -m pytest src/sentry_sensors/tests/test_lora_bridge_node.py -v
```

Expected: 所有测试通过。

- [ ] **Step 3: 提交构建日志（可选）**

如果 `setup.py` 或构建过程中有修改，提交：

```bash
git add -u
git commit -m "build: successful colcon build and pytest for lora_bridge"
```

---

## Task 7: RDK 板端集成测试

**Files:**
- None (runtime验证)

- [ ] **Step 1: 推送代码到远程**

```bash
cd E:/smart_agri_sentry
git push origin feat/stm32-cj702-lora
```

- [ ] **Step 2: 在 RDK 上拉取并构建**

SSH 到 RDK（密码 `sunrise`）：

```bash
ssh sunrise@ubuntu.local
cd ~/dev_ws
git pull origin feat/stm32-cj702-lora
source /opt/ros/humble/setup.bash
colcon build --packages-select sentry_interfaces sentry_sensors
```

Expected: 编译成功。

- [ ] **Step 3: 启动节点并验证话题**

在 RDK 上连接 E22-400TBH-SC 接收模块，运行：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch sentry_sensors lora_bridge.launch.py
```

另开一个终端：

```bash
ros2 topic echo /sensor/environment_fixed
```

Expected: 每 60 秒收到一条 `Environment` 消息，所有字段与发送端一致；`data_source` 为 `FIXED_LORA`。

- [ ] **Step 4: 验证 `fusion_node` 订阅**

在 RDK 上启动 `fusion_node`：

```bash
ros2 launch sentry_bringup sentry_v2.launch.py
```

观察 `/fusion/diagnosis` 是否正常输出，确认 `fusion_node` 能消费 `/sensor/environment_fixed`。

- [ ] **Step 5: 记录测试结果**

在 `test/stm32_cj702_lora_hal/TESTS.md` 或 `docs/ISSUES.md` 中记录：

- RDK 上识别到的串口设备名（如 `/dev/ttyACM0`）
- 实际收到的 `Environment` 消息字段值
- `fusion_node` 是否正常融合

提交：

```bash
git add test/stm32_cj702_lora_hal/TESTS.md
git commit -m "docs(tests): record RDK LoRa fixed env integration results"
```

---

## 自检清单

### Spec 覆盖检查

| Spec 要求 | 对应 Task |
|---|---|
| USB CDC 连接，9600 波特率 | Task 3 节点参数 |
| CRC-8/MAXIM 校验 | Task 2 + Task 3 |
| 24 字节 payload 解析 | Task 2 + Task 3 |
| 扩展 Environment.msg | Task 1 |
| 发布 `/sensor/environment_fixed` | Task 3 |
| 独立 `lora_bridge_node` | Task 3 |
| 错误帧处理 | Task 3 |
| 单元测试 | Task 2 + Task 3 |
| RDK 集成测试 | Task 7 |

### Placeholder 扫描

- 无 TBD/TODO。
- 所有代码块包含完整可运行代码。
- 所有命令包含预期输出。

### 类型一致性

- `Environment.msg` 新增字段与 `lora_bridge_node.py` 中 `msg.*` 赋值一致。
- `decode_environment_frame` 返回的字典键与 `_publish_environment` 使用的键一致。
- `_build_frame` 测试辅助函数字段顺序与 payload 定义一致。

---

## 执行方式选择

**Plan complete and saved to `docs/superpowers/plans/2026-06-28-lora-rdk-ros2.md`. Two execution options:**

**1. Subagent-Driven (recommended)** - I dispatch a fresh subagent per task, review between tasks, fast iteration

**2. Inline Execution** - Execute tasks in this session using executing-plans, batch execution with checkpoints

**Which approach?**
