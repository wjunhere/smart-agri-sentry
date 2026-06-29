# IMU 驱动集成实施计划

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 将 example/imu_ros2_device 示例驱动集成到 src/sentry_sensors 包，修复 5 个潜在问题，配置 Madgwick 滤波，接入 sentry_v2.launch.py。

**Architecture:** 在 sentry_sensors 包内新增 imu_node.py，参数化驱动（串口/频率/协方差），通过 imu.launch.py 同时启动驱动节点 + Madgwick 滤波 + static_transform_publisher，最终由 IncludeLaunchDescription 接入主启动文件。

**Tech Stack:** ROS2 Humble, Python 3, ament_python, imu_filter_madgwick (ROS2 包)

---

## 文件结构

```
src/sentry_sensors/
├── sentry_sensors/
│   ├── __init__.py
│   ├── uart_bridge_node.py
│   └── imu_node.py                       [新增] 核心驱动节点
├── config/
│   ├── imu.yaml                          [新增] IMU 节点参数
│   └── imu_filter_madgwick.yaml          [新增] Madgwick 配置
├── launch/
│   └── imu.launch.py                     [新增] 启动描述
├── udev/
│   └── 99-myimu.rules                    [新增] udev 规则
├── tests/
│   └── test_imu_node.py                  [新增] 单元测试
├── setup.py                              [修改] + entry_point
└── package.xml                           [修改] + 依赖

src/sentry_bringup/launch/
└── sentry_v2.launch.py                   [修改] Include imu.launch.py
```

---

## Task 1: 修改包元数据（package.xml + setup.py）

**Files:**
- Modify: `src/sentry_sensors/package.xml`
- Modify: `src/sentry_sensors/setup.py`

- [ ] **Step 1: 修改 package.xml 添加依赖**

```xml
  <depend>sensor_msgs</depend>
  <depend>geometry_msgs</depend>
```

插入到 `</package>` 之前、现有 `<depend>` 之后。

- [ ] **Step 2: 修改 setup.py 添加 entry_point 和 data_files**

在 `entry_points` 的 `console_scripts` 列表中新增一行：
```python
            'imu_node = sentry_sensors.imu_node:main',
```

在 `data_files` 列表中新增 launch 和 config 目录的数据文件注册（参考现有 sentry_lidar 的 CMakeLists 或 sentry_bringup 的 setup.py 模式）。对于 ament_python 包，需要显式列出非 Python 文件：

```python
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/imu.launch.py']),
        ('share/' + package_name + '/config', ['config/imu.yaml', 'config/imu_filter_madgwick.yaml']),
        ('share/' + package_name + '/udev', ['udev/99-myimu.rules']),
    ],
```

- [ ] **Step 3: Commit**

```bash
git add src/sentry_sensors/package.xml src/sentry_sensors/setup.py
git commit -m "chore(sentry_sensors): add imu_node entry_point and dependencies"
```

---

## Task 2: TDD — 写单元测试

**Files:**
- Create: `src/sentry_sensors/tests/test_imu_node.py`

- [ ] **Step 1: 创建测试文件骨架**

```python
import unittest
import math

from sentry_sensors.imu_node import ImuNode


class TestQuaternionNormalize(unittest.TestCase):
    """Test the quaternion normalization helper."""

    def test_unit_quaternion_unchanged(self):
        q = [1.0, 0.0, 0.0, 0.0]
        result = ImuNode._normalize_quaternion_static(q)
        self.assertEqual(result, [1.0, 0.0, 0.0, 0.0])

    def test_zero_quaternion_fallback(self):
        q = [0.0, 0.0, 0.0, 0.0]
        result = ImuNode._normalize_quaternion_static(q)
        self.assertEqual(result, [1.0, 0.0, 0.0, 0.0])

    def test_large_norm_normalized(self):
        q = [2.0, 0.0, 0.0, 0.0]
        result = ImuNode._normalize_quaternion_static(q)
        self.assertEqual(result, [1.0, 0.0, 0.0, 0.0])

    def test_non_unit_normalized(self):
        q = [0.5, 0.5, 0.5, 0.5]
        result = ImuNode._normalize_quaternion_static(q)
        norm = math.sqrt(sum(x * x for x in result))
        self.assertAlmostEqual(norm, 1.0, places=6)


class TestCovarianceStructure(unittest.TestCase):
    """Test covariance matrix helpers."""

    def test_build_covariance_3x3(self):
        flat = [0.0005, 0.0, 0.0, 0.0, 0.0005, 0.0, 0.0, 0.0, 0.0008]
        matrix = ImuNode._build_covariance_matrix(flat)
        self.assertEqual(len(matrix), 9)
        self.assertEqual(matrix[0], 0.0005)
        self.assertEqual(matrix[4], 0.0005)
        self.assertEqual(matrix[8], 0.0008)


if __name__ == '__main__':
    unittest.main()
```

- [ ] **Step 2: 运行测试确认失败**

```bash
cd src/sentry_sensors && python -m pytest tests/test_imu_node.py -v
```

**Expected:** `ModuleNotFoundError: No module named 'sentry_sensors.imu_node'` 或 `AttributeError: type object 'ImuNode' has no attribute '_normalize_quaternion_static'`

- [ ] **Step 3: Commit 测试文件**

```bash
git add src/sentry_sensors/tests/test_imu_node.py
git commit -m "test(sentry_sensors): add imu_node unit tests (failing)"
```

---

## Task 3: 实现 imu_node.py

**Files:**
- Create: `src/sentry_sensors/sentry_sensors/imu_node.py`

- [ ] **Step 1: 编写 imu_node.py 完整实现**

```python
#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
import math

import rclpy
from rclpy.node import Node
from rclpy.clock import Clock
from sensor_msgs.msg import Imu, MagneticField
from std_msgs.msg import Float32MultiArray

# External library: YbImuLib must be installed on target platform
from YbImuLib import YbImuSerial


class ImuNode(Node):
    """YB-IMU sensor driver for Smart Agri Sentry."""

    def __init__(self):
        super().__init__('imu_node')
        self.robot = None

        # Declare parameters
        self.declare_parameter('port', '/dev/myimu')
        self.declare_parameter('frame_id', 'imu_link')
        self.declare_parameter('pub_rate_hz', 100.0)
        self.declare_parameter('use_mag', True)
        self.declare_parameter(
            'linear_accel_cov',
            [0.0005, 0.0, 0.0, 0.0, 0.0005, 0.0, 0.0, 0.0, 0.0008]
        )
        self.declare_parameter(
            'angular_vel_cov',
            [0.00002, 0.0, 0.0, 0.0, 0.00002, 0.0, 0.0, 0.0, 0.00005]
        )
        self.declare_parameter(
            'orientation_cov',
            [0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.2]
        )

        self.port = self.get_parameter('port').value
        self.frame_id = self.get_parameter('frame_id').value
        self.pub_rate_hz = self.get_parameter('pub_rate_hz').value
        self.use_mag = self.get_parameter('use_mag').value

        self.linear_accel_cov = self._build_covariance_matrix(
            self.get_parameter('linear_accel_cov').value
        )
        self.angular_vel_cov = self._build_covariance_matrix(
            self.get_parameter('angular_vel_cov').value
        )
        self.orientation_cov = self._build_covariance_matrix(
            self.get_parameter('orientation_cov').value
        )

        self._init_serial()
        if self.robot is None:
            self.get_logger().error('Failed to initialize IMU serial port')
            return

        self._init_publishers()
        self._init_timer()

    @staticmethod
    def _normalize_quaternion_static(q):
        """Normalize a quaternion. Returns unit quaternion if input is zero."""
        norm = math.sqrt(sum(x * x for x in q))
        if norm < 1e-6:
            return [1.0, 0.0, 0.0, 0.0]
        return [x / norm for x in q]

    @staticmethod
    def _build_covariance_matrix(flat_list):
        """Build a 9-element covariance list from flat input."""
        if len(flat_list) != 9:
            return [0.0] * 9
        return list(flat_list)

    def _init_serial(self):
        """Initialize serial connection to IMU."""
        try:
            self.robot = YbImuSerial(self.port)
            self.get_logger().info(f'Opened IMU serial port: {self.port}')
            self.robot.create_receive_threading()
        except Exception as e:
            self.get_logger().error(
                f'Failed to open IMU serial port {self.port}: {e}'
            )
            self.robot = None

    def _init_publishers(self):
        self.imu_publisher = self.create_publisher(
            Imu, '/sensor/imu/data_raw', 100
        )
        self.mag_publisher = self.create_publisher(
            MagneticField, '/sensor/imu/mag', 100
        )
        self.baro_publisher = self.create_publisher(
            Float32MultiArray, '/sensor/imu/baro', 100
        )
        self.euler_publisher = self.create_publisher(
            Float32MultiArray, '/sensor/imu/euler', 100
        )

    def _init_timer(self):
        period = 1.0 / self.pub_rate_hz
        self.timer = self.create_timer(period, self._pub_data)
        self.get_logger().info(
            f'IMU publisher timer started at {self.pub_rate_hz} Hz'
        )

    def _pub_data(self):
        if self.robot is None:
            return

        time_stamp = Clock().now()
        imu = Imu()
        mag = MagneticField()
        baro = Float32MultiArray()
        euler = Float32MultiArray()

        try:
            [ax, ay, az] = self.robot.get_accelerometer_data()
            [gx, gy, gz] = self.robot.get_gyroscope_data()
            [mx, my, mz] = self.robot.get_magnetometer_data()
            [q0, q1, q2, q3] = self.robot.get_imu_quaternion_data()
            [height, temperature, pressure, pressure_contrast] = self.robot.get_baro_data()
            [roll, pitch, yaw] = self.robot.get_imu_attitude_data(True)
        except Exception as e:
            self.get_logger().warning(f'Failed to read IMU data: {e}')
            return

        # Normalize quaternion and warn if significantly off
        q = self._normalize_quaternion_static([q0, q1, q2, q3])
        norm_before = math.sqrt(sum(x * x for x in [q0, q1, q2, q3]))
        if abs(norm_before - 1.0) > 0.01:
            self.get_logger().warning(
                f'Quaternion norm deviated: {norm_before:.4f}, normalized'
            )

        # Fill IMU message
        imu.header.stamp = time_stamp.to_msg()
        imu.header.frame_id = self.frame_id
        imu.linear_acceleration.x = float(ax)
        imu.linear_acceleration.y = float(ay)
        imu.linear_acceleration.z = float(az)
        imu.linear_acceleration_covariance = self.linear_accel_cov
        imu.angular_velocity.x = float(gx)
        imu.angular_velocity.y = float(gy)
        imu.angular_velocity.z = float(gz)
        imu.angular_velocity_covariance = self.angular_vel_cov
        imu.orientation.w = q[0]
        imu.orientation.x = q[1]
        imu.orientation.y = q[2]
        imu.orientation.z = q[3]
        imu.orientation_covariance = self.orientation_cov

        # Fill magnetometer message
        mag.header.stamp = time_stamp.to_msg()
        mag.header.frame_id = self.frame_id
        mag.magnetic_field.x = float(mx)
        mag.magnetic_field.y = -float(my)  # Y-axis sign flip for ENU convention
        mag.magnetic_field.z = float(mz)

        # Fill barometer and euler messages
        baro.data = [float(height), float(temperature),
                     float(pressure), float(pressure_contrast)]
        euler.data = [float(roll), float(pitch), float(yaw)]

        self.imu_publisher.publish(imu)
        if self.use_mag:
            self.mag_publisher.publish(mag)
        self.baro_publisher.publish(baro)
        self.euler_publisher.publish(euler)


def main(args=None):
    rclpy.init(args=args)
    node = ImuNode()
    if node.robot is None:
        node.destroy_node()
        rclpy.shutdown()
        return 1
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
```

- [ ] **Step 2: 运行测试确认通过**

```bash
cd src/sentry_sensors && python -m pytest tests/test_imu_node.py -v
```

**Expected:** 所有 5 个测试 PASS

- [ ] **Step 3: Commit**

```bash
git add src/sentry_sensors/sentry_sensors/imu_node.py
git commit -m "feat(sentry_sensors): add imu_node with parametric config and quaternion normalization"
```

---

## Task 4: 配置文件

**Files:**
- Create: `src/sentry_sensors/config/imu.yaml`
- Create: `src/sentry_sensors/config/imu_filter_madgwick.yaml`

- [ ] **Step 1: 创建 imu.yaml**

```yaml
imu_node:
  ros__parameters:
    port: "/dev/myimu"
    frame_id: "imu_link"
    pub_rate_hz: 100.0
    use_mag: true
    linear_accel_cov: [0.0005, 0.0, 0.0, 0.0, 0.0005, 0.0, 0.0, 0.0, 0.0008]
    angular_vel_cov: [0.00002, 0.0, 0.0, 0.0, 0.00002, 0.0, 0.0, 0.0, 0.00005]
    orientation_cov: [0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.2]
```

- [ ] **Step 2: 创建 imu_filter_madgwick.yaml**

```yaml
imu_filter_madgwick:
  ros__parameters:
    fixed_frame: "odom"
    use_mag: true
    publish_tf: false
    world_frame: "enu"
    gain: 0.1
    zeta: 0.0
    stateless: false
```

- [ ] **Step 3: Commit**

```bash
git add src/sentry_sensors/config/
git commit -m "config(sentry_sensors): add imu and madgwick filter parameters"
```

---

## Task 5: Launch 文件

**Files:**
- Create: `src/sentry_sensors/launch/imu.launch.py`

- [ ] **Step 1: 创建 imu.launch.py**

```python
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('sentry_sensors')

    imu_config = os.path.join(pkg_share, 'config', 'imu.yaml')
    madgwick_config = os.path.join(
        pkg_share, 'config', 'imu_filter_madgwick.yaml'
    )

    imu_node = Node(
        package='sentry_sensors',
        executable='imu_node',
        name='imu_node',
        parameters=[imu_config],
        output='screen',
    )

    imu_filter_node = Node(
        package='imu_filter_madgwick',
        executable='imu_filter_madgwick_node',
        name='imu_filter_madgwick',
        parameters=[madgwick_config],
        remappings=[
            ('/imu/data_raw', '/sensor/imu/data_raw'),
            ('/imu/mag', '/sensor/imu/mag'),
            ('/imu/data', '/sensor/imu/data'),
        ],
        output='screen',
    )

    # Static TF: base_link -> imu_link
    # Adjust xyz/rpy if IMU is not mounted at robot center
    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='imu_static_tf',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'imu_link'],
    )

    return LaunchDescription([
        imu_node,
        imu_filter_node,
        static_tf_node,
    ])
```

- [ ] **Step 2: Commit**

```bash
git add src/sentry_sensors/launch/imu.launch.py
git commit -m "feat(sentry_sensors): add imu launch with driver, madgwick filter and static tf"
```

---

## Task 6: udev 规则

**Files:**
- Create: `src/sentry_sensors/udev/99-myimu.rules`

- [ ] **Step 1: 创建规则文件**

```
KERNEL=="ttyUSB*", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="7523", MODE:="0777", SYMLINK+="myimu"
```

- [ ] **Step 2: Commit**

```bash
git add src/sentry_sensors/udev/99-myimu.rules
git commit -m "chore(sentry_sensors): add udev rule for CH340 IMU (/dev/myimu)"
```

---

## Task 7: 接入 sentry_v2.launch.py

**Files:**
- Modify: `src/sentry_bringup/launch/sentry_v2.launch.py`

- [ ] **Step 1: 添加 import**

在文件顶部已有 imports 之后，确认 `IncludeLaunchDescription` 和 `PythonLaunchDescriptionSource` 已导入（已在）。

- [ ] **Step 2: 添加 IMU launch 路径和 IncludeLaunchDescription**

在 `generate_launch_description()` 中，在 `lidar_launch_path` 定义之后、第一个 `Node` 之前，插入：

```python
    imu_launch_path = os.path.join(
        get_package_share_directory('sentry_sensors'), 'launch', 'imu.launch.py'
    )
```

在 `return LaunchDescription([` 列表中，在 LiDAR 的 `IncludeLaunchDescription` 之后、Fusion node 之前，插入：

```python
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(imu_launch_path)
        ),
```

- [ ] **Step 3: Commit**

```bash
git add src/sentry_bringup/launch/sentry_v2.launch.py
git commit -m "feat(sentry_bringup): include imu.launch.py in sentry_v2 launch"
```

---

## Task 8: 代码审查（Code Review）

**Files:**
- Review all modified/created files in `src/sentry_sensors/`

- [ ] **Step 1: 自检清单**

逐条核对：
- [ ] `imu_node.py` 中所有参数都有 `declare_parameter`
- [ ] `_normalize_quaternion_static` 处理了零向量（返回 [1,0,0,0]）
- [ ] `_build_covariance_matrix` 处理了长度不为 9 的情况
- [ ] 磁力计 Y 轴符号翻转保留（`mag.magnetic_field.y = -float(my)`）
- [ ] `main()` 中 `rclpy.shutdown()` 在 `finally` 中且 guarded by `rclpy.ok()`
- [ ] `setup.py` 的 `data_files` 包含 launch/config/udev
- [ ] `imu.launch.py` 中 `remappings` 正确映射了话题名
- [ ] `publish_tf: false` 在 madgwick yaml 中
- [ ] 所有新增文件已添加到 git

- [ ] **Step 2: 运行 Python 静态检查**

```bash
cd src/sentry_sensors && python -m py_compile sentry_sensors/imu_node.py
```

**Expected:** 无输出（无语法错误）

```bash
cd src/sentry_sensors && python -m py_compile launch/imu.launch.py
```

**Expected:** 无输出

- [ ] **Step 3: Commit review 结果（如有修复）**

如果有任何修复，单独 commit：
```bash
git commit -m "review(sentry_sensors): fix issues from code review"
```

---

## 自检

### Spec Coverage

| 设计文档章节 | 对应任务 |
|-------------|---------|
| 文件变更清单 | Task 1-7 |
| imu_node.py 参数化 | Task 3 |
| 协方差矩阵（分轴精细） | Task 4 (imu.yaml) |
| Madgwick publish_tf=false | Task 4 (yaml) + Task 5 (launch) |
| 话题前缀 /sensor/imu/ | Task 3 |
| 100Hz 频率 | Task 3 (参数) + Task 4 (yaml) |
| static_transform_publisher | Task 5 |
| sentry_v2.launch.py 集成 | Task 7 |
| udev 规则 | Task 6 |
| 单元测试（TDD） | Task 2 + Task 3 |
| 5 个潜在问题修复 | Task 3 |

**无遗漏。**

### Placeholder Scan

- 无 TBD/TODO
- 无 "implement later"
- 无 "add appropriate error handling" 等模糊描述
- 所有代码步骤包含完整代码
- 所有命令包含预期输出

### Type Consistency

- `ImuNode` 类名在 Task 2 测试和 Task 3 实现中一致
- `_normalize_quaternion_static` 签名一致
- `_build_covariance_matrix` 签名一致
- 话题名 `/sensor/imu/data_raw` 等在 Task 3、Task 5 中一致

**无不一致。**

---

## 执行选项

Plan complete and saved to `docs/superpowers/plans/2026-06-03-imu-integration.md`.

**Two execution options:**

**1. Subagent-Driven (recommended)** - I dispatch a fresh subagent per task, review between tasks, fast iteration

**2. Inline Execution** - Execute tasks in this session using executing-plans, batch execution with checkpoints

**Which approach?**
