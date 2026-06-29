# IMU 驱动集成设计文档

> 日期：2026-06-03
> 范围：将 example/imu_ros2_device 示例代码集成到 src/sentry_sensors，并修复潜在问题
> 状态：设计已确认，待实施

---

## 1. 目标

将 `example/imu_ros2_device/` 中的 YB-IMU 示例驱动，正式集成到 `src/sentry_sensors` 包中，作为导航级 IMU 为 Nav2/SLAM 提供姿态数据。

---

## 2. 硬件信息

| 项目 | 值 |
|------|-----|
| 传感器 | YB-IMU（CH340 USB-转串口） |
| USB ID | `1a86:7523` |
| 连接方式 | USB 直连 RDK X5 |
| 固定串口 | `/dev/myimu`（udev 规则已配置） |
| 波特率 | 由 YbImuLib 内部管理 |

---

## 3. 文件变更清单

### 3.1 新增文件

```
src/sentry_sensors/
├── sentry_sensors/
│   └── imu_node.py                  # 核心驱动节点（重构后）
├── config/
│   ├── imu.yaml                     # IMU 节点参数配置
│   └── imu_filter_madgwick.yaml     # Madgwick 滤波器配置
├── launch/
│   └── imu.launch.py                # 启动 IMU 节点 + Madgwick 滤波
├── udev/
│   └── 99-myimu.rules               # udev 规则（从现有文件复制）
└── tests/
    └── test_imu_node.py             # 单元测试

src/sentry_bringup/launch/
└── sentry_v2.launch.py              # 修改：IncludeLaunchDescription(imu.launch.py)
```

### 3.2 修改文件

| 文件 | 修改内容 |
|------|----------|
| `src/sentry_sensors/setup.py` | 新增 `imu_node` entry_point |
| `src/sentry_sensors/package.xml` | 新增 `sensor_msgs`、`geometry_msgs` 依赖 |
| `src/sentry_bringup/launch/sentry_v2.launch.py` | Include `imu.launch.py` |

---

## 4. 节点设计

### 4.1 `imu_node.py`

基于 `example/imu_ros2_device/ybimu_driver.py` 重构。

#### 参数（通过 `imu.yaml` 声明）

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

#### 输出话题

| Topic | 类型 | 频率 | 说明 |
|-------|------|------|------|
| `/sensor/imu/data_raw` | `sensor_msgs/Imu` | 100Hz | 原始 IMU（加速度 + 角速度 + 四元数） |
| `/sensor/imu/mag` | `sensor_msgs/MagneticField` | 100Hz | 磁力计 |
| `/sensor/imu/baro` | `Float32MultiArray` | 100Hz | [height, temperature, pressure, pressure_contrast] |
| `/sensor/imu/euler` | `Float32MultiArray` | 100Hz | [roll, pitch, yaw] |

#### 修复的问题（对比原示例代码）

| 原问题 | 修复方式 |
|--------|----------|
| 裸 `except: pass` | 改为捕获具体异常并记录日志 |
| 串口遍历探测 | 简化为只尝试 `/dev/myimu`，失败即报错 |
| 无 `rclpy.shutdown()` | `main()` `finally` 块中补充 |
| 四元数未归一化检查 | 增加 `_normalize_quaternion()`，偏离 >0.01 时 log warn |
| `Float32MultiArray` 语义不明 | 保留原类型，topic 加 `/sensor/imu/` 前缀 |
| 10Hz 偏低 | 提升为参数化 100Hz |

---

## 5. Madgwick 滤波器配置

```yaml
imu_filter_madgwick:
  ros__parameters:
    fixed_frame: "odom"
    use_mag: true
    publish_tf: false              # 关闭 TF 发布，由 EKF 统一管理
    world_frame: "enu"
    gain: 0.1
    zeta: 0.0
    stateless: false
```

**关键决策：** `publish_tf: false`
- 原因：后续使用 `robot_localization` EKF 融合 IMU + 编码器，统一发布 `odom` → `base_link` TF
- Madgwick 只输出滤波后话题 `/sensor/imu/data`，不参与 TF 树

---

## 6. Launch 文件设计

`imu.launch.py` 启动三个组件：

```
┌─────────────────┐
│   imu_node      │ ──▶ /sensor/imu/data_raw, /sensor/imu/mag, ...
└─────────────────┘
         │
         ▼
┌─────────────────────────┐
│ imu_filter_madgwick     │ ──▶ /sensor/imu/data
└─────────────────────────┘
         │
         ▼
┌─────────────────────────┐
│ static_transform_pub    │ ──▶ base_link ──▶ imu_link (静态)
└─────────────────────────┘
```

- `imu_node`：发布原始数据
- `imu_filter_madgwick`：融合输出滤波后 IMU（`publish_tf: false`）
- `static_transform_publisher`：发布 `base_link` → `imu_link` 静态 TF（IMU 安装偏移，默认零偏移）

---

## 7. 与 `sentry_v2.launch.py` 集成

在现有 launch 中 LiDAR 节点之后、Fusion 节点之前加入：

```python
imu_launch_path = os.path.join(
    get_package_share_directory('sentry_sensors'), 'launch', 'imu.launch.py')

IncludeLaunchDescription(
    PythonLaunchDescriptionSource(imu_launch_path)
),
```

---

## 8. 测试策略

### 8.1 单元测试（`tests/test_imu_node.py`）

| 测试用例 | 验证内容 |
|----------|----------|
| `test_parameters_declared` | 节点启动后所有参数正确声明并可读取 |
| `test_quaternion_normalize_unit` | 输入 [1,0,0,0] 返回不变 |
| `test_quaternion_normalize_zero` | 输入 [0,0,0,0] 的处理（避免除零） |
| `test_quaternion_normalize_large` | 输入 [2,0,0,0] 归一化为 [1,0,0,0] |
| `test_covariance_shape` | 协方差矩阵为 3×3 且对称 |

### 8.2 集成测试（RDK X5 板端执行）

| 测试 | 命令 | 预期结果 |
|------|------|----------|
| 节点启动 | `ros2 launch sentry_sensors imu.launch.py` | 无报错，串口打开成功 |
| 数据输出 | `ros2 topic echo /sensor/imu/data_raw` | 字段非零，stamp 递增 |
| TF 树检查 | `ros2 run tf2_tools view_frames` | 无分叉，`imu_link` 挂在 `base_link` 下 |
| 频率验证 | `ros2 topic hz /sensor/imu/data_raw` | ≈100Hz |

---

## 9. 依赖

### 9.1 运行时依赖

```bash
# ROS2 包
sudo apt install ros-${ROS_DISTRO}-imu-filter-madgwick

# Python 库（RDK X5 上需安装）
# YbImuLib —— 由用户自行提供/安装
```

### 9.2 package.xml 新增依赖

```xml
<depend>sensor_msgs</depend>
<depend>geometry_msgs</depend>
```

---

## 10. 风险与备注

| 风险 | 说明 | 缓解 |
|------|------|------|
| `YbImuLib` 在 RDK X5 上的可用性 | 该库为外部依赖，RDK 上可能未安装 | 用户需确认库已安装或提供安装方式 |
| EKF 尚未部署 | Madgwick `publish_tf: false`，当前无 TF 发布者 | 临时方案：launch 中 static_transform_publisher 保证 TF 树完整；EKF 部署后移除 |
| 板端测试受限 | 当前在 PC 端开发，无法实际串口测试 | 先写代码 + 单元测试，上板后执行集成测试 |
| 协方差值需标定 | 当前为理论估算 + 安全裕量 | 上板后根据实际数据微调 |

---

## 11. 实施顺序

1. 修改 `package.xml`、`setup.py`
2. 创建 `imu_node.py`（TDD：先写测试再写实现）
3. 创建 `imu.yaml`、`imu_filter_madgwick.yaml`
4. 创建 `imu.launch.py`
5. 创建 `99-myimu.rules`
6. 修改 `sentry_v2.launch.py`
7. 写单元测试
8. 代码审查（Code Review）
9. 上板集成测试
