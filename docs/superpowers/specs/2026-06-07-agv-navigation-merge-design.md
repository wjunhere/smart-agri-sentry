# AGV 导航代码合并设计 — 从 example/agv_ws 到主项目 src/

**日期:** 2026-06-07
**任务:** 将 example/agv_ws 中的路径规划/导航代码合并到主项目 src/，在独立 git 分支上开发
**状态:** 待实现

---

## 1. 背景与目标

### 1.1 当前状况

主项目 (`e:/smart_agri_sentry/src/`) 已有完整的农业巡检机器人感知-决策链：

- **感知:** Camera + LiDAR + IMU + 环境传感器
- **决策:** mission_control_node (CRUISING → APPROACHING → STOPPED → ANALYZING → ACTION → RESUME)
- **融合:** fusion_node (多源数据融合 + 病害诊断)

但**缺少完整的导航栈**，没有：
- 编码器里程计
- EKF 状态估计
- Nav2 路径规划
- 航点遍历

example/agv_ws 提供了完整的 Nav2 导航示例，包含轮式里程计、EKF 融合、Nav2 配置、航点状态机、Web 遥控。

### 1.2 目标

将 example 中的导航相关代码合并到主项目，与现有感知-决策链无缝集成：

1. 扩展底盘消息协议，增加编码器脉冲
2. 新增轮式里程计节点
3. 集成 EKF 状态估计
4. 集成 Nav2 导航栈（无地图模式）
5. 重构 mission_control_node，融合 Nav2 航点导航
6. 移植 Web 遥控功能

---

## 2. 架构设计

### 2.1 系统架构

```
┌──────────────────────────────────────────────────────────────────────────┐
│                    Sentry V2 + Navigation 架构                            │
├──────────────────────────────────────────────────────────────────────────┤
│  感知层    │ Camera → camera_node → /vision/plant_detected               │
│            │ LiDAR  → sentry_lidar → /scan, /lidar/obstacle_info          │
│            │ IMU    → imu_node → /sensor/imu/data_raw → Madgwick         │
│            │        → /sensor/imu/data                                    │
│            │ STM32  → uart_bridge → /sentry/chassis/status (扩展)         │
│            │        → /sensor/environment_mobile, /sensor/soil_nutrition  │
├──────────────────────────────────────────────────────────────────────────┤
│  里程计层  │ wheel_odom_node:                                             │
│            │   /sentry/chassis/status → left_pulse, right_pulse          │
│            │   → /wheel/odom (Odometry, dead reckoning)                   │
├──────────────────────────────────────────────────────────────────────────┤
│  融合层    │ EKF (robot_localization):                                    │
│            │   /wheel/odom + /sensor/imu/data → /odom (融合)              │
├──────────────────────────────────────────────────────────────────────────┤
│  规划层    │ Nav2:                                                        │
│            │   • NavfnPlanner (全局) + MPPIController (局部)              │
│            │   • Costmap: /scan 输入，odom 帧，无地图模式                   │
├──────────────────────────────────────────────────────────────────────────┤
│  行为层    │ mission_control_node (重构):                                 │
│            │   • PATROL: Nav2 航点巡航                                    │
│            │   • APPROACHING: 视觉伺服 (bbox 居中)                        │
│            │   → STOPPED → ANALYZING → ACTION → RESUME → PATROL          │
│            │   • MANUAL: Web 遥控，Nav2 暂停                             │
├──────────────────────────────────────────────────────────────────────────┤
│  交互层    │ web_remote_node (Flask HTTP) → /cmd_vel (MANUAL 模式)       │
└──────────────────────────────────────────────────────────────────────────┘
```

### 2.2 Topic 映射

| Topic | Publisher | Subscriber | 说明 |
|-------|-----------|------------|------|
| `/sentry/chassis/status` | uart_bridge_node | wheel_odom_node, fusion_node | 底盘状态（扩展后含脉冲） |
| `/wheel/odom` | wheel_odom_node | EKF, Nav2 | 编码器里程计 |
| `/sensor/imu/data` | imu_filter_madgwick | EKF | IMU 姿态数据 |
| `/scan` | sentry_lidar | Nav2 costmap | LiDAR 扫描 |
| `/odom` | EKF | Nav2, TF | 融合后的里程计 |
| `/cmd_vel` | Nav2 / web_remote | uart_bridge | 统一速度指令 |
| `/vision/plant_detected` | plant_detector_node | mission_control | 植株检测结果 |
| `/fusion/diagnosis` | fusion_node | mission_control | 病害诊断结果 |
| `/mission/status` | mission_control | (监控) | 任务状态 |

---

## 3. 关键决策

### 3.1 ChassisStatus 扩展（方案 A）

在现有 `ChassisStatus.msg` 中增加三个字段，而不是新增独立帧类型：

```protobuf
float32 left_speed          # m/s
float32 right_speed         # m/s
float32 battery_voltage     # V
uint8   alarm_bits

# === 新增 ===
uint32  left_pulse          # 左轮编码器累计脉冲
uint32  right_pulse         # 右轮编码器累计脉冲
uint32  encoder_timestamp   # ms，STM32 时间戳
```

**STM32 协议变更：**
- `TYPE_CHASSIS` payload 从 7 字节 → 19 字节
- 追加字段：`left_pulse(uint32)`, `right_pulse(uint32)`, `timestamp(uint32)`
- 帧格式：`[header:2][type:1][length:1][payload:19][crc:2]`

### 3.2 mission_control_node 融合模式

将 Nav2 航点导航直接整合进 `mission_control_node`，替换现有的简单 CRUISING 逻辑：

```
IDLE ──(启动)──> PATROL
  │                    │
  │                    ▼ 检测到植株
  │              APPROACHING (视觉伺服)
  │                    │
  │                    ▼ bbox 居中且足够近
  │              STOPPED ──> ANALYZING
  │                    │
  │                    ▼ 分析完成或超时
  │              ACTION (记录结果)
  │                    │
  │                    ▼
  │              RESUME (暂停) ──> PATROL (恢复航点)
  │
  ▼ (Web 遥控切 MANUAL)
MANUAL (Nav2 取消，等待)
  │
  ▼ (Web 遥控切 AUTO)
PATROL (从保存索引恢复)
```

**Nav2 集成要点：**
- `PATROL` 状态使用 `BasicNavigator.goToPose()` 发送航点
- 检测到植株时：保存当前航点索引 → `navigator.cancelTask()` → 转入 `APPROACHING`
- `RESUME → PATROL`：从保存的索引恢复航点发送
- MANUAL 模式：取消 Nav2 任务，不发布 Twist，由 web_remote 接管

### 3.3 cmd_vel 统一

所有速度指令统一发布到 `/cmd_vel`：
- Nav2 (AUTO 模式)
- mission_control_node (APPROACHING 状态时的视觉伺服)
- web_remote_node (MANUAL 模式)

仲裁：mission_control_node 通过 `/set_auto_mode` 服务管理模式。MANUAL 时 mission_control 不发布任何 Twist。

---

## 4. 文件变更清单

### 4.1 修改现有文件

| 文件 | 变更 |
|------|------|
| `sentry_interfaces/msg/ChassisStatus.msg` | 增加 `left_pulse`, `right_pulse`, `encoder_timestamp` |
| `sentry_sensors/sentry_sensors/uart_bridge_node.py` | 扩展 `decode_chassis_frame` 解析新增 12 字节 |
| `sentry_mission/sentry_mission/mission_control_node.py` | **完全重写**，集成 Nav2 + 航点 + 视觉伺服 + 模式切换 |
| `sentry_mission/sentry_mission/__init__.py` | 无需变更 |
| `sentry_mission/setup.py` | 增加 `wheel_odom_node`, `web_remote_node` entry_points |
| `sentry_mission/package.xml` | 添加 `nav2_simple_commander`, `robot_localization` 依赖 |
| `sentry_bringup/launch/sentry_v2.launch.py` | 新增 Nav2, EKF, wheel_odom, web_remote 启动 |
| `config/mission_params.yaml` | 增加 Nav2/waypoint 相关参数 |

### 4.2 新增文件

| 文件 | 来源 | 说明 |
|------|------|------|
| `sentry_mission/sentry_mission/wheel_odom_node.py` | 改编自 `wheel_odom.py` | 订阅 ChassisStatus，发布 `/wheel/odom` |
| `sentry_mission/sentry_mission/web_remote_node.py` | 改编自 `web_remote.py` | Flask HTTP 遥控 + 模式切换 |
| `sentry_mission/config/ekf.yaml` | 改编自 `ekf.yaml` | topic 适配：/wheel/odom, /sensor/imu/data |
| `sentry_mission/config/nav2_no_map.yaml` | 改编自 `nav2_no_map.yaml` | Nav2 无地图参数 |
| `sentry_mission/config/waypoints.yaml` | 改编自 `waypoints.yaml` | 默认弓字形航点 |
| `sentry_mission/static/index.html` | 新建 | Web 遥控前端页面 |

---

## 5. 配置参数

### 5.1 EKF (`ekf.yaml`)

```yaml
ekf_filter_node:
  ros__parameters:
    frequency: 30.0
    sensor_timeout: 0.1
    two_d_mode: true
    publish_tf: true
    map_frame: map
    odom_frame: odom
    base_link_frame: base_link
    world_frame: odom

    odom0: /wheel/odom
    odom0_config: [true, true, false,   # x, y, z
                   false, false, true,  # roll, pitch, yaw
                   true, false, false,  # vx, vy, vz
                   false, false, true,  # vroll, vpitch, vyaw
                   false, false, false]
    odom0_differential: false
    odom0_queue_size: 10

    imu0: /sensor/imu/data
    imu0_config: [false, false, false,
                  true, true, true,
                  false, false, false,
                  true, true, true,
                  false, false, false]
    imu0_differential: true
    imu0_queue_size: 10
```

### 5.2 Nav2 (`nav2_no_map.yaml`)

- 全局/局部 costmap 使用 `rolling_window: true`（无地图模式）
- Planner: NavfnPlanner (Dijkstra)
- Controller: MPPIController, DiffDrive 模型
- 最大速度: 0.3 m/s, 最大角速度: 0.8 rad/s
- yaw_goal_tolerance: 3.1416（不检查朝向，只关心位置）

### 5.3 Wheel Odometry 参数

| 参数 | 值 | 说明 |
|------|-----|------|
| wheel_base | 0.4 m | 轮距 |
| pulses_per_meter | 1000 | 每米脉冲数 |
| max_pulse_delta | 100 | 最大允许脉冲跳变（防抖动） |
| publish_rate | 20 Hz | 里程计发布频率 |

---

## 6. 错误处理

| 场景 | 处理 |
|------|------|
| STM32 串口未打开 | uart_bridge 持续尝试，wheel_odom 不发消息 |
| Nav2 未激活 | mission_control `waitUntilNav2Active()` 阻塞等待 |
| 脉冲跳变 | wheel_odom 检测跳变超过阈值，跳过该帧 |
| Web 遥控超时 | 500ms 无指令自动归零速度 |
| 分析超时 | ANALYZING 状态 5s 无 fusion 结果，记录超时继续 |
| 航点全部完成 | mission_control 停在 IDLE，等待新指令 |

---

## 7. 测试策略

### 7.1 单元测试

| 测试项 | 方法 |
|--------|------|
| uart_bridge 帧解析 | 模拟二进制帧输入，验证 ChassisStatus 字段 |
| wheel_odom dead reckoning | 输入模拟脉冲序列，验证 pose 计算 |
| mission_control 状态转换 | 模拟 topic 输入，验证状态机跳转 |

### 7.2 集成测试

| 测试项 | 方法 |
|--------|------|
| EKF 融合 | 同时发布 /wheel/odom 和 /sensor/imu/data，验证 /odom 输出 |
| Nav2 航点导航 | 发布单个航点，验证机器人到达 |
| 端到端流程 | 模拟植株检测 → 停车 → 分析 → 恢复航点 |

---

## 8. STM32 固件配合

STM32 需要修改 `TYPE_CHASSIS` 帧的 payload 格式：

```c
// 原格式 (7 bytes)
// struct { int16_t left_speed; int16_t right_speed; uint8_t battery; uint8_t alarm; }

// 新格式 (19 bytes)
struct ChassisPayload {
    int16_t  left_speed;      // mm/s
    int16_t  right_speed;     // mm/s
    uint8_t  battery_voltage; // 0.1V
    uint8_t  alarm_bits;
    uint32_t left_pulse;      // 累计脉冲
    uint32_t right_pulse;     // 累计脉冲
    uint32_t timestamp_ms;    // STM32 运行时间
};
```

**注意：** STM32 固件变更与 ROS 端变更必须同步部署，否则帧解析会错位。

---

## 9. 分支策略

在 git 上创建独立分支 `feature/agv-navigation-merge`，所有变更在该分支上开发：

1. 创建分支
2. 修改 `ChassisStatus.msg` + uart_bridge
3. 新增 wheel_odom_node
4. 新增 EKF 配置 + Nav2 配置
5. 重写 mission_control_node
6. 新增 web_remote_node + 前端
7. 更新 sentry_v2.launch.py
8. 测试验证
9. PR 合并

---

## 10. 依赖

### ROS2 包依赖（新增）

- `nav2_simple_commander` — Nav2 Python API
- `robot_localization` — EKF 状态估计
- `nav2_bringup` — Nav2 启动
- `imu_filter_madgwick` — 已有，IMU 滤波

### Python 依赖（已有）

- `flask` — Web 遥控后端
- `pyserial` — 串口通信（已有）
- `PyYAML` — YAML 配置解析（已有）

---

*设计完成，等待实现计划。*
