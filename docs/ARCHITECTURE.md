# 系统架构

> Architecture version: v2.9 field cruise and frontend-control baseline  
> 更新日期：2026-07-15

---

## 1. 项目概述

**智农哨兵**是一款面向番茄/小麦/草莓多作物病害巡检的嵌入式比赛原型机，核心能力包括：

- 底盘自动巡航与植株检测触发停车（YOLOv8n 检测 → MobileNetV3 分类 两阶段管线）
- 端侧 AI 病害识别（RDK X5 NPU）
- 环境数据融合决策（移动传感器 + 固定环境节点）
- 农艺建议生成与本地数据记录

**非目标**：工业级防护、续航极限、Web 前端、外部天气接口、端侧大模型（均后置到后续版本）。

---

## 2. 架构总览

采用**三层解耦 + 事件驱动巡检**架构：

| 层级 | 职责 | 关键节点/组件 |
|---|---|---|
| **感知层** | 视觉推理（YOLO 检测 + 分类）、移动/固定环境传感、底盘状态、LiDAR/IMU | `camera_node`, `plant_detector_node`, `vision_diagnosis_node`, `yolo_detection_node`(计划), `uart_bridge_node`, `lora_bridge_node`, `sentry_lidar`, `imu_node` |
| **决策层** | 实时融合、趋势预测、农艺建议 | `fusion_node`, `forecast_node`, `advisory_node` |
| **控制层** | 巡检状态机、底盘/云台控制、数据记录 | `mission_control_node`, `wheel_odom_node`, `web_remote_node`, `servo_driver_node`, `data_logger_node`, Nav2 |

```
┌─────────────────────────────────────────────────────────────┐
│ Perception Layer                                            │
│  ├─ camera_node          → /sentry/camera/image_raw        │
│  ├─ plant_detector_node  → /vision/plant_detected          │
│  ├─ vision_diagnosis_node→ /vision/diagnosis               │
│  ├─ uart_bridge_node     → /sensor/environment_mobile      │
│  │                       → /sensor/soil_nutrition          │
│  │                       → /sentry/chassis/status          │
│  ├─ lora_bridge_node     → /sensor/environment_fixed       │
│  ├─ sentry_lidar         → /scan + /lidar/obstacle_info    │
│  └─ imu_node             → /sensor/imu/data_raw/data       │
└──────────────────────────┬──────────────────────────────────┘
                           │
┌──────────────────────────▼──────────────────────────────────┐
│ Decision Layer                                              │
│  ├─ fusion_node      → /fusion/diagnosis  (事件驱动)       │
│  ├─ forecast_node    → /forecast/alert    (10min)          │
│  └─ advisory_node    → /advisory/action   (事件触发)       │
└──────────────────────────┬──────────────────────────────────┘
                           │
┌──────────────────────────▼──────────────────────────────────┐
│ Control Layer                                               │
│  ├─ mission_control_node → /cmd_vel + /mission/status      │
│  ├─ wheel_odom_node      → /wheel/odom                     │
│  ├─ ekf_filter           → /odom                           │
│  ├─ Nav2                 → /cmd_vel (AUTO)                 │
│  ├─ servo_driver_node    → PWM yaw/pitch                   │
│  ├─ web_remote_node      → /cmd_vel (MANUAL)               │
│  └─ data_logger_node     → ros2 bag                        │
└─────────────────────────────────┬───────────────────────────┘
                                  │
                              STM32F407
```

---

## 3. 模块划分

```
smart-agri-sentry/
├── src/
│   ├── sentry_interfaces/     # ROS2 消息定义
│   ├── sentry_bringup/        # Launch 文件与启动配置
│   ├── sentry_lidar/          # STL19P 激光雷达驱动
│   ├── sentry_vision/         # plant_detector + vision_diagnosis
│   ├── sentry_fusion/         # fusion_node + lwd_calculator
│   ├── sentry_forecast/       # forecast_node
│   ├── sentry_advisory/       # advisory_node + rule_engine
│   ├── sentry_mission/        # mission_control + web_remote + wheel_odom + chassis_cmd + imu_turn
│   ├── sentry_sensors/        # lora_bridge + uart_bridge + imu
│   ├── sentry_servo/          # RDK X5 直接 PWM 云台驱动
│   └── sentry_data_logger/    # rosbag2 录制 + CRITICAL 快照
├── firmware/                  # STM32F407 底盘 + STM32F103 固定环境节点
├── models/                    # *.onnx 源模型（RDK 运行时为 .bin）
├── config/                    # 作物/规则/任务参数
├── docs/                      # 项目文档
└── tests/                     # 单元测试与离线验证
```

详见：
- 节点与话题定义 → [`docs/ROS2.md`](ROS2.md)
- 硬件规格与接线 → [`docs/HARDWARE.md`](HARDWARE.md)
- 环境搭建与编译 → [`docs/SETUP.md`](SETUP.md)

---

## 3.1 Field Cruise Baseline

The current demo baseline uses mapless Nav2 with odom-frame waypoints, RPP path tracking, and mission-owned short-range obstacle avoidance.

Progressive flow:

0. At boot, systemd `sentry-bridge.service` (installed once via `scripts/rdk/install_autostart.sh`) starts the gateway layer — `miniprogram_bridge_node` (:8765), `web_remote_node` (:5000), `weather_node`, `llm_advisor_node`. No SSH is needed after the one-time install; heavy work nodes stay off until requested.
1. `web_remote_node` serves the operator panel at `http://<rdk-ip>:5000/`; the WeChat mini-program talks to `miniprogram_bridge_node` at `http://<rdk-ip>:8765/` (REST + WS).
2. The operator clicks Preheat; the backend runs `scripts/rdk/start_robot_stack.sh` to clean leftovers and start the formal launch.
3. The operator clicks Start Cruise; `/stack/start` switches `/set_auto_mode=true`.
4. `mission_control_node` sends the three cruise waypoints to Nav2.
5. `sentry_lidar` publishes `/lidar/obstacle_info`; when an obstacle is between the robot and the active waypoint and below the stop distance, mission cancels Nav2, publishes zero velocity, backs up, turns aside, drives around, turns back, and rejoins.
6. During the internal avoidance sequence, the normal obstacle trigger is suppressed; only hard safety thresholds can stop the motion.
7. On mission completion, Pause, or E-STOP, `web_remote_node` calls `scripts/rdk/stop_robot_stack.sh` to publish zero velocity and clear ROS leftovers.

Stable baseline summary:

| Item | Baseline |
|---|---|
| Default waypoints | `(2.5,0,90deg) -> (2.5,0.6,180deg) -> (0,0.6,180deg)` |
| Track scale | `left_speed_scale=1.00`, `right_speed_scale=1.00` |
| Goal checker | `xy_goal_tolerance=0.05`, `yaw_goal_tolerance=0.10` |
| Re-trigger suppression | `avoidance_retrigger_suppression_sec=2.5` |
| Stack scripts | `scripts/rdk/start_robot_stack.sh`, `scripts/rdk/stop_robot_stack.sh` |

---

## 4. 数据流

### 4.1 全链路数据流

```
┌─────────────────────────────────────────────────────────────────────┐
│                           STM32F407ZGT6                              │
│  ┌─────────────┐    ┌─────────────┐    ┌─────────────────────────┐  │
│  │ 空气传感器  │    │ 土壤传感器  │    │ 电机PID + 编码器 + 舵机 │  │
│  │ (UART4)     │    │ (UART5)     │    │ (TIM/PWM)               │  │
│  └──────┬──────┘    └──────┬──────┘    └───────────┬─────────────┘  │
│         │                  │                        │                │
│         └──────────────────┴────────────────────────┘                │
│                            │                                         │
│                            ▼                                         │
│                   ┌─────────────────┐                                │
│                   │   TaskSensor    │ 100ms                          │
│                   │   TaskControl   │ 20ms                           │
│                   │   TaskComm      │ 100ms                          │
│                   └────────┬────────┘                                │
│                            │ UART2_TX                                │
└────────────────────────────┼────────────────────────────────────────┘
                             │
                             ▼
┌─────────────────────────────────────────────────────────────────────┐
│                            RDK X5                                    │
│                                                                      │
│  ┌─────────────┐    ┌─────────────┐    ┌─────────────────────────┐  │
│  │ uart_bridge │    │  MIPI Camera│    │  IMU (YB-IMU CH340)     │  │
│  │  (UART2)    │    │  (IMX219)   │    │  → /sensor/imu/data     │  │
│  └──────┬──────┘    └──────┬──────┘    └───────────┬─────────────┘  │
│         │                  │                        │                │
│  ┌──────────────────────────────────────────────────────────────┐   │
│  │     sentry_lidar (STL19P + CP2102, UART 230400)             │   │
│  │           → /scan  +  /lidar/obstacle_info                  │   │
│  └──────────────────────────────────────────────────────────────┘   │
│         │                  │              │          │                │
│         ▼                  ▼              ▼          ▼                │
│  /sensor/           /sentry/camera/    /sensor/     /scan            │
│  environment_mobile image_raw          imu/data                      │
│  /sensor/soil_nutrition                                              │
│                                                                      │
│  ┌──────────────────────────────────────────────────────────────┐   │
│  │  USB转串口 ← LoRa网关 ← [固定环境节点: STM32F103RCT6+CJ702+leaf(RS485)+soil(NPK)+E22-400T30S] │
│  │         lora_bridge_node → /sensor/environment_fixed        │   │
│  └──────────────────────────────────────────────────────────────┘   │
│                                                                      │
│  ┌──────────────────────────────────────────────────────────────┐   │
│  │     wheel_odom_node  (编码器脉冲 → dead reckoning)           │   │
│  │           → /wheel/odom                                       │   │
│  └──────────────────────────────────────────────────────────────┘   │
│                            │                                         │
│                            ▼                                         │
│  ┌──────────────────────────────────────────────────────────────┐   │
│  │     ekf_filter  (robot_localization)                         │   │
│  │     /wheel/odom + /sensor/imu/data → /odom                  │   │
│  └──────────────────────────────────────────────────────────────┘   │
│                            │                                         │
│                            ▼                                         │
│  ┌──────────────────────────────────────────────────────────────┐   │
│  │     Nav2 (nav2_bringup)                                       │   │
│  │     /odom + /scan → /cmd_vel (AUTO模式)                     │   │
│  │     NavfnPlanner + MPPIController (mapless)                 │   │
│  └──────────────────────────────────────────────────────────────┘   │
│         ┌──────────────────────────────────────────┐                 │
│         │         plant_detector_node              │                 │
│         │   (YOLO-Nano, 5fps, 检测植株bbox)        │                 │
│         └──────────────────┬───────────────────────┘                 │
│                            │                                         │
│                            ▼ /vision/plant_detected                  │
│         ┌──────────────────────────────────────────┐                 │
│         │         mission_control_node             │                 │
│         │   (PATROL→APPROACHING→STOPPED→          │                 │
│         │    ANALYZING→ACTION→RESUME→PATROL)      │                 │
│         │   Nav2航点巡航 + 视觉伺服 + 模式切换     │                 │
│         └──────────┬─────────────────────┬───────┘                 │
│                    │                     │                          │
│                    ▼                     ▼                          │
│              /cmd_vel (AUTO)      STOPPED触发拍照                   │
│                                                                      │
│         ┌──────────────────────────────────────────┐                 │
│         │         vision_diagnosis_node            │                 │
│         │   (.bin, 作物-specific, 2fps)            │                 │
│         └──────────────────┬───────────────────────┘                 │
│                            │                                         │
│                            ▼ /vision/diagnosis                       │
│         ┌──────────────────────────────────────────┐                 │
│         │         fusion_node                      │                 │
│         │   (LWD滑动窗口 + 优先级门控 + 证据链)    │                 │
│         └──────────────────┬───────────────────────┘                 │
│                            │                                         │
│                            ▼ /fusion/diagnosis                       │
│         ┌──────────────────────────────────────────┐                 │
│         │    forecast_node / advisory_node         │                 │
│         └──────────────────┬───────────────────────┘                 │
│                            │                                         │
│                            ▼ /forecast/alert                         │
│                            ▼ /advisory/action                        │
│         ┌──────────────────────────────────────────┐                 │
│         │    servo_driver_node / data_logger_node  │                 │
│         └──────────────────────────────────────────┘                 │
└─────────────────────────────────────────────────────────────────────┘
```

### 4.2 事件驱动巡检流程

```
PATROL (Nav2 航点巡航)
   │
   ▼  plant_detector_node 检测到植株
APPROACHING (视觉伺服，bbox 居中)
   │
   ▼  到达停车距离
STOPPED
   │
   ▼  触发拍照
ANALYZING
   │  vision_diagnosis_node → fusion_node → forecast/advisory
   ▼
ACTION (记录、生成建议、必要时喷雾/标记)
   │
   ▼  恢复巡航
RESUME → PATROL
```

---

## 5. 导航架构

### 5.1 当前实现：Mapless Nav2

- **定位/里程计**：`wheel_odom_node` 编码器航位推算 → EKF 融合 IMU → `/odom`
- **全局规划**：`NavfnPlanner`（Dijkstra）
- **局部规划**：`MPPIController`
- **代价地图**：滚动窗口（rolling window），`odom` 帧，无地图服务器
- **航点**：`waypoints.yaml` 中硬编码（odom 坐标系下的弓字形覆盖航点）
- **配置文件**：`src/sentry_mission/config/nav2_no_map.yaml`

### 5.2 目标：LiDAR SLAM 建图导航

为降低长距离巡航的里程计漂移，计划迁移到 LiDAR SLAM/mapping：

- 候选方案：`slam_toolbox` 或 `cartographer`
- 输出：持久化地图、`map` 坐标系、AMCL/重定位
- 影响：需新增地图保存/加载流程，调整 `mission_control_node` 航点管理
- 状态：**已决策，待实现**（详见 [`docs/DECISIONS.md`](DECISIONS.md) ADR-006 与 [`docs/TODO.md`](TODO.md)）

---

## 6. 融合决策逻辑

`fusion_node` 在 `mission_control_node` 进入 `STOPPED` 后被触发，内部持续维护 LWD（叶面湿润时长）24h 滑动窗口。

### 6.1 模式选择

```python
def select_mode(P_vis, env, E_norm, h_risk, t_risk, env_history):
    if P_vis >= 0.80:
        return "VISION_DOMINANT"

    if not env_history.is_cold_boot():
        lwd = env_history.get_lwd_hours()
        if lwd >= LWD_THRESHOLD[crop] and P_vis <= 0.30 and t_risk >= 0.60:
            return "LATENT_SUSPICION"

    hum_threshold = 90 if env_history.is_cold_boot() else 80
    if env.humidity >= hum_threshold and 15 <= env.temperature <= 28 and P_vis >= 0.50:
        return "HIGH_HUMIDITY_PATHOGEN"

    if env.humidity <= 40 and env.temperature >= 30:
        return "DROUGHT_STRESS"

    return "BALANCED"
```

### 6.2 风险计算

```
interaction   = P_vis × E_norm
trend_factor  = 1.0 + 0.2 × max(0, humidity_trend_2h)
Risk = w_v·P_vis + w_e·E_norm·trend_factor + w_i·interaction + bias
Risk = clip(Risk, 0.0, 1.0)

agreement = 1.0 - |P_vis - E_norm|
base_confidence = 0.55 + 0.45 × agreement

if COLD_BOOT:   confidence = base_confidence × 0.75
elif WARM_UP:     confidence = base_confidence × 0.90
else:             confidence = base_confidence
```

### 6.3 报警分级

| 级别 | 条件 |
|---|---|
| `CRITICAL` | Risk ≥ 0.80 且 confidence ≥ 0.80（冷启动最多降级为 WARNING） |
| `WARNING` | Risk ≥ 0.60 |
| `SUSPICION` | mode == LATENT_SUSPICION 且 Risk ≥ 0.40 |
| `NORMAL` | 其余 |

---

## 7. 数据存储策略

| 场景 | 方案 | 位置 |
|---|---|---|
| 实时录制 | `ros2 bag` 选择性录制核心 topic | RDK X5 SD 卡 |
| 循环策略 | 7 天自动覆盖 | RDK X5 SD 卡 |
| CRITICAL 事件 | 前后 5 分钟片段永久保留 | `records/critical/` |
| 离线分析 | `ros2 bag play` → InfluxDB + Grafana | 办公室 PC |

---

## 8. 模型部署

- **源格式**：`models/` 下存放 `tomato_mobilenetv3.onnx`、`wheat_mobilenetv3.onnx`、`strawberry_mobilenetv3.onnx`
- **运行时格式**：RDK X5 NPU 使用从 ONNX 转换的 `.bin` 格式
- **自动适配**：`vision_diagnosis_node` 显式传入 `.tflite` 路径时会被自动重写为 `.bin`
- **输入尺寸**：224×224

病害类别详见 [`docs/ROS2.md`](ROS2.md) §消息接口。

---

## 9. 路线与阶段

| 阶段 | 目标 | 状态 |
|---|---|---|
| Phase 1 | 消息接口 + plant_detector + vision_diagnosis + fusion + mission_control | 已完成 |
| Phase 2 | forecast + advisory + data_logger | 已完成 |
| Phase 3 | 固定环境节点 LoRa 通信 + lora_bridge_node | 已完成（2026-06-28，CJ702 空气传感器端到端验证通过，土壤/叶面待接入） |
| Phase 4 | 外部天气 + Web 前端 + InfluxDB 离线分析 | 后续完善 |
| **新增目标** | LiDAR SLAM/mapping 替代 mapless Nav2 | 已决策，待实现 |

当前任务与阻塞项见 [`docs/TODO.md`](TODO.md)。
