# 智农哨兵 · Smart Agri Sentry v2.6

> 基于 RDK X5 的番茄/小麦/草莓多作物病害巡检机器人，融合端侧视觉推理、环境感知与农艺决策。

[![ROS2 Humble](https://img.shields.io/badge/ROS2-Humble-blue)](https://docs.ros.org/en/humble/)
[![Platform](https://img.shields.io/badge/Platform-RDK%20X5-orange)](https://developer.d-robotics.cc/)
[![License](https://img.shields.io/badge/License-MIT-green.svg)](LICENSE)

---

## 项目概述

智农哨兵是一款面向嵌入式比赛的农作物病害自主巡检原型机：

- **自主巡航**（当前 mapless Nav2，目标 LiDAR SLAM）
- **植株检测触发停车** → 端侧 AI 病害识别（RDK X5 BPU，`pyeasy_dnn` 推理）
- **移动 + 固定环境传感器融合** → 风险评估 → 农艺建议
- **本地 ros2 bag** 数据记录，7 天循环覆盖

### 病害覆盖

| 作物 | 类别数 | 模型架构 | BPU 精度 | 部署状态 |
|------|--------|---------|---------|---------|
| 番茄 | 7 | MobileNetV3-Large | int8 | 已部署 |
| 小麦 | 5 | MobileNetV3-Small | int8 | 已部署 |
| 草莓 | 8 | MobileNetV3-Small | int16 | 已部署 |

**植株检测**：YOLOv8n（Crop/Weed 二分类），int8 BPU，mAP50 = 0.860

---

## 系统架构

```
┌─────────────────────────────────────────────────────────────┐
│ 感知层                                                      │
│  ├─ plant_detector_node   → /vision/plant_detected         │
│  ├─ vision_diagnosis_node → /vision/diagnosis              │
│  ├─ vision_pipeline_node  → 云台多角度扫描编排              │
│  ├─ uart_bridge_node      → /sensor/environment_mobile     │
│  │                         → /sensor/soil_nutrition        │
│  ├─ lora_bridge_node      → /sensor/environment_fixed      │
│  └─ imu_node              → /sensor/imu                    │
└──────────────────────────┬──────────────────────────────────┘
                           │
┌──────────────────────────▼──────────────────────────────────┐
│ 决策层                                                      │
│  ├─ fusion_node     → /fusion/diagnosis  (LWD窗口+门控)    │
│  ├─ forecast_node   → /forecast/alert    (趋势外推)        │
│  └─ advisory_node   → /advisory/action   (YAML规则引擎)    │
└──────────────────────────┬──────────────────────────────────┘
                           │
┌──────────────────────────▼──────────────────────────────────┐
│ 控制层                                                      │
│  ├─ mission_control_node → /mission/status + cmd_vel       │
│  ├─ keyboard_control_node→ 键盘手动控制                     │
│  ├─ web_remote_node      → Flask Web 前端 + rosbridge      │
│  ├─ wheel_odom_node      → 轮式里程计（EKF 输入）           │
│  └─ data_logger_node     → ros2 bag（7天循环+CRITICAL保留）│
└─────────────────────────────────────────────────────────────┘
```

### 核心特性

- **多作物支持**：动态切换番茄/小麦/草莓
- **事件驱动巡检**：植株检测 → 停车 → 云台多角度扫描 → 分类 → 决策 → 恢复巡航
- **24h 叶面湿润时长（LWD）**：固定环境节点 5 分钟采样，288 点滑动窗口，冷启动优雅降级
- **严格优先级门控**：VISION_DOMINANT → LATENT_SUSPICION → HIGH_HUMIDITY_PATHOGEN → DROUGHT_STRESS → BALANCED，带滞回缓冲防抖动
- **结构化农艺建议**：YAML 规则引擎，毫秒级响应，比赛可解释
- **Web 前端**：实时监控面板，支持 mock 模式离线测试

---

## 硬件清单

| 模块 | 型号/方案 | 备注 |
|------|----------|------|
| AI 主控 | RDK X5（8 核 A55, R5 NPU 10 TOPS） | ROS2 Humble，视觉推理 + 决策节点 |
| 运动控制 | STM32F407ZGT6（FreeRTOS） | UART 协议，100Hz 控制 + 1Hz 遥测 |
| 摄像头 | IMX219 MIPI-CSI | 1920×1080，NV12 格式 |
| 激光雷达 | STL19P / LD19 | CP2102 UART，波特率 230400 |
| IMU | YB-IMU（CH340 USB） | /dev/myimu，波特率 115200 |
| 云台 | 2-DOF 舵机 | RDK X5 直接 PWM 控制 |
| 移动传感器 | 七合一空气（CJ702）+ 土壤 NPK（RS485 ModBus）+ 叶面湿度（LWS10） | 通过底盘串口回传 |
| 固定环境节点 | STM32F103RCT6 + SX1262（LoRa） | 太阳能供电，IP65 防水盒 |
| LoRa 网关 | E22-400TBH-SC（ESP32-S3 + SX1262） | USB 串口直连 RDK X5 |
| 固定节点传感器 | SHT30 + SCD40 + RS485 土壤 + LWS10 | 空气/CO₂/土壤/叶面湿度 |

---

## 快速开始

### 环境要求

- RDK X5，Ubuntu 22.04 + ROS2 Humble
- Python 3.10+

### 编译

```bash
cd ~/dev_ws
git clone https://github.com/wjunhere/smart-agri-sentry.git src/smart_agri_sentry
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

### 配置

```bash
# 作物特异性参数（温度窗口、LWD 阈值）
cp config/crop_profiles.yaml.example config/crop_profiles.yaml

# 农艺建议规则库
cp config/advisory_rules.yaml.example config/advisory_rules.yaml

# 巡检参数（巡航速度、检测阈值等）
cp config/mission_params.yaml.example config/mission_params.yaml
```

### 启动

```bash
# 完整系统（mapless Nav2）
ros2 launch sentry_bringup sentry_v2.launch.py crop_type:=tomato

# 键盘手动控制
ros2 run sentry_mission keyboard_control

# 单独调试节点
ros2 run sentry_fusion fusion_node --ros-args -p crop_type:=tomato
```

---

## 项目结构

```
smart_agri_sentry/
├── src/
│   ├── sentry_interfaces/        # ROS2 消息定义（.msg）
│   ├── sentry_bringup/           # Launch 文件、URDF、Web 前端
│   ├── sentry_vision/            # 植株检测 + 病害分类 + 云台扫描管线
│   ├── sentry_fusion/            # 实时融合 + LWD 计算器
│   ├── sentry_forecast/          # 趋势外推 + 预警
│   ├── sentry_advisory/          # YAML 规则引擎（农艺建议）
│   ├── sentry_mission/           # 巡检状态机 + 键盘控制 + 轮式里程计
│   ├── sentry_sensors/           # UART/LoRa/环境桥接 + IMU 驱动
│   ├── sentry_data_logger/       # ros2 bag 录制与保留策略
│   ├── sentry_servo/             # 2-DOF 云台舵机控制
│   └── sentry_lidar/             # LD19/STL19P 激光雷达驱动
├── firmware/
│   ├── chassis/                  # STM32F407 FreeRTOS 底盘固件（GCC 编译）
│   └── stm32_cj702_lora_hal/     # STM32F103 固定环境节点固件
├── models/
│   ├── quantization/             # ONNX → BPU .bin 量化校准配置
│   ├── yolo_quantize/            # YOLOv8n 量化产物
│   ├── tomato_mobilenetv3.onnx
│   ├── wheat_mobilenetv3.onnx
│   ├── strawberry_mobilenetv3.onnx
│   └── yolov8n_crop_weed_bayese_640x640_nv12.bin
├── config/
│   ├── crop_profiles.yaml        # 各作物参数
│   ├── advisory_rules.yaml       # 农艺建议规则
│   ├── mission_params.yaml       # 状态机参数
│   ├── forecast_params.yaml      # 预测算法参数
│   └── data_logger_params.yaml   # 日志保留策略
├── docs/
│   ├── ARCHITECTURE.md           # 系统架构与数据流
│   ├── HARDWARE.md               # 硬件规格、接线、通信协议
│   ├── ROS2.md                   # 节点图、话题/服务/参数、TF 树
│   ├── SETUP.md                  # 环境搭建、编译、烧录、模型部署
│   ├── DECISIONS.md              # 技术决策记录（ADR）
│   ├── TODO.md                   # 当前 Sprint 任务与阻塞项
│   ├── ISSUES.md                 # 已知问题与硬件限制
│   ├── hardware_refs/            # RDK X5、STM32、LoRa 模块数据手册
│   └── sensors/                  # 传感器数据手册与协议文档
├── test/                         # 参考实现与实验
├── report/                       # 比赛设计报告
├── example/                      # ROS2 开发例程
├── fix/                          # Bug 修复笔记与方案
└── videos/                       # 演示视频
```

---

## 节点说明

### 感知层

| 节点 | 订阅 | 发布 | 说明 |
|------|------|------|------|
| `camera_node` | - | `/sentry/camera/image_raw` | MIPI 摄像头驱动（IMX219） |
| `plant_detector_node` | `image_raw` | `/vision/plant_detected` | YOLOv8n BPU 推理，触发停车 |
| `vision_diagnosis_node` | `image_raw` | `/vision/diagnosis` | MobileNetV3 BPU 作物特异性病害分类 |
| `vision_pipeline_node` | `image_raw`, `plant_detected` | `/vision/diagnosis`, 舵机指令 | 云台多角度扫描编排 |
| `uart_bridge_node` | `cmd_vel`, `servo_cmd` | `/sensor/environment_mobile`, `/sensor/soil_nutrition`, `/sentry/chassis/status` | STM32F4 串口桥接 |
| `lora_bridge_node` | LoRa 网关串口 | `/sensor/environment_fixed` | 固定环境节点数据（多点采集，融合时取平均） |
| `imu_node` | - | `/sensor/imu` | YB-IMU 驱动（含 CH340 ARM 读取补丁） |

### 决策层

| 节点 | 订阅 | 发布 | 说明 |
|------|------|------|------|
| `fusion_node` | `/vision/diagnosis`, `/sensor/environment_fixed`, `/sensor/environment_mobile` | `/fusion/diagnosis` | LWD 滑动窗口 + 优先级门控 + 证据链 |
| `forecast_node` | `/fusion/diagnosis` | `/forecast/alert` | 趋势外推（默认），预留 SIR-like 模型 |
| `advisory_node` | `/fusion/diagnosis`, `/forecast/alert` | `/advisory/action` | YAML 规则引擎，事件触发 |

### 控制层

| 节点 | 订阅 | 发布 | 说明 |
|------|------|------|------|
| `mission_control_node` | `/vision/plant_detected`, `/fusion/diagnosis`, `/advisory/action`, `/sentry/chassis/status` | `/sentry/cmd_vel`, `/mission/status` | 停-拍-判-走状态机 |
| `keyboard_control_node` | 键盘输入 | `/sentry/cmd_vel` | 方向键手动控制底盘 |
| `web_remote_node` | HTTP API | `/sentry/cmd_vel` | Flask Web 前端 + rosbridge WebSocket |
| `wheel_odom_node` | 底盘状态 | `/wheel/odometry` | 轮式里程计（供 EKF 使用） |
| `data_logger_node` | 核心 topic | ros2 bag 文件 | 7 天循环，CRITICAL 事件永久保留 |

---

## 核心算法

### LWD 滑动窗口与冷启动

固定环境节点每 5 分钟采样一次，维护 288 点（24 小时）滑动窗口：

| 阶段 | 时长 | LWD 策略 | LATENT_SUSPICION | 置信度 |
|------|------|---------|------------------|--------|
| COLD_BOOT | 0–30 分钟 | 回退瞬时湿度，上限 0.70 | 禁用 | ×0.75 |
| WARM_UP | 30 分钟–24 小时 | 短时 LWD 线性外推 | 条件放宽 | ×0.90 |
| NORMAL | ≥24 小时 | 完整 24h 查表 | 正常触发 | ×1.0 |

作物特异性 LWD 阈值：

| 作物 | 临界 (≥h) | 高危 (≥h) | 中等 (≥h) | h_risk |
|------|----------|----------|----------|--------|
| 番茄 | 6 | 4 | 2 | 0.95 / 0.80 / 0.55 |
| 小麦 | 4 | 3 | 1.5 | 0.95 / 0.80 / 0.55 |
| 草莓 | 8 | 5 | 3 | 0.95 / 0.80 / 0.55 |

### 优先级门控

```
VISION_DOMINANT（P_vis ≥ 0.80，滞回退出阈值 0.75）
       ↓
LATENT_SUSPICION（LWD ≥ 阈值，P_vis ≤ 0.30，冷启动禁用）
       ↓
HIGH_HUMIDITY_PATHOGEN（湿度 ≥ 80–90%，15–28°C，P_vis ≥ 0.50）
       ↓
DROUGHT_STRESS（湿度 ≤ 40%，温度 ≥ 30°C）
       ↓
BALANCED（兜底）
```

### 融合公式

```
interaction  = P_vis × E_norm
trend_factor = 1.0 + 0.2 × max(0, humidity_trend_2h)

Risk = w_v·P_vis + w_e·E_norm·trend_factor + w_i·interaction + bias
Risk = clip(Risk, 0.0, 1.0)

agreement       = 1.0 - |P_vis - E_norm|
base_confidence = 0.55 + 0.45 × agreement
confidence      = base_confidence × (COLD_BOOT: 0.75, WARM_UP: 0.90, NORMAL: 1.0)
```

报警分级：
- **CRITICAL**：Risk ≥ 0.80 且 confidence ≥ 0.80（冷启动最多 WARNING）
- **WARNING**：Risk ≥ 0.60
- **SUSPICION**：mode == LATENT_SUSPICION 且 Risk ≥ 0.40
- **NORMAL**：其余

---

## 数据存储

| 场景 | 方案 | 位置 |
|------|------|------|
| 实时录制 | `ros2 bag` 选择性录制核心 topic | RDK X5 SD 卡 |
| 循环策略 | 7 天自动覆盖 | RDK X5 SD 卡 |
| CRITICAL 事件 | 前后 5 分钟永久保留 | `records/critical/` |
| 离线分析 | `ros2 bag play` → InfluxDB + Grafana | 办公 PC |

---

## 参与贡献

欢迎提交 Issue 和 PR。

## 许可证

MIT License。详见 [LICENSE](LICENSE)。
