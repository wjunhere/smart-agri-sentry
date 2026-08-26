# 智农哨兵 · Smart Agri Sentry v3.3

> 基于 RDK X5 的番茄/小麦/草莓多作物病害巡检机器人，融合端侧视觉推理、环境感知与农艺决策。

[![ROS2 Humble](https://img.shields.io/badge/ROS2-Humble-blue)](https://docs.ros.org/en/humble/)
[![Platform](https://img.shields.io/badge/Platform-RDK%20X5-orange)](https://developer.d-robotics.cc/)
[![License](https://img.shields.io/badge/License-MIT-green.svg)](LICENSE)

---

## 项目概述

智农哨兵是一款面向嵌入式比赛的农作物病害自主巡检原型机：

- **自主巡航**（当前 mapless Nav2，目标 LiDAR SLAM）
- **植株检测触发停车** → 端侧 AI 病害识别（RDK X5 BPU，`pyeasy_dnn` 推理）
- **植株检测**：单类 `yolo11s` 于 BPU 推理，conf 0.35 + 3 帧 / 2 票时序投票
- **病害诊断**：YOLO 框裁剪（外扩 20%）+ letterbox 224 输入，按作物 MobileNetV3 分类
- **LoRa 上行链路**：固定环境节点 → `/sensor/environment_fixed`（12 字段，opt_v2 协议）
- **移动 + 固定环境传感器融合** → 风险评估 → 农艺建议
- **Web 面板 + 微信小程序控制**，通过统一网关层（`sentry-bridge.service`）
- **本地 ros2 bag** 数据记录，7 天循环覆盖

### 病害覆盖

| 作物 | 类别数 | 模型架构 | BPU 精度 | 输入 | 准确率 | 部署状态 |
|------|--------|---------|---------|------|--------|---------|
| 番茄 | 7 | MobileNetV3-Large **v5（板端域微调）** | int8 | NV12 224×224（**YOLO 裁剪 + letterbox**） | 92.0%（数字基准） | 已部署 |
| 小麦 | 5 | MobileNetV3-Small | int8 | NV12 224×224 | — | 已部署 |
| 草莓 | 8 | MobileNetV3-Small | int16 | RGB 224×224 | — | 已部署 |

**植株检测**：`yolo11s` 单类 "plant"，int8 BPU，mAP50 = 0.970（mAP50-95 = 0.645），conf 0.35 + 时序投票。取代旧 YOLOv8n Crop/Weed 二分类模型（mAP50 0.860，文件保留可回滚）。

---

## 系统架构

```
┌─────────────────────────────────────────────────────────────┐
│ 感知层                                                      │
│  ├─ mipi_camera_node     → /sentry/camera/image_raw         │
│  │                        (IMX477 MIPI-CSI，去畸变+翻转)    │
│  ├─ plant_detector_node  → /vision/plant_detected  (yolo11s)│
│  ├─ vision_diagnosis_node→ /vision/diagnosis (裁剪+letter)  │
│  ├─ vision_pipeline_node → 云台多角度扫描编排                │
│  ├─ uart_bridge_node     → /sentry/chassis/status           │
│  ├─ lora_bridge_node     → /sensor/environment_fixed        │
│  └─ imu_node             → /sensor/imu                      │
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
│ 控制与网关层                                                │
│  ├─ mission_control_node → /mission/status + /sentry/cmd_vel│
│  ├─ keyboard_control_node→ 键盘手动控制                     │
│  ├─ web_remote_node      → Flask Web 面板  :5000            │
│  ├─ miniprogram_bridge_node → REST + WS   :8765             │
│  ├─ weather_node         → 外部天气                         │
│  ├─ llm_advisor_node     → 农艺大模型分析                   │
│  ├─ wheel_odom_node      → 轮式里程计（EKF 输入）           │
│  └─ data_logger_node     → ros2 bag（7天循环+critical保留） │
└─────────────────────────────────────────────────────────────┘
```

### 核心特性

- **多作物支持**：动态切换番茄/小麦/草莓
- **事件驱动巡检**：植株检测 → 停车 → 云台多角度扫描 → 分类 → 决策 → 恢复巡航
- **前端网关自启**：`sentry-bridge.service` 启动控制面（bridge :8765、web :5000、天气、LLM）；相机与推理由顶栏按钮切换（先杀后启），不随开机自启
- **任务栈巡航可靠性**：停止/巡航结束舵机回中、巡航启动恢复检测器、已扫描植株避障抑制、视觉节点 `respawn` 自愈
- **24h 叶面湿润时长（LWD）**：固定环境节点每 60s 回传一帧，1440 点滑动窗口，冷启动优雅降级
- **严格优先级门控**：VISION_DOMINANT → LATENT_SUSPICION → HIGH_HUMIDITY_PATHOGEN → DROUGHT_STRESS → BALANCED，带滞回缓冲防抖动
- **结构化农艺建议**：YAML 规则引擎，毫秒级响应，比赛可解释
- **Web 前端**：实时监控面板，支持 mock 模式离线测试

---

## 硬件清单

| 模块 | 型号/方案 | 备注 |
|------|----------|------|
| AI 主控 | RDK X5（8 核 A55, R5 NPU 10 TOPS） | ROS2 Humble，视觉推理 + 决策节点 |
| 运动控制 | STM32F407ZGT6（FreeRTOS） | UART 协议，编码器闭环，100Hz 控制 |
| 摄像头 | **IMX477 MIPI-CSI**（现役） | 640×480，棋盘格去畸变标定 `config/imx477_640x480.yaml`，`flip_code=-1`（180°） |
| 摄像头（备用） | 海康 MV-CS016-10UC (USB3) | 软件自动曝光（硬件 AE 失效），MIPI 不可用时使用 |
| 激光雷达 | STL19P / LD19 | CP2102 UART，波特率 230400，udev → `/dev/wheeltec_lidar` |
| IMU | YB-IMU（CH340 USB） | udev → `/dev/myimu`（hub 1-1.1），波特率 115200 |
| 云台 | 2-DOF 舵机 | RDK X5 直接 PWM，回中 yaw=67.5° / pitch=45° |
| 固定环境节点 | STM32F103RCT6 + SX1262（LoRa） | CJ702 空气 + 叶面湿度（RS485）+ 土壤 NPK（TTL ModBus） |
| LoRa 网关 | E22-400TBH-SC | USB 串口直连 RDK X5，udev → `/dev/lora`（hub 1-1.4），9600，opt_v2 协议 |

> **GPS 已移除**，不再使用。USB 串口设备按物理口绑定 udev（上游口位，而非 CH340 芯片级匹配），避免同类芯片误匹配。

---

## 快速开始

### 环境要求

- RDK X5，Ubuntu 22.04 + ROS2 Humble
- Python 3.10+

### 编译（板端）

```bash
cd ~/dev_ws
git clone git@github.com:wjunhere/smart-agri-sentry.git src/smart_agri_sentry
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

### 配置

各作物参数、农艺规则、巡检参数与 IMX477 标定文件已直接入库在 `config/` 下，可直接编辑：

| 文件 | 内容 |
|------|------|
| `config/crop_profiles.yaml` | 作物特异性阈值（温度窗口、LWD） |
| `config/advisory_rules.yaml` | 农艺建议规则库 |
| `config/mission_params.yaml` | 状态机参数（巡航速度、检测阈值） |
| `config/forecast_params.yaml` | 预测算法参数 |
| `config/data_logger_params.yaml` | 日志保留策略 |
| `config/imx477_640x480.yaml` | IMX477 去畸变标定 |

### 启动

板端完全由前端控制，一次性安装后无需 SSH：

```bash
# 一次性安装自启网关（systemd sentry-bridge.service）
bash scripts/rdk/install_autostart.sh

# 完整系统（mapless Nav2）
ros2 launch sentry_bringup sentry_v2.launch.py crop_type:=tomato

# 或使用任务栈脚本（由前端按钮调用）：
bash scripts/rdk/start_robot_stack.sh
bash scripts/rdk/stop_robot_stack.sh
```

- Web 面板：`http://<板端IP>:5000/`
- 微信小程序：`http://<板端IP>:8765/`（REST + WS）
- 相机 / 推理由 Web 顶栏按钮切换（`/vision/*`、`/inference/*`），每次按压先杀后启（`start_camera_stack.sh` / `start_inference_stack.sh`）

### 板端连接

| 通道 | 命令 |
|------|------|
| 热点 | `ssh rdk1`（sunrise@10.66.175.213） |
| Type-C RNDIS | `ssh sunrise@192.168.128.10` |

> **GitHub 推送走 SSH**（`git push git@github.com:wjunhere/smart-agri-sentry.git main`）—— HTTPS 代理/直连均不稳。

---

## 项目结构

```
smart_agri_sentry/
├── src/
│   ├── sentry_interfaces/        # ROS2 消息定义（.msg）
│   ├── sentry_bringup/           # Launch 文件、URDF、mipi/hikrobot 相机、Web 前端
│   ├── sentry_vision/            # yolo11s 植株检测 + MobileNetV3 分类 + 管线
│   ├── sentry_fusion/            # 实时融合 + LWD 计算器
│   ├── sentry_forecast/          # 趋势外推 + 预警
│   ├── sentry_advisory/          # YAML 规则引擎（农艺建议）
│   ├── sentry_mission/           # 巡检状态机 + web_remote + wheel_odom + chassis_cmd + imu_turn + keyboard
│   ├── sentry_sensors/           # UART/LoRa/环境桥接 + IMU 驱动
│   ├── sentry_servo/             # 2-DOF 云台舵机控制（直接 PWM）
│   ├── sentry_lidar/             # LD19/STL19P 激光雷达驱动
│   ├── sentry_data_logger/       # ros2 bag 录制与保留策略
│   ├── sentry_miniprogram/       # miniprogram_bridge_node（REST + WS 网关）
│   ├── sentry_weather/           # 外部天气节点
│   └── sentry_llm/               # llm_advisor_node（农艺大模型分析）
├── firmware/
│   ├── chassis/                  # STM32F407 FreeRTOS 底盘固件（GCC 编译）
│   └── stm32_cj702_lora_hal/     # STM32F103 固定环境节点固件
├── models/
│   ├── quantization/             # ONNX → BPU .bin 量化校准配置
│   ├── yolo_quantize/            # yolo11s 量化产物（output_r3）
│   ├── tomato_mobilenetv3_v5.onnx
│   ├── wheat_mobilenetv3.onnx
│   ├── strawberry_mobilenetv3.onnx
│   └── yolov8n_crop_weed_bayese_640x640_nv12.bin   # 旧版（回滚用）
├── config/
│   ├── crop_profiles.yaml        # 各作物参数
│   ├── advisory_rules.yaml       # 农艺建议规则
│   ├── mission_params.yaml       # 状态机参数
│   ├── forecast_params.yaml      # 预测算法参数
│   ├── data_logger_params.yaml   # 日志保留策略
│   └── imx477_640x480.yaml       # IMX477 去畸变标定
├── scripts/
│   └── rdk/                      # start/stop_robot_stack、camera/inference 栈、install_autostart
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
└── videos/                       # 演示视频
```

---

## 节点说明

### 感知层

| 节点 | 订阅 | 发布 | 说明 |
|------|------|------|------|
| `mipi_camera_node` | - | `/sentry/camera/image_raw` | IMX477 MIPI 驱动（去畸变、翻转、软件调参） |
| `hikrobot_camera_node` | - | `/sentry/camera/image_raw` | 海康 MV-CS016-10UC 备用相机（软件 AE） |
| `plant_detector_node` | `image_raw` | `/vision/plant_detected` | yolo11s BPU 推理，conf 0.35 + 时序投票，触发停车 |
| `vision_diagnosis_node` | `image_raw`, `plant_detected` | `/vision/diagnosis` | MobileNetV3 BPU 作物特异性病害分类（YOLO 裁剪 + letterbox） |
| `vision_pipeline_node` | `image_raw`, `plant_detected` | `/vision/diagnosis`, 舵机指令 | 云台多角度扫描编排 |
| `uart_bridge_node` | `/sentry/cmd_vel`, 舵机指令 | `/sentry/chassis/status` | STM32F4 串口桥接 |
| `lora_bridge_node` | LoRa 网关串口 | `/sensor/environment_fixed` | 固定环境节点数据（opt_v2 协议，12 字段） |
| `imu_node` | - | `/sensor/imu` | YB-IMU 驱动（含 CH340 ARM 读取补丁） |

### 决策层

| 节点 | 订阅 | 发布 | 说明 |
|------|------|------|------|
| `fusion_node` | `/vision/diagnosis`, `/sensor/environment_fixed` | `/fusion/diagnosis` | LWD 滑动窗口 + 优先级门控 + 证据链 |
| `forecast_node` | `/fusion/diagnosis` | `/forecast/alert` | 趋势外推（默认），预留 SIR-like 模型 |
| `advisory_node` | `/fusion/diagnosis`, `/forecast/alert` | `/advisory/action` | YAML 规则引擎，事件触发 |

### 控制与网关层

| 节点 | 订阅 | 发布 | 说明 |
|------|------|------|------|
| `mission_control_node` | `/vision/plant_detected`, `/fusion/diagnosis`, `/advisory/action`, `/sentry/chassis/status` | `/sentry/cmd_vel`, `/mission/status` | 停-拍-判-走状态机 |
| `keyboard_control_node` | 键盘输入 | `/sentry/cmd_vel` | 方向键手动控制底盘 |
| `web_remote_node` | HTTP API | `/sentry/cmd_vel` | Flask Web 面板（:5000）+ rosbridge WebSocket |
| `miniprogram_bridge_node` | REST + WS | `/sentry/cmd_vel`, WS 流 | 微信小程序网关（:8765），`/stack/*` 编排 |
| `weather_node` | 外部 API | `/api/weather` | 天气数据（mock + 真实，60s 重发） |
| `llm_advisor_node` | `/api/weather`, 上下文 | 分析 | 农艺大模型分析（DeepSeek） |
| `wheel_odom_node` | `/sentry/chassis/status` | `/wheel/odom` | 轮式里程计（供 EKF 使用） |
| `data_logger_node` | 核心 topic | ros2 bag 文件 | 7 天循环，CRITICAL 事件永久保留 |

> 工具：`chassis_cmd`（编码器闭环运动测试）、`imu_turn`（IMU 陀螺仪闭环原地转弯，精度 ~4%）。

---

## 核心算法

### LWD 滑动窗口与冷启动

固定环境节点每 60s 回传一帧，维护 1440 点（24 小时）滑动窗口：

| 阶段 | 时长 | LWD 策略 | LATENT_SUSPICION | 置信度 |
|------|------|---------|------------------|--------|
| COLD_BOOT | 首帧到达前 | 回退瞬时湿度，上限 0.70 | 禁用 | ×0.75 |
| WARM_UP | <12 点（约 12 分钟） | 短时 LWD 线性外推 | 条件放宽 | ×0.90 |
| NORMAL | ≥12 点（窗口在 24h 内逐渐填满） | 完整 24h 查表 | 正常触发 | ×1.0 |

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

### 诊断输入预处理

取自 `plant_detector_node` 的边界框 **外扩 20%** 裁剪 + letterbox 224（共享 `diagnosis_utils.crop_letterbox`）——与番茄 v5 板端域微调的训练分布一致。无框时回退整帧。量化校准集使用同一批板端翻拍图。

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
