# Smart Agri Sentry v3.3

> Multi-crop (tomato / wheat / strawberry) disease inspection robot powered by RDK X5, fusing onboard vision, environmental sensing, and agronomic decision-making.

[![ROS2 Humble](https://img.shields.io/badge/ROS2-Humble-blue)](https://docs.ros.org/en/humble/)
[![Platform](https://img.shields.io/badge/Platform-RDK%20X5-orange)](https://developer.d-robotics.cc/)
[![License](https://img.shields.io/badge/License-MIT-green.svg)](LICENSE)

---

## Overview

Smart Agri Sentry is an embedded competition prototype for autonomous crop disease inspection:

- **Autonomous cruise** (mapless Nav2, migrating to LiDAR SLAM)
- **Plant detection triggers auto-stop** → on-device AI disease classification (RDK X5 BPU via `pyeasy_dnn`)
- **Plant detection**: single-class `yolo11s` on the BPU, conf 0.35 + 3-frame / 2-vote temporal voting
- **Diagnosis**: YOLO bounding-box crop (20% margin) + letterbox 224 input, per-crop MobileNetV3 classifier
- **LoRa uplink** from fixed environmental nodes → `/sensor/environment_fixed` (12 fields, opt_v2 protocol)
- **Mobile + fixed environmental sensor fusion** → risk assessment → agronomic recommendations
- **Web panel + WeChat mini-program control** through a unified gateway layer (`sentry-bridge.service`)
- **Local ros2 bag** data recording with 7-day rotation

### Crop & Disease Coverage

| Crop | Classes | Model Architecture | BPU Precision | Input | Accuracy | Status |
|------|---------|-------------------|---------------|-------|----------|--------|
| Tomato | 7 | MobileNetV3-Large **v5 (board domain fine-tune)** | int8 | NV12 224×224 (**YOLO crop + letterbox**) | 92.0% (digital benchmark) | Deployed |
| Wheat | 5 | MobileNetV3-Small | int8 | NV12 224×224 | — | Deployed |
| Strawberry | 8 | MobileNetV3-Small | int16 | RGB 224×224 | — | Deployed |

**Plant detection**: `yolo11s` single-class "plant", int8 BPU, mAP50 = 0.970 (mAP50-95 = 0.645), conf 0.35 + temporal voting. Replaces the older YOLOv8n Crop/Weed 2-class model (mAP50 0.860, retained for rollback).

---

## System Architecture

```
┌─────────────────────────────────────────────────────────────┐
│ Perception Layer                                            │
│  ├─ mipi_camera_node    → /sentry/camera/image_raw         │
│  │                        (IMX477 MIPI-CSI, calibr + flip) │
│  ├─ plant_detector_node → /vision/plant_detected  (yolo11s)│
│  ├─ vision_diagnosis_node → /vision/diagnosis (crop+letter)│
│  ├─ vision_pipeline_node  → gimbal multi-angle scan        │
│  ├─ uart_bridge_node    → /sentry/chassis/status           │
│  ├─ lora_bridge_node    → /sensor/environment_fixed        │
│  └─ imu_node            → /sensor/imu                      │
└──────────────────────────┬──────────────────────────────────┘
                           │
┌──────────────────────────▼──────────────────────────────────┐
│ Decision Layer                                              │
│  ├─ fusion_node     → /fusion/diagnosis  (LWD window + gate)│
│  ├─ forecast_node   → /forecast/alert    (trend extrap.)   │
│  └─ advisory_node   → /advisory/action   (YAML rule engine)│
└──────────────────────────┬──────────────────────────────────┘
                           │
┌──────────────────────────▼──────────────────────────────────┐
│ Control & Gateway Layer                                    │
│  ├─ mission_control_node → /mission/status + /sentry/cmd_vel│
│  ├─ keyboard_control_node→ manual keyboard driving          │
│  ├─ web_remote_node      → Flasks web panel  :5000          │
│  ├─ miniprogram_bridge_node → REST + WS     :8765           │
│  ├─ weather_node         → external weather                 │
│  ├─ llm_advisor_node     → agronomic LLM analysis           │
│  ├─ wheel_odom_node      → wheel odometry (EKF input)       │
│  └─ data_logger_node     → ros2 bag (7d rotation + critical)│
└─────────────────────────────────────────────────────────────┘
```

### Key Features

- **Multi-crop support**: dynamic crop switching (tomato / wheat / strawberry)
- **Event-driven inspection**: plant detection → stop → multi-angle gimbal scan → classify → decide → resume
- **Frontend gateway autostart**: `sentry-bridge.service` boots the control plane (bridge :8765, web :5000, weather, LLM); camera & inference are toggled from the top bar (kill-then-start), not autostarted
- **Mission cruise reliability**: servo home restore on stop, detector auto-resume on cruise start, already-scanned plant avoidance suppression, vision node `respawn` self-healing
- **24h Leaf Wetness Duration (LWD)**: fixed env node sends one frame per 60s, 1440-point sliding window, cold-boot graceful degradation
- **Priority gating**: VISION_DOMINANT → LATENT_SUSPICION → HIGH_HUMIDITY_PATHOGEN → DROUGHT_STRESS → BALANCED, with hysteresis to prevent mode flutter
- **Structured agronomic advice**: YAML rule engine, millisecond response, competition-ready explainability
- **Snake-frontend**, mock mode for offline testing

---

## Hardware

| Module | Model / Solution | Notes |
|--------|-----------------|-------|
| AI Main Controller | RDK X5 (8×A55, R5 NPU 10 TOPS) | ROS2 Humble, visual inference + decision nodes |
| Motion Controller | STM32F407ZGT6 (FreeRTOS) | UART protocol, encoder closed-loop, 100 Hz control |
| Camera | **IMX477 MIPI-CSI** (operative) | 640×480, chessboard undistort calibration `config/imx477_640x480.yaml`, `flip_code=-1` (180°) |
| Camera (backup) | 海康 MV-CS016-10UC (USB3) | Software auto-exposure (HW AE broken), used when MIPI unavailable |
| LiDAR | STL19P / LD19 | CP2102 UART 230400 baud, udev → `/dev/wheeltec_lidar` |
| IMU | YB-IMU (CH340 USB) | udev → `/dev/myimu` (hub 1-1.1), 115200 baud |
| Gimbal | 2-DOF servo | RDK X5 direct PWM, home yaw=67.5° / pitch=45° |
| Fixed Env Node | STM32F103RCT6 + SX1262 (LoRa) | CJ702 air + leaf wetness (RS485) + soil NPK (TTL ModBus) |
| LoRa Gateway | E22-400TBH-SC | USB serial to RDK X5, udev → `/dev/lora` (hub 1-1.4), 9600 baud, opt_v2 protocol |

> **GPS removed** — no longer used. USB serial devices are bound by physical udev rules (upstream port), not CH340 chip-level matching, to avoid cross-device conflicts.

---

## Quick Start

### Prerequisites

- RDK X5 running Ubuntu 22.04 with ROS2 Humble
- Python 3.10+

### Build (on the board)

```bash
cd ~/dev_ws
git clone git@github.com:wjunhere/smart-agri-sentry.git src/smart_agri_sentry
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

### Configuration

Per-crop parameters, advisory rules, mission parameters, and IMX477 calibration are checked in under `config/` and can be edited directly:

| File | Contents |
|------|----------|
| `config/crop_profiles.yaml` | Crop-specific thresholds (temperature windows, LWD) |
| `config/advisory_rules.yaml` | Agronomic advisory rule base |
| `config/mission_params.yaml` | State machine parameters (cruise speed, detection thresholds) |
| `config/forecast_params.yaml` | Forecast algorithm parameters |
| `config/data_logger_params.yaml` | Logging retention policy |
| `config/imx477_640x480.yaml` | IMX477 undistortion calibration |

### Launch

The board is controlled entirely from the frontend; no SSH is needed after a one-time install:

```bash
# One-time: install the autostart gateway (systemd sentry-bridge.service)
bash scripts/rdk/install_autostart.sh

# Full system (mapless Nav2)
ros2 launch sentry_bringup sentry_v2.launch.py crop_type:=tomato

# Or drive the stack from the board/scripts (used by the frontend buttons):
bash scripts/rdk/start_robot_stack.sh
bash scripts/rdk/stop_robot_stack.sh
```

- Web panel: `http://<board-ip>:5000/`
- WeChat mini-program: `http://<board-ip>:8765/` (REST + WS)
- Camera / inference are toggled from the web top-bar buttons (`/vision/*`, `/inference/*`) — each press kill-then-starts the stacks (`start_camera_stack.sh` / `start_inference_stack.sh`)

### RDK Board Access

| Channel | Command |
|---------|---------|
| Hotspot | `ssh rdk1` (sunrise@10.66.175.213) |
| Type-C RNDIS | `ssh sunrise@192.168.128.10` |

> **GitHub push uses SSH** (`git push git@github.com:wjunhere/smart-agri-sentry.git main`) — HTTPS proxy / direct are unstable.

---

## Repository Structure

```
smart_agri_sentry/
├── src/
│   ├── sentry_interfaces/        # ROS2 message definitions (.msg)
│   ├── sentry_bringup/           # Launch files, URDF, mipi/hikrobot camera, web frontend
│   ├── sentry_vision/            # yolo11s plant detection + MobileNetV3 diagnosis + pipeline
│   ├── sentry_fusion/            # Real-time fusion + LWD calculator
│   ├── sentry_forecast/          # Trend extrapolation + alerting
│   ├── sentry_advisory/          # YAML rule engine for agronomic advice
│   ├── sentry_mission/           # Mission state machine + web_remote + wheel_odom + chassis_cmd + imu_turn + keyboard
│   ├── sentry_sensors/           # UART/LoRa/env bridges + IMU driver
│   ├── sentry_servo/             # 2-DOF gimbal servo control (direct PWM)
│   ├── sentry_lidar/             # LD19/STL19P LiDAR driver
│   ├── sentry_data_logger/       # ros2 bag recording with retention policy
│   ├── sentry_miniprogram/       # miniprogram_bridge_node (REST + WS gateway)
│   ├── sentry_weather/           # external weather node
│   └── sentry_llm/               # llm_advisor_node (agronomic LLM analysis)
├── firmware/
│   ├── chassis/                  # STM32F407 FreeRTOS chassis firmware (GCC)
│   └── stm32_cj702_lora_hal/     # STM32F103 fixed env node firmware
├── models/
│   ├── quantization/             # ONNX → BPU .bin calibration configs
│   ├── yolo_quantize/            # yolo11s quantization artifacts (output_r3)
│   ├── tomato_mobilenetv3_v5.onnx
│   ├── wheat_mobilenetv3.onnx
│   ├── strawberry_mobilenetv3.onnx
│   └── yolov8n_crop_weed_bayese_640x640_nv12.bin   # legacy (rollback)
├── config/
│   ├── crop_profiles.yaml        # Per-crop parameters
│   ├── advisory_rules.yaml       # Advisory rule base
│   ├── mission_params.yaml       # State machine parameters
│   ├── forecast_params.yaml      # Forecast algorithm parameters
│   ├── data_logger_params.yaml   # Logging retention policy
│   └── imx477_640x480.yaml       # IMX477 undistortion calibration
├── scripts/
│   └── rdk/                      # start/stop_robot_stack, camera/inference stacks, install_autostart
├── docs/
│   ├── ARCHITECTURE.md           # System architecture & data flow
│   ├── HARDWARE.md               # Hardware specs, wiring, protocols
│   ├── ROS2.md                   # Node graph, topics/services, TF tree
│   ├── SETUP.md                  # Environment setup, build, flash, deploy
│   ├── DECISIONS.md              # Architecture Decision Records (ADR)
│   ├── TODO.md                   # Current sprint tasks & blockers
│   ├── ISSUES.md                 # Known issues & hardware limitations
│   ├── hardware_refs/            # RDK X5, STM32, LoRa module datasheets
│   └── sensors/                  # Sensor datasheets & protocol docs
├── test/                         # Reference implementations & experiments
├── report/                       # Competition design report
├── example/                      # ROS2 development examples
└── videos/                       # Demo videos
```

---

## Nodes

### Perception

| Node | Subscribes | Publishes | Description |
|------|-----------|-----------|-------------|
| `mipi_camera_node` | - | `/sentry/camera/image_raw` | IMX477 MIPI driver (undistort, flip, SW sensor tuning) |
| `hikrobot_camera_node` | - | `/sentry/camera/image_raw` | 海康 MV-CS016-10UC backup camera (software AE) |
| `plant_detector_node` | `image_raw` | `/vision/plant_detected` | yolo11s BPU inference, conf 0.35 + temporal voting, triggers stop |
| `vision_diagnosis_node` | `image_raw`, `plant_detected` | `/vision/diagnosis` | MobileNetV3 BPU crop-specific disease classification (YOLO crop + letterbox) |
| `vision_pipeline_node` | `image_raw`, `plant_detected` | `/vision/diagnosis`, servo cmd | Gimbal multi-angle scan orchestration |
| `uart_bridge_node` | `/sentry/cmd_vel`, servo cmd | `/sentry/chassis/status` | STM32F4 UART bridge |
| `lora_bridge_node` | LoRa gateway serial | `/sensor/environment_fixed` | Fixed env node data (opt_v2 protocol, 12 fields) |
| `imu_node` | - | `/sensor/imu` | YB-IMU driver with CH340 ARM read patch |

### Decision

| Node | Subscribes | Publishes | Description |
|------|-----------|-----------|-------------|
| `fusion_node` | `/vision/diagnosis`, `/sensor/environment_fixed` | `/fusion/diagnosis` | LWD sliding window + priority gating + evidence chain |
| `forecast_node` | `/fusion/diagnosis` | `/forecast/alert` | Trend extrapolation (default), SIR-like model reserved |
| `advisory_node` | `/fusion/diagnosis`, `/forecast/alert` | `/advisory/action` | YAML rule engine, event-triggered |

### Control & Gateway

| Node | Subscribes | Publishes | Description |
|------|-----------|-----------|-------------|
| `mission_control_node` | `/vision/plant_detected`, `/fusion/diagnosis`, `/advisory/action`, `/sentry/chassis/status` | `/sentry/cmd_vel`, `/mission/status` | Stop-photograph-classify-go state machine |
| `keyboard_control_node` | keyboard stdin | `/sentry/cmd_vel` | Manual driving via arrow keys |
| `web_remote_node` | HTTP API | `/sentry/cmd_vel` | Flask web panel (:5000) + rosbridge WebSocket |
| `miniprogram_bridge_node` | REST + WS | `/sentry/cmd_vel`, WS stream | WeChat mini-program gateway (:8765), `/stack/*` orchestration |
| `weather_node` | external API | `/api/weather` | Weather data (mock + real, 60s republish) |
| `llm_advisor_node` | `/api/weather`, context | analysis | Agronomic LLM analysis (DeepSeek) |
| `wheel_odom_node` | `/sentry/chassis/status` | `/wheel/odom` | Wheel odometry for EKF |
| `data_logger_node` | core topics | ros2 bag files | 7-day rotation, CRITICAL events permanently retained |

> Tooling: `chassis_cmd` (encoder closed-loop motion test), `imu_turn` (IMU gyro closed-loop in-place turn, ~4% accuracy).

---

## Core Algorithms

### LWD Sliding Window & Cold Boot

Fixed env nodes send one frame per 60s, maintaining a 1440-point (24h) sliding window:

| Phase | Duration | LWD Strategy | LATENT_SUSPICION | Confidence |
|-------|----------|-------------|------------------|------------|
| COLD_BOOT | before first frame | Fallback to instantaneous humidity, cap 0.70 | Disabled | ×0.75 |
| WARM_UP | <12 points (~12 min) | Short-term LWD linear extrapolation | Relaxed conditions | ×0.90 |
| NORMAL | ≥12 points (window fills over 24h) | Full 24h look-up table | Normal trigger | ×1.0 |

Crop-specific LWD thresholds:

| Crop | Critical (≥h) | High (≥h) | Moderate (≥h) | h_risk |
|------|--------------|-----------|---------------|--------|
| Tomato | 6 | 4 | 2 | 0.95 / 0.80 / 0.55 |
| Wheat | 4 | 3 | 1.5 | 0.95 / 0.80 / 0.55 |
| Strawberry | 8 | 5 | 3 | 0.95 / 0.80 / 0.55 |

### Priority Gating

```
VISION_DOMINANT (P_vis ≥ 0.80, hysteresis exit at 0.75)
       ↓
LATENT_SUSPICION (LWD ≥ threshold, P_vis ≤ 0.30, cold-boot disabled)
       ↓
HIGH_HUMIDITY_PATHOGEN (RH ≥ 80–90%, 15–28°C, P_vis ≥ 0.50)
       ↓
DROUGHT_STRESS (RH ≤ 40%, temp ≥ 30°C)
       ↓
BALANCED (fallback)
```

### Fusion Formula

```
interaction  = P_vis × E_norm
trend_factor = 1.0 + 0.2 × max(0, humidity_trend_2h)

Risk = w_v·P_vis + w_e·E_norm·trend_factor + w_i·interaction + bias
Risk = clip(Risk, 0.0, 1.0)

agreement       = 1.0 - |P_vis - E_norm|
base_confidence = 0.55 + 0.45 × agreement
confidence      = base_confidence × (0.75 if COLD_BOOT else 0.90 if WARM_UP else 1.0)
```

Alert levels:
- **CRITICAL**: Risk ≥ 0.80 and confidence ≥ 0.80 (cold-boot downgraded to WARNING max)
- **WARNING**: Risk ≥ 0.60
- **SUSPICION**: mode == LATENT_SUSPICION and Risk ≥ 0.40
- **NORMAL**: otherwise

### Diagnosis Input Preprocessing

Bounding-box crop from `plant_detector_node` with **20% margin** + letterbox 224 (shared `diagnosis_utils.crop_letterbox`) — matches training distribution for the v5 tomato domain fine-tune. Falls back to full-frame when no box is available. Quantization calibration set uses the same board-captured images.

---

## Data Storage

| Scenario | Method | Location |
|----------|--------|----------|
| Real-time recording | `ros2 bag` selective topic recording | RDK X5 SD card |
| Rotation policy | 7-day auto-overwrite | RDK X5 SD card |
| CRITICAL events | ±5 min permanently retained | `records/critical/` |
| Offline analysis | `ros2 bag play` → InfluxDB + Grafana | Office PC |

---

## Contributing

Issues and PRs welcome.

## License

MIT License. See [LICENSE](LICENSE) for details.
