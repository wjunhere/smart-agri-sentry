# Smart Agri Sentry v2.6

> Multi-crop (tomato / wheat / strawberry) disease inspection robot powered by RDK X5, fusing onboard vision, environmental sensing, and agronomic decision-making.

[![ROS2 Humble](https://img.shields.io/badge/ROS2-Humble-blue)](https://docs.ros.org/en/humble/)
[![Platform](https://img.shields.io/badge/Platform-RDK%20X5-orange)](https://developer.d-robotics.cc/)
[![License](https://img.shields.io/badge/License-MIT-green.svg)](LICENSE)

---

## Overview

Smart Agri Sentry is an embedded competition prototype for autonomous crop disease inspection:

- **Autonomous cruise** (mapless Nav2, migrating to LiDAR SLAM)
- **Plant detection triggers auto-stop** → on-device AI disease classification (RDK X5 BPU via `pyeasy_dnn`)
- **Mobile + fixed environmental sensor fusion** → risk assessment → agronomic recommendations
- **Local ros2 bag** data recording with 7-day rotation

### Crop & Disease Coverage

| Crop | Classes | Model Architecture | BPU Precision | Status |
|------|---------|-------------------|---------------|--------|
| Tomato | 7 | MobileNetV3-Large | int8 | Deployed |
| Wheat | 5 | MobileNetV3-Small | int8 | Deployed |
| Strawberry | 8 | MobileNetV3-Small | int16 | Deployed |

**Plant detection**: YOLOv8n (Crop/Weed, 2-class), int8 BPU, mAP50 = 0.860

---

## System Architecture

```
┌─────────────────────────────────────────────────────────────┐
│ Perception Layer                                            │
│  ├─ plant_detector_node   → /vision/plant_detected         │
│  ├─ vision_diagnosis_node → /vision/diagnosis              │
│  ├─ vision_pipeline_node  → gimbal multi-angle scan        │
│  ├─ uart_bridge_node      → /sensor/environment_mobile     │
│  │                         → /sensor/soil_nutrition        │
│  ├─ lora_bridge_node      → /sensor/environment_fixed      │
│  └─ imu_node              → /sensor/imu                    │
└──────────────────────────┬──────────────────────────────────┘
                           │
┌──────────────────────────▼──────────────────────────────────┐
│ Decision Layer                                              │
│  ├─ fusion_node     → /fusion/diagnosis  (LWD window+gate) │
│  ├─ forecast_node   → /forecast/alert    (trend extrap.)   │
│  └─ advisory_node   → /advisory/action   (YAML rule engine)│
└──────────────────────────┬──────────────────────────────────┘
                           │
┌──────────────────────────▼──────────────────────────────────┐
│ Control Layer                                               │
│  ├─ mission_control_node → /mission/status + cmd_vel       │
│  ├─ keyboard_control_node→ manual keyboard driving         │
│  ├─ web_remote_node      → Flask web frontend + rosbridge  │
│  ├─ wheel_odom_node      → wheel odometry (EKF input)      │
│  └─ data_logger_node     → ros2 bag (7d rotation + CRITICAL)│
└─────────────────────────────────────────────────────────────┘
```

### Key Features

- **Multi-crop support**: dynamic crop switching (tomato / wheat / strawberry)
- **Event-driven inspection**: plant detection → stop → multi-angle gimbal scan → classify → decide → resume
- **24h Leaf Wetness Duration (LWD)**: fixed env node samples every 5 min, 288-point sliding window, cold-boot graceful degradation
- **Priority gating**: VISION_DOMINANT → LATENT_SUSPICION → HIGH_HUMIDITY_PATHOGEN → DROUGHT_STRESS → BALANCED, with hysteresis to prevent mode flutter
- **Structured agronomic advice**: YAML rule engine, millisecond response, competition-ready explainability
- **Web frontend**: real-time monitoring dashboard with mock mode for offline testing

---

## Hardware

| Module | Model / Solution | Notes |
|--------|-----------------|-------|
| AI Main Controller | RDK X5 (8×A55, R5 NPU 10 TOPS) | ROS2 Humble, visual inference + decision nodes |
| Motion Controller | STM32F407ZGT6 (FreeRTOS) | UART protocol, 100 Hz control + 1 Hz telemetry |
| Camera | IMX219 MIPI-CSI | 1920×1080, NV12 format |
| LiDAR | STL19P / LD19 | CP2102 UART 230400 baud |
| IMU | YB-IMU (CH340 USB) | /dev/myimu, 115200 baud |
| Gimbal | 2-DOF servo | RDK X5 direct PWM |
| Mobile Sensors | 7-in-1 air (CJ702) + soil NPK (RS485 ModBus) + leaf wetness (LWS10) | Via chassis UART |
| Fixed Env Node | STM32F103RCT6 + SX1262 (LoRa) | Solar-powered, IP65 enclosure |
| LoRa Gateway | E22-400TBH-SC (ESP32-S3 + SX1262) | USB serial to RDK X5 |
| Fixed Node Sensors | SHT30 + SCD40 + RS485 soil + LWS10 | Air / CO2 / soil / leaf wetness |

---

## Quick Start

### Prerequisites

- RDK X5 running Ubuntu 22.04 with ROS2 Humble
- Python 3.10+

### Build

```bash
cd ~/dev_ws
git clone https://github.com/wjunhere/smart-agri-sentry.git src/smart_agri_sentry
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

### Configuration

```bash
# Crop-specific parameters (temperature windows, LWD thresholds)
cp config/crop_profiles.yaml.example config/crop_profiles.yaml

# Agronomic advisory rules
cp config/advisory_rules.yaml.example config/advisory_rules.yaml

# Mission parameters (cruise speed, detection thresholds, etc.)
cp config/mission_params.yaml.example config/mission_params.yaml
```

### Launch

```bash
# Full system (mapless Nav2)
ros2 launch sentry_bringup sentry_v2.launch.py crop_type:=tomato

# Keyboard manual control
ros2 run sentry_mission keyboard_control

# Individual node debugging
ros2 run sentry_fusion fusion_node --ros-args -p crop_type:=tomato
```

---

## Repository Structure

```
smart_agri_sentry/
├── src/
│   ├── sentry_interfaces/        # ROS2 message definitions (.msg)
│   ├── sentry_bringup/           # Launch files, URDF, web frontend
│   ├── sentry_vision/            # Plant detection + disease classification + gimbal pipeline
│   ├── sentry_fusion/            # Real-time fusion + LWD calculator
│   ├── sentry_forecast/          # Trend extrapolation + alerting
│   ├── sentry_advisory/          # YAML rule engine for agronomic advice
│   ├── sentry_mission/           # Mission state machine + keyboard control + wheel odom
│   ├── sentry_sensors/           # UART/LoRa/env bridges + IMU driver
│   ├── sentry_data_logger/       # ros2 bag recording with retention policy
│   ├── sentry_servo/             # 2-DOF gimbal servo control
│   └── sentry_lidar/             # LD19/STL19P LiDAR driver
├── firmware/
│   ├── chassis/                  # STM32F407 FreeRTOS chassis firmware (GCC)
│   └── stm32_cj702_lora_hal/     # STM32F103 fixed env node firmware
├── models/
│   ├── quantization/             # ONNX → BPU .bin calibration configs
│   ├── yolo_quantize/            # YOLOv8n quantization artifacts
│   ├── tomato_mobilenetv3.onnx
│   ├── wheat_mobilenetv3.onnx
│   ├── strawberry_mobilenetv3.onnx
│   └── yolov8n_crop_weed_bayese_640x640_nv12.bin
├── config/
│   ├── crop_profiles.yaml        # Per-crop parameters
│   ├── advisory_rules.yaml       # Advisory rule base
│   ├── mission_params.yaml       # State machine parameters
│   ├── forecast_params.yaml      # Forecast algorithm parameters
│   └── data_logger_params.yaml   # Logging retention policy
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
├── fix/                          # Bugfix notes & plans
└── videos/                       # Demo videos
```

---

## Nodes

### Perception

| Node | Subscribes | Publishes | Description |
|------|-----------|-----------|-------------|
| `camera_node` | - | `/sentry/camera/image_raw` | MIPI camera driver (IMX219) |
| `plant_detector_node` | `image_raw` | `/vision/plant_detected` | YOLOv8n BPU inference, triggers stop |
| `vision_diagnosis_node` | `image_raw` | `/vision/diagnosis` | MobileNetV3 BPU crop-specific disease classification |
| `vision_pipeline_node` | `image_raw`, `plant_detected` | `/vision/diagnosis`, servo cmd | Gimbal multi-angle scan orchestration |
| `uart_bridge_node` | `cmd_vel`, `servo_cmd` | `/sensor/environment_mobile`, `/sensor/soil_nutrition`, `/sentry/chassis/status` | STM32F4 UART bridge |
| `lora_bridge_node` | LoRa gateway serial | `/sensor/environment_fixed` | Fixed env node data (multi-node, averaged in fusion) |
| `imu_node` | - | `/sensor/imu` | YB-IMU driver with CH340 ARM read patch |

### Decision

| Node | Subscribes | Publishes | Description |
|------|-----------|-----------|-------------|
| `fusion_node` | `/vision/diagnosis`, `/sensor/environment_fixed`, `/sensor/environment_mobile` | `/fusion/diagnosis` | LWD sliding window + priority gating + evidence chain |
| `forecast_node` | `/fusion/diagnosis` | `/forecast/alert` | Trend extrapolation (default), SIR-like model reserved |
| `advisory_node` | `/fusion/diagnosis`, `/forecast/alert` | `/advisory/action` | YAML rule engine, event-triggered |

### Control

| Node | Subscribes | Publishes | Description |
|------|-----------|-----------|-------------|
| `mission_control_node` | `/vision/plant_detected`, `/fusion/diagnosis`, `/advisory/action`, `/sentry/chassis/status` | `/sentry/cmd_vel`, `/mission/status` | Stop-photograph-classify-go state machine |
| `keyboard_control_node` | keyboard stdin | `/sentry/cmd_vel` | Manual driving via arrow keys |
| `web_remote_node` | HTTP API | `/sentry/cmd_vel` | Flask web frontend + rosbridge WebSocket |
| `wheel_odom_node` | chassis status | `/wheel/odometry` | Wheel odometry for EKF |
| `data_logger_node` | core topics | ros2 bag files | 7-day rotation, CRITICAL events permanently retained |

---

## Core Algorithms

### LWD Sliding Window & Cold Boot

Fixed env nodes sample every 5 minutes, maintaining a 288-point (24h) sliding window:

| Phase | Duration | LWD Strategy | LATENT_SUSPICION | Confidence |
|-------|----------|-------------|------------------|------------|
| COLD_BOOT | 0–30 min | Fallback to instantaneous humidity, cap 0.70 | Disabled | ×0.75 |
| WARM_UP | 30 min–24h | Short-term LWD linear extrapolation | Relaxed conditions | ×0.90 |
| NORMAL | ≥24h | Full 24h look-up table | Normal trigger | ×1.0 |

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

Issues and PRs welcome. Follow the development workflow in [CLAUDE.md](CLAUDE.md).

## License

MIT License. See [LICENSE](LICENSE) for details.
