# 智农哨兵 · 项目快速概览

> 架构版本 v2.5 · 更新日期 2026-07-06  
> 详细文档见 [`docs/`](../docs/)。

---

## 项目目标

面向番茄/小麦/草莓多作物病害巡检的嵌入式比赛原型机：

- 底盘自动巡航（当前 mapless Nav2，目标 LiDAR SLAM）
- 植株检测触发停车 → 端侧 AI 病害识别（RDK X5 BPU，`pyeasy_dnn` 推理）
- 移动/固定环境数据融合决策 → 农艺建议
- 本地 ros2 bag 数据记录

---

## 核心硬件

| 模块 | 型号/方案 |
|---|---|
| AI 主控 | RDK X5（8 核 A55, R5 NPU 10 TOPS） |
| 运动控制 | STM32F407ZGT6（FreeRTOS） |
| 雷达 | STL19P / LD19（UART 230400） |
| 摄像头 | IMX219 MIPI-CSI |
| IMU | YB-IMU（CH340 USB） |
| 云台 | 2-DOF 舵机，RDK X5 直接 PWM |
| 环境传感 | 移动七合一空气/土壤 + 固定 LoRa 节点 |

**注意**：GPS 模块已移除，不再使用。

---

## 核心软件栈

- ROS2 Humble on Ubuntu 22.04
- Nav2 + robot_localization EKF
- Python/C++ 混合节点（`sentry_*` 包）
- ONNX 训练模型 → `hb_mapper` 量化 → RDK X5 BPU `.bin`（`pyeasy_dnn` 加载）

---

## 文档地图

| 文档 | 内容 |
|---|---|
| [`docs/ARCHITECTURE.md`](../docs/ARCHITECTURE.md) | 系统架构、模块划分、数据流、当前/目标导航 |
| [`docs/HARDWARE.md`](../docs/HARDWARE.md) | 硬件规格、传感器、接线、FreeRTOS、通信协议 |
| [`docs/ROS2.md`](../docs/ROS2.md) | 节点图、话题/服务/参数、消息定义、TF |
| [`docs/DECISIONS.md`](../docs/DECISIONS.md) | 技术决策记录（ADR） |
| [`docs/TODO.md`](../docs/TODO.md) | 当前 Sprint 任务与阻塞项 |
| [`docs/ISSUES.md`](../docs/ISSUES.md) | 已知问题、硬件限制、规避方案 |
| [`docs/SETUP.md`](../docs/SETUP.md) | 环境搭建、编译、启动、STM32 烧录、模型部署 |

开发规范见根目录 [`CLAUDE.md`](../CLAUDE.md)。

---

## 当前重点

1. **病害模型量化部署 ✅**：三作物 MobileNetV3 已量化为 BPU `.bin`，`vision_diagnosis_node` 迁移至 `pyeasy_dnn` 推理。
2. **底盘控制 ✅**：STM32F407 固件编译烧录通过，RDK↔STM32 UART 协议联调完成。新增 `chassis_cmd` 编码器闭环运动测试工具与 `imu_turn_controller` IMU 陀螺仪闭环原地转弯控制器（地面无关精度 ~4%）。
3. **YOLOv8 植株检测 ✅**：Crop/Weed 二分类检测模型训练完成（mAP50=0.860），已量化为 BPU `.bin`（cosine ≥ 0.997）。
4. 导航升级：从 mapless Nav2 迁移到 LiDAR SLAM/mapping。
5. **传感器协议 ✅**：CJ702 七合一空气传感器 UART 协议已解析；叶面传感器 RS485 ModBus 驱动完成；土壤 NPK 七合一 TTL ModBus 驱动完成（含地址自动探针）。
6. **固定环境节点固件 🔄**：STM32F103RCT6 三传感器同步采集固件完成（空气 CJ702 + 叶面 RS485 + 土壤 NPK ModBus），LoRa 帧打包就绪；待完成低功耗睡眠逻辑与 LoRa 发送联调。
7. **植株检测 + 病害分类两阶段管线 ✅**：YOLOv8n BPU 接入 `plant_detector_node`，新建 `vision_pipeline_node` 云台多角度扫描编排，`mission_control_node` 重构（移除 APPROACHING，新增 SCANNING + 里程计去重）。板端相机驱动（IMX219 overlay）和 YOLO 推理已调通。舵机初始位置已校准（yaw=67.5°, pitch=45°）。待 MobileNet 联调和全链路实测。
8. **键盘控制底盘 ✅**：新增 `keyboard_control_node`（`sentry_mission` 包），方向键控制线速度 ±0.05 m/s，角速度 ±0.05 rad/s，空格急停，Q 退出。复用 `web_remote_node` 的 MANUAL 模式 + `/cmd_vel` 发布机制，0.5s 无操作自动停车。注册为 `ros2 run sentry_mission keyboard_control` 入口点。
9. **STM32 GCC 构建 ✅**：新增 `firmware/chassis/Makefile`，使用 `arm-none-eabi-gcc` 直接编译烧录，绕过 Keil AC5/AC6 兼容问题。`make` 编译，`make flash` 通过 STM32_Programmer_CLI(SWD) 烧录。

## 模型矩阵

### 病害分类

| 作物 | 架构 | 类别数 | BPU 精度 | 输入 | Cosine | 部署状态 |
|------|------|--------|---------|------|--------|---------|
| 番茄 | MobileNetV3-**Large** | 7 | int8 | NV12 224×224 | 0.9997 | ✅ 已部署 |
| 小麦 | MobileNetV3-Small | 5 | int8 | NV12 224×224 | 0.977 | ✅ 已部署 |
| 草莓 | MobileNetV3-Small | 8 | int16 | RGB 224×224 | 0.977 | ✅ 已部署 |

### 植株检测

| 任务 | 架构 | 类别数 | BPU 精度 | 输入 | Cosine | mAP50 | 部署状态 |
|------|------|--------|---------|------|--------|:---:|---------|
| Crop/Weed | YOLOv8n | 2 | int8 | NV12 640×640 | ≥0.997 | 0.860 | ✅ 已集成（`plant_detector_node`） |

> 量化配置与 ONNX 模型见 `models/quantization/`；推理节点见 `src/sentry_vision/`。YOLO 训练脚本与报告见 `D:\wjun\data\yolo\`。
