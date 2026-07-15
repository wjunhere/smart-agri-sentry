# 智农哨兵 · 项目快速概览

> 架构版本 v2.9 · 更新日期 2026-07-15  
> 详细文档见 [`docs/`](../docs/)。

---

## Current field baseline

- Stable branch work from `fix/autonomous-cruise` has been merged into `main`.
- The robot has demonstrated three-point cruise, mission-owned short-range avoidance, frontend Preheat/Start/Pause/E-STOP, waypoint editing, and automatic stack stop after mission completion.
- Details are intentionally split by topic: architecture in `docs/ARCHITECTURE.md`, ROS/HTTP interfaces in `docs/ROS2.md`, startup in `docs/SETUP.md`, decisions in `docs/DECISIONS.md`, known issues in `docs/ISSUES.md`, and remaining work in `docs/TODO.md`.
- RDK access: `ssh rdk` or `ssh sunrise@10.66.175.106`; frontend: `http://10.66.175.106:5000/`.

## 项目目标

面向番茄/小麦/草莓多作物病害巡检的嵌入式比赛原型机：

- 底盘自动巡航（当前 mapless Nav2，目标 LiDAR SLAM）
- 植株检测触发停车 → 端侧 AI 病害识别（RDK X5 BPU，`pyeasy_dnn` 推理）
- 移动/固定环境数据融合决策 → 农艺建议
- 本地 ros2 bag 数据记录
- 微信小程序远程控制（原生 TS + Less + Skyline，4 Tab 布局）
- FastAPI 桥接节点 `miniprogram_bridge_node :8765`（WebSocket 实时 + HTTP 控制）

---

## 核心硬件

| 模块 | 型号/方案 |
|---|---|
| AI 主控 | RDK X5（8 核 A55, R5 NPU 10 TOPS） |
| 运动控制 | STM32F407ZGT6（FreeRTOS） |
| 雷达 | STL19P / LD19（UART 230400） |
| 摄像头 | IMX219 MIPI-CSI |
| IMU | YB-IMU（CH340 USB, /dev/ttyUSB0 → /dev/myimu, 115200） |
| 云台 | 2-DOF 舵机，RDK X5 直接 PWM |
| 环境传感 | 移动七合一空气/土壤 + 固定 LoRa 节点 |

**注意**：GPS 模块已移除，不再使用。USB 串口设备识别：CH340=ttyUSB0=IMU，CP2102=ttyUSB2=LiDAR。LiDAR 波特率 230400。

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
10. **导航稳定性修复 ✅ (2026-07-08)**：修复 `wheel_odom_node` twist dt 硬编码、mission_control Nav2 任务失败重试、keyboard_control /cmd_vel 多发布者冲突、EKF 频率 30→10Hz、yaw_goal_tolerance 3.14→0.2、新增 transform_tolerance。详细见 commits `afe5f3e`~`95b75c4`。
11. **IMU CH340 ARM 驱动适配 ✅ (2026-07-08)**：YB-IMU (CH340, ttyUSB0, 115200) 在 RDK X5 ARM Linux 上 `in_waiting` 报告有数据但 `read()` 返回 0，导致 YbImuSerial 读线程崩溃。`imu_node.py` 增加 `_patch_ch340_read()` monkey-patch `read_all` 加容错重试。板端验证 IMU 数据正常发布。
12. **MIPI 相机 ISP 调通 ✅ (2026-07-08)**：IMX219 关键约束：`open_cam` 第一通道必须小分辨率 (512×512)，第二通道可设目标分辨率。`get_img(type=2, w, h)` 中 type=2 固定 NV12 格式，通道由传入的 w×h 匹配 `out_w/out_h` 列表决定。NV12 stride 因 ISP 硬件对齐可能 ≠ width，已改为根据 `actual_size / (height * 1.5)` 自动检测。前端 `image_transport republish raw compressed` 生成 `/out/compressed` 供 rosbridge 传输 JPEG。
13. **USB 串口设备识别**：CH340 (ttyUSB0) = IMU，CP2102 (ttyUSB2) = LiDAR。udev 规则：`/dev/myimu → ttyUSB0`，`/dev/wheeltec_lidar → ttyUSB2`。
14. **前端 mock 测试系统**：`static_v2/ros.js` 中 `injectMock()` + TOPICS 回调可硬编码各模块数据用于离线测试，修改处标注 `// === MOCK START/END ===` 便于恢复。
15. **病害分类阈值 ✅**：`vision_diagnosis_node` 新增 `healthy_threshold` 参数（默认 0.15），板端测试总体准确率 91.58%。
16. **天气 mock 周期修复**：`sentry_weather` mock 模式改 60s 周期发布，避免桥接节点错过单次消息。
17. **LLM 板端部署**：API key 需放在 `~/.bashrc` 交互守卫之前，否则非交互 SSH 无法加载。
18. **微信小程序 monitor 视频流优化 (2026-07-14)**：`<image>` 无法直接消费 MJPEG，后端新增 `/api/camera/snapshot` 并缓存 JPEG；前端用 A/B 双缓冲 + view 容器 opacity 过渡实现 200ms 平滑刷新，离开 monitor 页自动暂停。

## 模型矩阵

### 病害分类

| 作物 | 架构 | 类别数 | BPU 精度 | 输入 | Cosine | 准确率 | 部署状态 |
|------|------|--------|---------|------|--------|--------|---------|
| 番茄 | MobileNetV3-**Large** | 7 | int8 | NV12 224×224 | 0.9997 | 91.58% | ✅ 已部署 |
| 小麦 | MobileNetV3-Small | 5 | int8 | NV12 224×224 | 0.977 | — | ✅ 已部署 |
| 草莓 | MobileNetV3-Small | 8 | int16 | RGB 224×224 | 0.977 | — | ✅ 已部署 |

### 植株检测

| 任务 | 架构 | 类别数 | BPU 精度 | 输入 | Cosine | mAP50 | 部署状态 |
|------|------|--------|---------|------|--------|:---:|---------|
| Crop/Weed | YOLOv8n | 2 | int8 | NV12 640×640 | ≥0.997 | 0.860 | ✅ 已集成（`plant_detector_node`） |

> 量化配置与 ONNX 模型见 `models/quantization/`；推理节点见 `src/sentry_vision/`。YOLO 训练脚本与报告见 `D:\wjun\data\yolo\`。
