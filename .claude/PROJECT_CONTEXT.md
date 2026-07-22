# 智农哨兵 · 项目快速概览

> 架构版本 v3.0 · 更新日期 2026-07-19  
> 详细文档见 [`docs/`](../docs/)。

---

## Current field baseline

- Stable branch work from `fix/autonomous-cruise` has been merged into `main`.
- The robot has demonstrated three-point cruise, mission-owned short-range avoidance, frontend Preheat/Start/Pause/E-STOP, waypoint editing, and automatic stack stop after mission completion.
- Frontend gateway autostart: systemd `sentry-bridge.service` (installed once via `scripts/rdk/install_autostart.sh`) boots the gateway layer — miniprogram bridge :8765, web panel :5000, weather, LLM. Cruise stack is started/stopped from frontend buttons (`/stack/*`), no SSH needed.
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
| 摄像头 | 海康 MV-CS016-10UC（USB3，默认）/ IMX219 MIPI-CSI |
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
19. **海康相机软件自适应曝光 ✅ (2026-07-17)**：MV-CS016-10UC 硬件 AE 失效，新增节点内闭环软件 AE（`auto_exposure.py`）：帧亮度反馈 + 饱和保护 + 运动/静止分档曝光上限（巡航 20ms 防拖影 / 停车 100ms 提亮），里程计迟滞判定运动状态，`ae_enabled:=false` 可回退固定曝光。板端实测暗场景收敛 mean→80.0。参数与行为详见 `docs/ROS2.md` 第 9 节。
20. **前端免 SSH 直连小车 ✅ (2026-07-19, PR #3)**：统一网关方案——`miniprogram_bridge_node` 新增 `/stack/preheat|start|stop|status` 编排端点（复用 `start_robot_stack.sh`，`SENTRY_PRESERVE_WEB=1` 保留控制面），WS 推送 `stack_status`。修复两个断链 bug：bridge 订阅话题名 `/sentry/sensor/*`→`/sensor/*`（环境数据此前到不了前端）、小程序 `wsConnect()` 此前从未被调用（实时通道全断）。`miniprogram_bridge.launch.py` 升级为网关层 launch（bridge + web_remote + weather_node + llm_advisor，LLM key 读 `SENTRY_LLM_API_KEY`/`DEEPSEEK_API_KEY` 环境变量）。`scripts/rdk/install_autostart.sh` 安装 systemd `sentry-bridge.service` 开机自启网关，小车上电即可被前端发现，巡航全由按钮启停。小程序 IP 配置化（`services/config.ts` + 控制页设置）。bridge mock 测试基建修复（`_FakeNode` 真实基类），13 测试通过。Spec/plan 见 `docs/superpowers/{specs,plans}/2026-07-19-local-frontend-car-connection*`。板端联调中发现并修复：sentry_weather 缺 setup.cfg（libexec 缺失）、weather 真实模式 3h 不重发（加 60s 重发定时器）、stop 脚本 `miniprogram_bridge` pkill 误杀网关（已加入 PRESERVE_WEB 保护）、llm_advisor 气象字段名错误（.desc→.weather_desc 等）、新板缺 flask/imu_filter_madgwick（pip + 源码编译）。rdk1 (10.66.175.213) 网关自启、四页功能、DeepSeek 分析均已验证。待办：接底盘后验证 /sensor/* 数据与完整巡航。
21. **小程序 UI 优化 ✅ (2026-07-22)**：风格不变（复用 app.less 全部 token）四页优化。新增共享 `state-block` 组件（loading/empty/offline 三态）；控制页状态条合并（模式+连接点+IP）、dpad 80→96rpx、巡航按钮 busy 禁用态；监测页视频区 16:9、offline/loading 覆盖层、数值占位 `·`/`--` 分语义（连接无数据/未连接）；分析页 state-block 占位 + 建议序号蓝色块；天气页 offline 态 + 湿度真实映射。纯 CSS 微交互（:active scale、数值 color 过渡）。验证：wechatide 自动化截图对比基线 + 错误 IP offline 实测。Spec/plan 见 `docs/superpowers/{specs,plans}/2026-07-22-miniprogram-ui-polish*`。
22. **前端 v2 深色精致化重设计 ✅ (2026-07-23, PR #4)**：Ethereal Glass 语言（OLED 深底 + 径向微光晕、Double-Bezel 双层卡、岛屿药丸按钮、`cubic-bezier(0.32,0.72,0,1)` 弹簧动效）。共享块在 `app.less`（.v2-page/.shell/.core/.eyebrow/.btn-*）；小程序四页全部重写。天气页逐时预报：纯 Canvas 平滑曲线（Catmull-Rom 样条 + 渐变面积 + 顶部留白防裁切，wx:if 延迟重建规避原生 canvas 定位不跟随布局的坑）。Web 面板 `static_v2` 同语言 CSS-only 覆盖层重设计（不动 JS/模板），控制栏四分区：方向舵｜急停偏左｜作物分段胶囊（对齐巡航速度）｜巡航整行。wechatide/Chrome CDP 自动化截图验证。

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
