# 当前任务与阻塞项

> 更新日期：2026-08-06

---

## Sprint 目标

1. 完成文档重组与 GPS 清理。
2. 推进 LiDAR SLAM/mapping 导航方案落地。
3. **传感器协议验证 ✅**：CJ702 空气 + 叶面 RS485 + 土壤 NPK ModBus 驱动完成。
4. **固定环境节点固件 + LoRa 上行 🔄**：三传感器采集 + LoRa 帧打包完成；RDK 侧 LoRa 上行链路已接通（`lora_bridge` opt_v2、`/dev/lora`、进任务栈，2026-08-06）；待节点端低功耗睡眠与野外部署。

---

## 活跃任务

### 导航

- [ ] **评估 LiDAR SLAM 方案**：`slam_toolbox` vs `cartographer` 在 RDK X5 上的可行性与性能
- [ ] **设计地图保存/加载流程**：比赛场地建图、地图文件管理、启动时加载
- [ ] **新增 `nav2_with_map.yaml` 配置**：替换当前 `nav2_no_map.yaml`
- [ ] **改造 `mission_control_node`**：支持 `map` 帧航点、重定位恢复
- [ ] **验证 EKF  covariance 调参**：板端标定 IMU 噪声与里程计协方差

### STM32 固件

- [x] **扩展 `TYPE_CHASSIS` 到 19 字节 payload**：追加 `left_pulse`、`right_pulse`、`encoder_timestamp`（v2.1 协议，2026-06-30）
- [ ] **验证编码器正交输入**：1000 线编码器 ×2 与定时器配置
- [ ] **确认电机驱动电流**：空载/堵转电流，决定 TB6612FNG 是否更换为 BTN7971B
- [ ] **确认电机 PID 参数**：与 Nav2 / MPPI 配合

### 传感器

- [x] **获取七合一空气传感器 UART 协议文档**：CJ702 协议已解析（17 字节帧，含 CO₂/HCHO/TVOC/PM2.5/PM10/温湿度）
- [x] **获取七合一土壤传感器 UART 协议文档**：TTL ModBus 寄存器映射已确认（8 寄存器：湿度/温度/EC/pH/N/P/K/盐分）
- [x] **验证固定环境节点叶面/土壤传感器协议**：叶面 RS485 ModBus 驱动完成（UART1，地址 0x01）；土壤 TTL ModBus 驱动完成（UART4，地址自动探针 0x01-0x03）
- [ ] **确认 LoRa 参数**：频段（433 MHz/470 MHz）、扩频因子、网关与节点距离

### 视觉与模型

- [x] **训练/获取 plant_detector 模型**：YOLOv8n Crop/Weed 二分类检测（mAP50=0.860），已量化为 BPU `.bin`（cosine ≥ 0.997）
- [x] **训练小麦病害模型**：MobileNetV3-Small 5 类已量化部署
- [x] **训练草莓病害模型**：MobileNetV3-Small 8 类已量化部署
- [x] **建立 ONNX → .bin 转换流程**：`hb_mapper` 量化管线已建立
- [x] **番茄病害模型板端测试 (2026-07-13)**：MobilenetV3-Large 准确率 91.58%（healthy_threshold=0.15），7 类 1995 张测试集
- [x] **病害分类 healthy 阈值 (2026-07-13)**：`vision_diagnosis_node` 新增 `healthy_threshold` 参数（默认 0.15），healthy 召回率 69.6%→84.5%

### 固定环境节点

- [x] **三传感器同步采集固件**：CJ702 空气 + 叶面 RS485 ModBus + 土壤 TTL ModBus 驱动完成，60s 窗口聚合，LoRa 帧打包就绪（`test/stm32_cj702_lora_hal/`）
- [ ] **组装 STM32F103RCT6 + E22-400TBH-SC 固定环境节点硬件**
- [ ] **实现低功耗睡眠逻辑**：5 分钟睡眠、秒级采集唤醒
- [x] **实现 LoRa 网关转发（RDK 侧）**：`lora_bridge_node` 切 opt_v2 帧协议（0xAA sync + TYPE/SEQ/FLAG/LEN + CRC16-CCITT），默认口 `/dev/lora`（udev 按 hub 物理口 1-1.4 绑定），RELIABLE QoS；进 `sentry_v2.launch` 任务栈 + `stop_robot_stack.sh` 清理名单（2026-08-06）
- [ ] **确认 IP65 外壳、太阳能板、电池安装方式**
- [x] **LoRa 发送联调（端到端）**：固定节点运行中 60s 一帧，RDK `lora_bridge_node` 解析并经 `/sensor/environment_fixed` 发布，前端/小程序消费 12 字段（2026-08-06）

### 部署与验证

- [ ] **验证 `YbImuLib` 在 RDK X5 可用**
- [ ] **验证 `rosbag2_py` 在 RDK X5 可用**（如不可用，启用 JSON fallback）
- [ ] **验证 `nav2_bringup`、`robot_localization`、`imu_filter_madgwick` 已安装**
- [x] **LLM + 小程序板端部署验证 (2026-07-13)**：`sentry_interfaces`/`sentry_llm`/`sentry_miniprogram` colcon build 通过，`llm_advisor_node` + `miniprogram_bridge_node` 启动正常
- [x] **天气数据链路验证 (2026-07-13)**：`sentry_weather` mock 60s 周期，`/api/weather` 端到端通过，字段名对齐+浮点精度修复
- [ ] **板端 `colcon test` 全通过**（bridge mock 基建已修复为 `_FakeNode` 真实基类，13 测试在开发机 venv 通过；板端全量待跑）
- [ ] **前端免 SSH 板端联调 (PR #3, 2026-07-19)**：板端 `git pull && colcon build` → `bash scripts/rdk/install_autostart.sh` → 重启验证网关自启 → 小程序逐页端到端验证（视频/环境/天气/巡航按钮）+ Web 面板回归

---

## 阻塞项

| 阻塞项 | 影响 | 缓解措施 |
|---|---|---|
| 电机驱动电流未确认 | TB6612FNG 可能烧毁 | 先用小功率测试，确认后决定更换 |
| ~~空气/土壤传感器协议文档缺失~~ ✅ | ~~STM32 无法解析~~ | CJ702 + 叶面 RS485 + 土壤 NPK ModBus 驱动已完成 |
| `YbImuLib` 在 RDK X5 可用性未知 | IMU 节点无法运行 | 先验证依赖，必要时改用标准 `sensor_msgs/Imu` 驱动 |
| `rosbag2_py` 在 RDK X5 可用性未知 | data_logger 可能失败 | 已实现 JSON fallback |
| 小麦/草莓模型缺失 | 当前仅番茄可识别 | v2.0 先写通用框架，模型后续补全 |
| ~~固定节点 LoRa 联调未完成~~ ✅ | ~~固定环境数据无法回传~~ | RDK 侧上行已接通（2026-08-06）；剩余节点端低功耗睡眠与野外部署 |

---

## 近期已完成

- [x] **任务栈巡航可靠性 + LoRa 上行 + 相机翻转 (2026-08-06, ADR-013)**：LoRa 上行进栈（opt_v2 协议、`/dev/lora`、RELIABLE QoS、前端/小程序消费 12 字段）；相机倒装 `flip_code=-1`；舵机/换行自动翻转默认开启 + 巡航自动结束/手动停止均复原舵机；修复"识别到植株不停下"（巡航开始恢复检测器 + `avoidance_scanned_radius` 抑制已扫描植株避障）；视觉节点 respawn 自愈；重复节点检查改进程数；检测投票边沿日志
- [x] **小程序 UI 优化 (2026-07-22)**：state-block 三态组件 + 四页排版/布局/微交互优化（风格不变）；控制页状态条合并、监测页 16:9+三态、分析页序号色块、天气页湿度真实映射；错误 IP offline 实测通过
- [x] **前端免 SSH 板端联调 (rdk1, 2026-07-22)**：修复 sentry_weather setup.cfg、weather 真实模式 60s 重发、stop 脚本误杀网关、llm 气象字段名；补装 flask/imu_filter_madgwick；网关自启 + /stack/* 全周期 + 四页 UI + DeepSeek 分析全部验证通过
- [x] **前端免 SSH 直连小车 (PR #3, 2026-07-19)**：bridge 新增 `/stack/*` 巡航编排端点 + WS `stack_status` 推送；修复话题名断链（`/sentry/sensor/*`→`/sensor/*`）与 `wsConnect()` 未接线两个 bug；网关层 launch（+weather_node +web_remote）；systemd `sentry-bridge.service` 开机自启；小程序 IP 配置化 + 巡航按钮组 + 连接角标；bridge mock 测试基建修复（13 测试通过）
- [x] **微信小程序 monitor 视频流去频闪 (2026-07-14)**：snapshot 双缓冲改为 z-index 前后层预加载，取消 opacity 过渡，避免 Skyline 硬切/黑底闪烁；刷新间隔 200ms，切 tab 暂停
- [x] **番茄病害模型板端测试 (2026-07-13)**：MobilenetV3-Large 准确率 91.58%（healthy_threshold=0.15）
- [x] **病害分类 healthy 阈值**：`vision_diagnosis_node` 新增参数，healthy 召回率 69.6%→84.5%
- [x] **LLM 农情分析板端部署**：`sentry_llm` 构建通过，API key 加载问题修复（`.bashrc` 守卫）
- [x] **天气数据链路修复**：字段名对齐、浮点精度、mock 周期 60s
- [x] **微信小程序天气页增强**：emoji 图标、7 日温度柱状图、逐时 CSS 柱状图（蓝紫渐变）
- [x] **`sentry_bringup` launch 安装修复**：`setup.py` 改用 glob 匹配所有 launch 文件
- [x] **固定环境节点三传感器驱动**：CJ702 空气 + 叶面 RS485 ModBus + 土壤 TTL ModBus（NPK/EC/pH），含地址自动探针与 OpenOCD 调试支持
- [x] **底盘运动控制工具**：`chassis_cmd` 编码器闭环运动测试 + `imu_turn_controller` IMU 陀螺仪闭环原地转弯（精度 ~4%）
- [x] Phase 2 节点：`forecast_node`、`advisory_node`、`data_logger_node` 落地
- [x] RDK X5 直接 PWM 驱动云台舵机
- [x] IMU 集成设计（待实现）
- [x] LiDAR 驱动设计与 `sentry_lidar` 包（待实现）
- [x] 项目文档重组：拆分 `.claude/PROJECT_CONTEXT.md` 到 `docs/`

---

## 阶段路线图

| 阶段 | 目标 | 状态 |
|---|---|---|
| Phase 1 | 消息接口 + plant_detector + vision_diagnosis + fusion + mission_control | 已完成 |
| Phase 2 | forecast + advisory + data_logger | 已完成 |
| Phase 3 | 固定环境节点硬件固件 + env_bridge | 待实现 |
| Phase 4 | 外部天气 + Web 前端 + InfluxDB 离线分析 | 后续完善 |
| **Phase N** | **LiDAR SLAM/mapping 导航迁移** | **已决策，待实现** |

---

## Field Cruise Status (2026-07-15)

Completed:

- [x] Field cruise/avoidance/frontend baseline merged from `fix/autonomous-cruise` into `main`.
- [x] Three-point cruise, mission-owned short-range avoidance, frontend Preheat/Start/Pause/E-STOP, waypoint editing, and auto stop-on-completion were demonstrated on the robot.
- [x] Full start/stop scripts are available under `scripts/rdk/`.

Next incremental work:

- [ ] Keep future cruise changes evidence-driven: change one parameter or behavior group at a time, then run the same field validation.
- [ ] Consider cleaning old tracked temporary shell test scripts from repository history/tree in a separate cleanup change.



---

## 视觉/部署后续优化项（2026-07-28 记）

1. **Nav2 降频**（用户确认暂缓）：`controller_frequency` 20→10Hz、costmap `update_frequency` 5→3Hz、local costmap 3m→2m，预估省 40~60% CPU。0.18 m/s 车速下影响可忽略（刹车反应 +50ms ≈ 1cm）。全栈 + 15fps 预估 405~485% CPU 超载，降频后 ~280~320%。
2. **真实植株数据集**（暂缓中）：250~400 张板端实拍真植株（田间/阳台），补屏幕图与真实场景的分布差；当前模型在真植株上的精度未重新验证。
3. **板端预处理 letterbox 对齐**：`yolo_utils.py` 目前直接 `cv2.resize` 拉伸，与训练 letterbox 不一致，小目标有掉点；改补灰边 letterbox 约十几行。
4. **肥料袋/纯绿图案硬负样本补强**：yolo11s 残留的 4/160 误检集中在这两类，下轮数据迭代时精准补拍。
5. **yolo11s + 全栈巡航实测**：新模型下"检测→停车→扫描诊断"全链路田间验证未做。
6. **第三次微调数据集已就绪**：`D:\wjun\data\yolo\train_plant_full`（5108 张单类）；训练/导出/量化/部署全流程已跑通三轮，可直接复用脚本（`train_yolo11s.py`、`export_monkey_patch.py`、`models/yolo_quantize/mapper.py`）。
7. **巡航可靠性修复田间回归 (2026-08-06 改动)**：在真实植株场景验证"识别→停车→扫描→继续"，重点覆盖：MANUAL→AUTO 切换后检测器在线（无 WARN）、手动停止舵机回中、已扫描植株再次经过不再触发避障。
