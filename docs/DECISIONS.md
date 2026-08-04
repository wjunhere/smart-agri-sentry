# 技术决策记录

> 更新日期：2026-07-15

---

## ADR-001：三层解耦 + 事件驱动巡检架构

**状态**：已实施  
**日期**：2026-04-21 → 2026-06-03

### 背景

v1.0 架构中所有传感器数据经 STM32 转发，存在移动传感器间歇工作导致环境历史断档的问题（v1.0 历史架构：彼时环境传感器挂载在移动节点上；当前移动节点仅负责视觉检测，环境数据全部由固定 LoRa 节点采集），无法支持 24h 叶面湿润时长（LWD）计算。

### 决策

引入三层解耦架构（感知 / 决策 / 控制）和事件驱动巡检：

- 固定环境节点（STM32F103RCT6 + CJ702 传感器 → E22-400TBH-SC LoRa 发送；E22-400TBH-SC 接收 → USB CDC → RDK X5）提供 24h 连续环境数据
- 植株检测节点触发停车事件
- 融合节点由"AI 触发"改为"停车事件触发"，内部持续维护 LWD 窗口

### 后果

- 支持 1440 点（24h，60s 帧）LWD 滑动窗口和冷启动降级
- 状态机清晰：PATROL → APPROACHING → STOPPED → ANALYZING → ACTION → RESUME
- 固定节点硬件未就绪时可用模拟数据跑通逻辑

---

## ADR-002：YAML 规则引擎替代 LLM

**状态**：已实施  
**日期**：2026-04-21

### 背景

v2.0 需要毫秒级农艺建议生成，且比赛场景要求结果可解释、可追溯。

### 决策

v2.0 使用结构化 YAML 规则引擎；端侧大模型（LLM）异步建议润色后置到 v3.0。

### 后果

- 建议生成确定性高、可审计
- 无需网络连接和额外算力
- 自然语言交互能力受限

---

## ADR-003：Mapless Nav2 + RPP 控制器

**状态**：已实施  
**日期**：2026-06-07

### 背景

比赛场景无预建地图，也不希望维护地图；需要快速实现底盘自动巡航与避障。

### 决策

采用无地图 Nav2：

- 全局规划：`NavfnPlanner`（Dijkstra）
- 局部规划：`RegulatedPurePursuitController` for the field baseline; MPPI was used earlier
- 代价地图：滚动窗口，odom 帧
- 航点：odom 坐标系下硬编码

### 后果

- 无需地图服务器和定位包，启动快
- 长距离运行会累积里程计漂移
- 为后续迁移到 LiDAR SLAM 埋下明确目标

---

## ADR-004：RDK X5 直接 PWM 驱动云台舵机

**状态**：已实施  
**日期**：2026-06-16

### 背景

原有链路 `/sentry/servo_cmd` → `uart_bridge_node` → STM32 驱动舵机，与 RDK X5 上层控制流程存在耦合和冲突风险。

### 决策

RDK X5 通过 40-pin PWM 直接驱动 yaw/pitch 两路舵机：

- yaw → Pin32 → `/sys/class/pwm/pwmchip0/pwm0`
- pitch → Pin33 → `/sys/class/pwm/pwmchip0/pwm1`
- `uart_bridge_node.forward_servo_cmd=False`（默认），避免 STM32 与 RDK 同时驱动

### 后果

- 脱离 STM32 即可快速验证云台
- 控制路径更短，时延更低
- 需要 `sunrise` 用户属于 `gpio` 组

---

## ADR-005：ONNX 源模型 + RDK X5 .bin 运行时

**状态**：已实施  
**日期**：2026-04-30

### 背景

模型训练在 PC 端产生 ONNX；RDK X5 NPU 需要 `.bin` 格式。需要保持单一源码、避免版本分叉。

### 决策

- `models/` 仓库中保留 `.onnx` 作为源格式
- RDK X5 部署时转换为 `.bin`
- `vision_diagnosis_node` 自动将显式传入的 `.tflite` 路径重写为 `.bin`

### 后果

- 单一天然模型来源
- 部署流程需包含转换步骤
- 文件名约定：`{crop}_mobilenetv3.onnx` → `{crop}_mobilenetv3.bin`

---

## ADR-006：LiDAR SLAM / 建图导航作为目标导航方案

**状态**：已决策，待实现  
**日期**：2026-06-25

### 背景

当前 mapless Nav2 在长距离巡航时受限于编码器里程计漂移，航点精度下降；用户明确希望使用激光雷达建图导航。

### 决策

将导航目标升级为 LiDAR SLAM/mapping：

- 候选工具：`slam_toolbox` 或 `cartographer`
- 输出：持久化地图、`map` 坐标系、定位/重定位能力
- 与现有 Nav2 规划器兼容

### 任务

- 评估 `slam_toolbox` vs `cartographer` 在 RDK X5 上的可行性与性能
- 设计地图保存/加载流程
- 修改 `mission_control_node` 支持 `map` 帧航点
- 新增/调整 Nav2 配置文件（`nav2_with_map.yaml`）

### 后果

- 降低长距离里程计漂移影响
- 增加地图维护、回环闭合调试工作量
- 需要在比赛场地提前建图或在线建图

---

## ADR-007：ChassisStatus 扩展而非新增帧类型

**状态**：已决策，部分实现  
**日期**：2026-06-07

### 背景

导航合并需要编码器脉冲数据，可选方案：

- 方案 A：在现有 `ChassisStatus` 中增加字段
- 方案 B：新增独立的编码器帧类型

### 决策

采用方案 A：扩展 `ChassisStatus`，追加 `left_pulse`、`right_pulse`、`encoder_timestamp`。

- `TYPE_CHASSIS` payload 从 7 字节 → 19 字节
- 向后兼容旧版 7 字节帧解析

### 后果

- 减少消息类型数量
- STM32 固件需同步更新
- `wheel_odom_node` 直接订阅 `/sentry/chassis/status`

---

## ADR-008：LiDAR 双话题输出

**状态**：已实施  
**日期**：2026-06-03

### 背景

Nav2 需要标准 `sensor_msgs/LaserScan`，而 `fusion_node` 只需要前方障碍物简化信息。

### 决策

`sentry_lidar` 同时发布：

- `/scan`：标准 LaserScan
- `/lidar/obstacle_info`：自定义 ObstacleInfo，含前方扇区最近/平均距离、障碍物标志

### 后果

- 降低 `fusion_node` 计算负担
- 增加一个自定义消息类型 `ObstacleInfo`
- 扇区预处理参数可配置

---

## ADR-009：IMU TF 由 EKF 发布

**状态**：已决策，待实现  
**日期**：2026-06-03

### 背景

Madgwick 滤波器和 `robot_localization` EKF 都可能发布 `odom → base_link` TF，存在冲突。

### 决策

- Madgwick `publish_tf: false`
- EKF 融合 `/wheel/odom` + `/sensor/imu/data` 后发布 `/odom` 和 TF

### 后果

- TF 单一来源，避免抖动
- EKF 协方差需要板端调参

---

## ADR-011：LoRa 固定环境节点数据聚合与双模帧支持

**状态**：已实施  
**日期**：2026-06-28

### 背景

固定环境节点通过 LoRa 将传感器数据传到 RDK X5。发送端 MCU（STM32F103RCT6）连接 CJ-702 空气传感器（当前）、土壤传感器和叶面温湿度传感器（未来）。需要决定数据聚合位置和帧格式。

### 决策

1. **MCU 端聚合**：发送端 MCU 在本地把所有传感器数据聚合成一帧后 LoRa 发出，RDK 侧只需解析一帧即可获得完整环境数据。
2. **统一帧格式**：`AA 55 | device_id | msg_type | payload_len | payload | CRC-8/MAXIM`
   - `msg_type 0x01` = 聚合环境数据（14 字节 CJ702 仅空气，或 24 字节全传感器）
   - `msg_type 0xFF` = 错误帧
3. **独立 lora_bridge_node**：不复用 `uart_bridge_node`，职责分离，LoRa 固定节点与移动底盘 UART 节点解耦。
4. **扩展 Environment.msg**：新增 `hcho`/`tvoc`/`pm25`/`pm10`/`leaf_temp`/`ec` 字段，把所有关键环境量集中在一条消息里。
5. **双模兼容**：RDK 侧同时支持 14 字节（当前 CJ702）和 24 字节（未来全传感器）payload，自动适配。

### 后果

- 代码实现见 `src/sentry_sensors/sentry_sensors/lora_bridge_node.py`
- 固定节点固件当前仅发送 14 字节空气帧，后续接入土壤/叶面传感器需升级到 24 字节
- CRC-8/MAXIM 无输入输出反射，与 MCU 端 `lora_frame.c` 保持一致
- E22 模块必须处于透传模式（Mode 0），接收端固件已修复默认 work_mode 从 2→0
- 集成测试记录见 `test/stm32_cj702_lora_hal/TESTS.md`

---

## ADR-010：病害分类 healthy 阈值策略

**状态**：已实施  
**日期**：2026-07-13

### 背景

MobileNetV3 7 类番茄病害模型在板端测试中 healthy 类别召回率仅 69.6%，大量健康植株被误判为病害。需要一种机制降低误报率。

### 决策

在 `vision_diagnosis_node` 中新增 `healthy_threshold` 参数（默认 0.15）：

- 若 softmax 后 `healthy` 类概率 ≥ 阈值 → 强制预测为 healthy
- 否则 → argmax 所有 7 类

### 后果

- 板端 1995 张测试集验证：总体准确率 89.62% → 91.58%，healthy 召回率 69.6% → 84.5%
- 阈值可通过 ROS2 参数动态调整（`--ros-args -p healthy_threshold:=0.2`）
- 实现见 `src/sentry_vision/sentry_vision/vision_diagnosis_node.py:72-77`

---

## ADR-012: Frontend-owned stack scripts and mission-owned obstacle avoidance

**Status**: Implemented  
**Date**: 2026-07-15

### Context

Field tests repeatedly exposed stale ROS nodes, old TF publishers, and temporary `/cmd_vel` publishers. They made frontend startup, cruise validation, and obstacle testing difficult to trust. Pure Nav2 costmap avoidance also tended to bend the whole path too early when an obstacle was still around 1m away, which did not match crop-row inspection expectations.

### Decision

- Add `scripts/rdk/start_robot_stack.sh` and `scripts/rdk/stop_robot_stack.sh` as the field-demo entry points.
- Let `web_remote_node` expose `/stack/preheat`, `/stack/start`, `/stack/stop`, and `/waypoints`.
- Make Preheat start and verify the ROS stack without switching AUTO.
- Make Start Cruise switch `/set_auto_mode=true` only after the stack is ready.
- Put short-range obstacle behavior in `mission_control_node`: stop Nav2, publish zero velocity, back up, choose the clearer side, drive around, turn back, rejoin, then hand control back to Nav2.
- Suppress normal obstacle re-triggering during the internal avoidance sequence and for a short period after rejoin.

### Consequences

- Field demos are more repeatable and frontend-driven.
- The main crop-row path remains straighter until an obstacle is close enough to matter.
- Startup is slower than a raw launch, but it includes cleanup and health checks.
- The same mission-owned safety layer can remain useful after a future map-based navigation upgrade.


---

## ADR-011：植株检测改单类 plant + COCO 基座全量重训（yolo11s）

**状态**：已实施
**日期**：2026-07-28

### 背景

原 crop/weed 二分类 YOLOv8n 在板端对屏幕/打印的病害图片不出框（只对"密集叶片+根部"出框）。排查出四类原因：部署端 conf/min_area 过滤过狠；训练 letterbox 与板端直接 resize 拉伸不一致；量化校准集全是数据集图、与板端输入分布不符；模型本身对"图片中的叶片"这一分布没见过。

### 决策

1. **类别合并为单类 `plant`**：板端后处理本来就取两类置信度最大值（`yolo_utils.py`），下游不消费 crop/weed 区分（病害分类由 MobileNet 负责）。单类省去类间区分负担，召回更高。
2. **数据闭环用板端实拍**：训练/校准数据必须与部署输入同分布。病害图通过平板显示 + 板端相机翻拍获得（330 张），硬负样本（风扇、吊灯、遮阳网、地膜等 13 类）用"网络搜图 → 平板翻拍"解决场景受限问题（160 张）。
3. **接力微调不可持续，改为 COCO 基座全量重训**：R1（best.pt 微调）、R2（R1 再微调）指标提升但误检感加重——每轮微调向最新数据漂移，且 PlantDoc 叶片特写占比过高使模型"纹理过敏"。最终 yolo11s 从 COCO 预训练基座用全量 5108 张一次训到位。
4. **更大模型换判别力**：v8n(3.2M) → v11s(9.4M)，BPU 估算 272→80 FPS 仍有 8 倍富余（相机 10~15fps），容量提升正对"风扇网罩 vs 叶丛"这类细粒度纹理区分。

### 结果

- mAP50 0.970、mAP50-95 0.645（板端实拍验证集，与 R1/R2 同一套）；硬负样本误检 21(R1) → 14(R2) → **4/160**；风扇误检消除（残留为肥料袋绿叶图案与纯绿几何图）。
- 板端配套：conf 0.35 + 时序投票（3帧2票）压边界/偶发误检。

### 经验记录

- **校准集必须用板端实拍图**（含负样本），数据集图校准会让量化掉点且分布错位。
- 预标注 + 人工修正的标注效率远高于纯手标（330 张约半天）；LabelImg 1.8.6 与新版 PyQt5 有 float→int 兼容问题需打补丁。
- 微调数据配比：新图 : 原数据 ≈ 1:2，lr 降 1/10；验证集必须含部署域（板端实拍）样本，混合域指标会虚高。

---

## ADR-012：番茄病害分类板端域适应（v5 翻拍微调 + YOLO 裁剪推理管线）

**状态**：已实施
**日期**：2026-07-30

### 背景

v4.2 MobileNetV3（数字基准 91.8%）在板端对打印/屏幕显示的病害图识别严重失真（early_blight 打印图被判 healthy 或 leaf_mold）。A/B 定位三连对比：数字原图 float 推理 0.946 正确 → 板端翻拍帧 float 推理错误 → 板端量化 bin 更错。**模型没问题，是"打印→翻拍"域差 + 整帧分类 + 量化三者叠加**。

### 决策

1. **训练分布 = 推理分布**：推理输入从整帧 resize 改为 **YOLO plant 框外扩 20% 裁剪 + letterbox 224**（灰 114 填充）。整帧下即使 v5 也认不准，裁剪是必需项不是优化项。
2. **域适应数据零标注采集**：平板分页播放训练集图片（每类一批，目录名即标签），板端相机翻拍。质量门只要两条可控项——整帧亮度 ≥60、屏幕占比 ≥30%（清晰度不可控：源图 224px 放大显示本身就软）。ORB 回源匹配做标签校验（纯特征点匹配在小图上有假阳性，必须加 RANSAC 几何验证；且只能验证"拍的是哪张"，不能代表"拍得好"）。
3. **翻拍 + 数字原图混合微调**：638 张翻拍（7 类，early_blight 加量）+ 每类 200~250 张数字原图（同走 YOLO 裁剪预处理），冻结 backbone 训 classifier → 全网低 lr 两段式，随机曝光增强兜底暗光鲁棒性。量化校准集同样换板端翻拍图。
4. **healthy_threshold 退役**：v4.2 时代 0.15 阈值是为补 healthy 召回；v5 校准后该阈值在数字基准上起反作用（89.8% vs 92.0%），默认改 0（禁用，0 时不再强判）。

### 结果

- v5 数字基准 92.0%/91.8%（acc/macroF1，略超 v4.2 且未用阈值）；原失败帧 YOLO 裁剪下 early_blight 0.78/0.91；板端实测打印 early_blight 稳定识别。
- 推理栈三节点：plant_detector + vision_pipeline + vision_diagnosis（流式，订阅 `/vision/plant_detected` 取框）。

### 经验记录

- **前端占位数据会伪装成真实故障**：ros.js 的 mock `healthy 0.85` 占位在无 `/vision/diagnosis` 发布者时长期显示，排查板端"误识别"前先确认前端显示的是真实话题数据。
- 翻拍采集的翻拍质量（亮度/屏占比）和标签正确性要分开校验；增强变体近重复的源图要用感知哈希去重后再做播放清单。
- 全套流程脚本在 `D:\wjun\data\toamtos\`：`slideshow/`（分页播放+补拍生成+去重）、`ab_test/`（A/B、ORB 校验、质量门）、`finetune/`（数据集构建 `build_finetune_dataset.py` + 训练 `finetune_v5.py`），小麦/草莓换病识别不准时直接复用。
