# 技术决策记录

> 更新日期：2026-06-25

---

## ADR-001：三层解耦 + 事件驱动巡检架构

**状态**：已实施  
**日期**：2026-04-21 → 2026-06-03

### 背景

v1.0 架构中所有传感器数据经 STM32 转发，存在移动传感器间歇工作导致环境历史断档的问题，无法支持 24h 叶面湿润时长（LWD）计算。

### 决策

引入三层解耦架构（感知 / 决策 / 控制）和事件驱动巡检：

- 固定环境节点（STM32L072 + LoRa）提供 24h 连续环境数据
- 植株检测节点触发停车事件
- 融合节点由"AI 触发"改为"停车事件触发"，内部持续维护 LWD 窗口

### 后果

- 支持 288 点（24h）LWD 滑动窗口和冷启动降级
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

## ADR-003：Mapless Nav2 + MPPI 控制器

**状态**：已实施  
**日期**：2026-06-07

### 背景

比赛场景无预建地图，也不希望维护地图；需要快速实现底盘自动巡航与避障。

### 决策

采用无地图 Nav2：

- 全局规划：`NavfnPlanner`（Dijkstra）
- 局部规划：`MPPIController`
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

## ADR-010：数据记录本地优先

**状态**：已实施  
**日期**：2026-06-13

### 背景

比赛场地通常无网络，需要本地可靠记录核心数据。

### 决策

- RDK X5 SD 卡本地 `ros2 bag` 选择性录制
- 7 天循环覆盖
- CRITICAL 事件前后 5 分钟永久保留到 `records/critical/`
- InfluxDB + Grafana 仅用于回办公室后离线分析

### 后果

- 离线运行不依赖网络
- SD 卡容量和写入寿命需关注
