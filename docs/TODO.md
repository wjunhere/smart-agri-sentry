# 当前任务与阻塞项

> 更新日期：2026-06-25

---

## Sprint 目标

1. 完成文档重组与 GPS 清理。
2. 推进 LiDAR SLAM/mapping 导航方案落地。
3. 补齐 STM32 固件与传感器协议验证。
4. 完成固定环境节点硬件与固件。

---

## 活跃任务

### 导航

- [ ] **评估 LiDAR SLAM 方案**：`slam_toolbox` vs `cartographer` 在 RDK X5 上的可行性与性能
- [ ] **设计地图保存/加载流程**：比赛场地建图、地图文件管理、启动时加载
- [ ] **新增 `nav2_with_map.yaml` 配置**：替换当前 `nav2_no_map.yaml`
- [ ] **改造 `mission_control_node`**：支持 `map` 帧航点、重定位恢复
- [ ] **验证 EKF  covariance 调参**：板端标定 IMU 噪声与里程计协方差

### STM32 固件

- [ ] **扩展 `TYPE_CHASSIS` 到 19 字节 payload**：追加 `left_pulse`、`right_pulse`、`encoder_timestamp`
- [ ] **验证编码器正交输入**：1000 线编码器 ×2 与定时器配置
- [ ] **确认电机驱动电流**：空载/堵转电流，决定 TB6612FNG 是否更换为 BTN7971B
- [ ] **确认电机 PID 参数**：与 Nav2 / MPPI 配合

### 传感器

- [ ] **获取七合一空气传感器 UART 协议文档**
- [ ] **获取七合一土壤传感器 UART 协议文档**
- [ ] **验证固定环境节点土壤传感器 RS485 协议**
- [ ] **确认 LoRa 参数**：频段（433 MHz/470 MHz）、扩频因子、网关与节点距离

### 视觉与模型

- [ ] **训练/获取 plant_detector 模型**：YOLO-Nano 或 MobileNet-SSD 叶片/植株检测
- [ ] **训练小麦病害模型**：当前 `wheat_mobilenetv3.onnx` 为占位
- [ ] **训练草莓病害模型**：当前 `strawberry_mobilenetv3.onnx` 为占位
- [ ] **建立 ONNX → .bin 转换脚本/流程**：确保部署到 RDK X5 自动化

### 固定环境节点

- [ ] **组装 STM32F103RCT6 + E22-400TBH-SC 固定环境节点硬件**
- [ ] **实现低功耗采集固件**：5 分钟采样、深度睡眠、异常上报
- [ ] **实现 LoRa 网关转发**：E22-400TBH-SC 内置 CBT6 接收后通过 UART（USB 转串口）输出 JSON 给 RDK X5
- [ ] **确认 IP65 外壳、太阳能板、电池安装方式**
- [ ] **如硬件来不及，先用模拟数据跑通 `env_bridge_node` 与 `fusion_node` 逻辑**

### 部署与验证

- [ ] **验证 `YbImuLib` 在 RDK X5 可用**
- [ ] **验证 `rosbag2_py` 在 RDK X5 可用**（如不可用，启用 JSON fallback）
- [ ] **验证 `nav2_bringup`、`robot_localization`、`imu_filter_madgwick` 已安装**
- [ ] **板端 `colcon build` 全包通过**
- [ ] **板端 `colcon test` 通过**

---

## 阻塞项

| 阻塞项 | 影响 | 缓解措施 |
|---|---|---|
| 电机驱动电流未确认 | TB6612FNG 可能烧毁 | 先用小功率测试，确认后决定更换 |
| 空气/土壤传感器协议文档缺失 | STM32 无法解析 | 继续向卖家索要文档；必要时抓包逆向 |
| `YbImuLib` 在 RDK X5 可用性未知 | IMU 节点无法运行 | 先验证依赖，必要时改用标准 `sensor_msgs/Imu` 驱动 |
| `rosbag2_py` 在 RDK X5 可用性未知 | data_logger 可能失败 | 已实现 JSON fallback |
| 小麦/草莓模型缺失 | 当前仅番茄可识别 | v2.0 先写通用框架，模型后续补全 |
| 固定节点硬件来不及 | 24h LWD 无法真实运行 | 用模拟数据跑通逻辑，硬件延后 |

---

## 近期已完成

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
