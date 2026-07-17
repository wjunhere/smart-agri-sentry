# 智农哨兵 · 实施计划

> 生成日期：2026-07-05 · 基于 PROJECT_CONTEXT v2.3  
> 需完成 P0 即可跑通完整演示闭环；P1-P3 为功能完整项；P4 为后续版本。

---

## P0：完整演示闭环（阻塞项）

### P0-1 两阶段视觉管线集成

- [x] YOLOv8n BPU `.bin` 接入 `plant_detector_node`（`pyeasy_dnn` 推理，640×640 NV12）
- [x] `plant_detector_node` → `/vision/plant_detected` → `mission_control_node` 触发停车
- [x] `vision_pipeline_node` 固定相机扫描编排 + 两阶段推理 + 汇总诊断（已移除云台移动，改为固定画面多帧推理）
- [x] `PipelineTrigger.srv` 同步服务定义
- [x] `mission_control_node` 重构（移除 APPROACHING，新增 SCANNING，里程计去重）
- [x] `Diagnosis.msg` 新增 `per_angle_confidences`
- [x] 板端 camera 驱动验证（MIPI IMX219 + overlay 加载）
- [x] 板端 YOLO BPU 推理跑通（多输出 DFL 后处理）
- [ ] MobileNet 病害分类板端联调
  - [x] **M1** 将 `mobilenetv3_tomato_disease_v4.2.onnx` 量化成 RDK X5 `.bin`
  - [ ] **M2** 板端 Python 推理验证（logits → softmax → healthy 阈值 0.15）
  - [ ] **M3** 接入 `vision_pipeline_node` 作为第二阶段分类器
- [ ] 固定相机扫描端到端测试（无需舵机，验证巡航 → 检测 → 停车 → 拍照 → 病害识别 → 农艺建议）

### P0-2 板端部署验证

- [ ] 验证 `YbImuLib` 在 RDK X5 可用（不可用则换标准 `sensor_msgs/Imu` 驱动）
- [ ] 验证 `rosbag2_py` 在 RDK X5 可用（不可用则启用 JSON fallback）
- [ ] 验证 `nav2_bringup`、`robot_localization`、`imu_filter_madgwick` 已安装
- [ ] 板端 `colcon build` 全包通过
- [ ] 板端 `colcon test` 通过
- [ ] 板端全系统启动验证（`ros2 launch sentry_bringup sentry_v2.launch.py crop_type:=tomato`）

---

## P1：导航升级（mapless → LiDAR SLAM）

- [ ] 评估 `slam_toolbox` vs `cartographer` 在 RDK X5 的可行性与性能
- [ ] 设计地图保存/加载流程（比赛场地建图、启动加载）
- [ ] 新增 `nav2_with_map.yaml` 配置（替换 `nav2_no_map.yaml`）
- [ ] 改造 `mission_control_node` 支持 `map` 帧航点、AMCL 重定位恢复
- [ ] 实车建图 + 导航端到端验证
- [ ] EKF covariance 板端调参（IMU 噪声 + 里程计协方差）

---

## P2：固定环境节点硬件 + LoRa 联调

- [ ] 组装 STM32F103RCT6 + CJ702 + 叶面 RS485 + 土壤 TTL + E22-400TBH-SC
- [ ] 确认 LoRa 参数：频段（433/470 MHz）、扩频因子、网关与节点距离
- [ ] LoRa TX/RX 端到端收发验证（节点 → 网关 → RDK X5 `lora_bridge_node`）
- [ ] 实现低功耗睡眠逻辑（5 分钟睡眠、秒级唤醒采集）
- [ ] 确认 IP65 外壳、太阳能板、18650 电池安装方式
- [ ] 固定节点野外 24h 连续采集验证

---

## P3：底盘硬件验证与调参

- [ ] 确认电机驱动电流（空载/堵转），决定 TB6612FNG 是否换 BTN7971B
- [ ] 验证编码器正交输入（1000 线 ×2 + 定时器配置）
- [ ] PID 参数板端标定（与 Nav2 / MPPI 配合的实车调参）

---

## 临时任务：fix/autonomous-cruise 分支修复（2026-07-10）

> 详细计划见 `docs/superpowers/plans/2026-07-10-autonomous-cruise-fixes.md`

解决自主巡航实测中 Nav2 "Failed to make progress" 的软件层缺陷。

- [ ] **T1** 添加 `mode_byte_for_mission_state` 纯函数 + off-board 测试
- [ ] **T2** 重构 `uart_bridge_node` 模式帧：心跳 + 状态联动 + shutdown 发待机
- [ ] **T3** 移除死参数 `cruise_speed`（mission_control + launch）
- [ ] **T4** launch 添加 `nav2_lifecycle_manager`
- [ ] **T5** `nav2_no_map.yaml` 添加 `progress_checker`（0.15m / 20s）
- [ ] **T6** 删除孤儿 `config/mission_params.yaml` + `setup.py` 引用
- [ ] **T7** 删除 `AGENT.md`，`tmp_rdk_auto_cruise_test.sh` → `scripts/`
- [ ] **T8** 新增 `scripts/chassis_direct_test.sh` 底盘直控诊断脚本
- [ ] **T9** 本地验证：off-board 测试 + 编译 + YAML lint

---

## 临时任务：海康相机软件自适应曝光（2026-07-17）

> 设计：`docs/superpowers/specs/2026-07-17-hikrobot-adaptive-exposure-design.md`
> 计划：`docs/superpowers/plans/2026-07-17-hikrobot-adaptive-exposure.md`
> 分支：`feat/adaptive-exposure`（基线 dbce5c4 + 4 提交：dcdb665, f475699, 013cff8, b08172c）

解决海康 MV-CS016-10UC 硬件自动曝光失效导致的过曝/拖影问题。

- [x] **T0** 板端 codex 未提交代码基线入库（dbce5c4）
- [x] **T1** 纯 Python 曝光控制器 TDD（`auto_exposure.py`，15 测试，dcdb665）
- [x] **T2** 节点集成 TDD（odom 迟滞运动判定 + 寄存器写入 + 硬件回读种子，f475699）
- [x] **T3** launch 切换软件 AE 默认参数（013cff8），本地 33 测试全过
- [x] **T4** 板端实测：暗场景收敛 mean→80.0（目标值）、动/静模式切换、硬件回读一致（ExposureAuto=0, ExposureTime=20000, GainAuto=0, Gain=11.99）、逃生舱 `ae_enabled:=false` 验证
- [x] **T5** Code review + 修复（b08172c：安全钳制绕过限频、写寄存器失败回滚重试），修复版板端冒烟通过；文档更新（ROS2.md 第 9 节、ISSUES.md、PROJECT_CONTEXT.md）

---

## 临时任务：巡航检测消息推送（2026-07-17）

> 设计：`docs/superpowers/specs/2026-07-17-mission-message-center-design.md`
> 计划：`docs/superpowers/plans/2026-07-17-mission-message-center.md`
> 分支：`feat/mission-message-center`

前端顶部状态栏消息中心：巡航批次内检测植株快照（画检测框）+ 病害类别，存 web_remote_node 内存，未读角标 + 一键清理。

- [x] **T1** `BatchRecorder` 纯逻辑 TDD（批次开关、空批次丢弃、诊断回填、未读、清理）
- [x] **T2** 快照画框 `draw_bbox_on_jpeg` TDD
- [x] **T3** web_remote_node 接线 TDD（PATROL→STOPPED 边沿快照、固定点无框不记录、class_id 254 哨兵忽略、`/api/messages*` 路由、`/status.message_unread`）
- [x] **T4** 前端：top-bar 铃铛 + 未读角标、message-center modal（批次分组、缩略图、大图预览、一键清理）
- [x] **T5** 本地 48 测试全过 + code review 修复（BatchRecorder 加 RLock 线程安全、bbox 长度校验、批次上限 10）+ 板端部署
- [ ] 板端巡航实测（用户验证：角标 → 快照带框 → 多批次 → 清理）

---

## P4：后续完善

- [x] Web 前端仪表盘（`static_v2/` Vue 3 + roslibjs，v2.0 已完成）
- [x] 微信小程序遥控终端（`wechat/` 原生 TS + Less + Skyline，4 Tab，v2.0 已完成）
- [x] 云端 LLM 农情分析（DeepSeek API，`sentry_llm` 包，v2.0 已完成）
  - [x] **2026-07-13 板端部署验证**：`sentry_interfaces` + `sentry_llm` + `sentry_miniprogram` colcon build 通过
  - [x] `llm_advisor_node` 启动正常，API key 从 `~/.bashrc` 加载（需放在交互守卫之前）
  - [x] `miniprogram_bridge_node` 启动正常，Uvicorn `0.0.0.0:8765`
  - [x] LLM 测试 7/7 通过；Bridge mock 测试因 Python 版本差异跳过（非代码 bug）
  - [ ] 待板端网络恢复后：全系统启动验证（`miniprogram_bridge.launch.py`）
- [ ] 外部天气 API 接入
- [ ] InfluxDB + Grafana 离线分析管线
- [ ] 端侧 LLM 农艺建议润色（v3.0）

---

## 阻塞项

| 阻塞项 | 影响 | 缓解措施 |
|---|---|---|
| 两阶段视觉管线已集成 | YOLO BPU 推理+后处理完成，待 MobileNet 联调 | 降低阈值到 0.2，或使用真实植株重新验证 |
| 板端未验证 | 所有节点可能在 RDK X5 上构建/运行失败 | 优先验证依赖，必要时禁用/替换 |
| 电机驱动电流未确认 | TB6612FNG 可能烧毁 | 小功率测试后决定 |
| 固定节点 LoRa 未联调 | 24h LWD 环境融合无法真实运行 | 移动传感器可先跑通融合逻辑，固定节点延后 |
