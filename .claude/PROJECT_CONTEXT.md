# 智农哨兵 - 项目上下文（PROJECT_CONTEXT）

> 文档用途：供 Claude Code / 团队成员快速理解项目约束、接口定义和技术栈
> 更新日期：2026-06-16
> 适用场景：嵌入式比赛项目，原型样机阶段
> **架构版本：v2.1 导航增强 + 事件驱动巡检**

---

## 版本变更记录

### v2.1 → 本次更新（2026-06-16）
1. **RDK X5 直接 PWM 舵机驱动**：新增 `sentry_servo` 包
   - `servo_driver.py`：Linux sysfs PWM 驱动，支持 0–180°、500–2500 µs、50 Hz
   - `servo_driver_node`：订阅 `/sentry/servo_cmd`，直接驱动 RDK X5 pwmchip0/pwm0（yaw）和 pwm1（pitch）
   - `servo_keyboard`：独立键盘脚本，无需 ROS2 依赖，用于快速调试
   - `servo_keyboard_node`：ROS2 节点版键盘控制，发布 `/sentry/servo_cmd`，可融入任务流程
   - 配置文件 `config/servo_config.yaml`：yaw 0–180°、pitch 30–150°、初始 90°/90°、步进 5°
2. **舵机控制接入任务流程**：
   - `sentry_v2.launch.py` 自动启动 `servo_driver_node`
   - `uart_bridge_node` 新增 `forward_servo_cmd` 参数（默认 `False`），避免 STM32 与 RDK X5 同时驱动舵机冲突
3. **键盘控制验证完成**：板端验证方向键可连续匀速控制云台，无卡顿

### v2.1 → 本次更新（2026-06-13）
1. **Phase 2 节点实现完成**：forecast / advisory / data_logger 三个 ROS2 Python 包已落地并上板验证
   - 新增 `sentry_forecast` 包：`forecast_node` 基于线性趋势外推发布 `/forecast/alert`
   - 新增 `sentry_advisory` 包：`advisory_node` 基于 YAML 规则引擎发布 `/advisory/action`
   - 新增 `sentry_data_logger` 包：`data_logger_node` 使用 `rosbag2_py.SequentialWriter` 选择性录制，CRITICAL 事件触发快照永久保留
   - 三个节点已注册到 `sentry_v2.launch.py`
   - RDK X5 验证：`colcon build` 10 个包全部通过，`colcon test` 24 tests / 0 failures
2. **视觉节点重构**：`vision_diagnosis_node` 抽离为 `diagnosis_utils.py`，支持 `.bin` 模型自动匹配与番茄/小麦/草莓三作物标签映射
3. **遥控节点完善**：`web_remote_node` 增加 `/set_auto_mode` 异步服务回调结果处理
4. **任务控制节点修正**：`mission_control_node` 在 MANUAL 模式下仍发布 `/mission/status`，同时抑制 `/cmd_vel`
5. **bringup 清理**：移除 `ai_inference_node.py`、`uart_bridge_node.py` 及对应 entry_point；`mipi_camera_node` topic 统一为 `/sentry/camera/image_raw`

### v2.1 → 本次更新（2026-06-07）
1. **新增完整导航栈**：集成 Nav2 + EKF + 轮式里程计 + 航点巡航
   - 新增 `wheel_odom_node`：从底盘编码器脉冲计算 dead reckoning，发布 `/wheel/odom`
   - 新增 EKF（robot_localization）：融合 `/wheel/odom` + `/sensor/imu/data` → `/odom`
   - 新增 Nav2：NavfnPlanner（全局）+ MPPIController（局部）+ 无地图 costmap 模式
   - 重构 `mission_control_node`：PATROL（Nav2 航点巡航）→ APPROACHING（视觉伺服）→ STOPPED → ANALYZING → ACTION → RESUME → PATROL
   - 新增 `web_remote_node`：Flask HTTP 遥控，支持 AUTO/MANUAL 模式切换和急停
2. **扩展底盘帧协议**：`ChassisStatus` 新增 `left_pulse`, `right_pulse`, `encoder_timestamp`
   - `TYPE_CHASSIS` payload 从 7 字节 → 19 字节（向后兼容旧版 7 字节帧）
3. **统一 cmd_vel topic**：所有速度指令统一发布到 `/cmd_vel`（Nav2 / mission_control / web_remote）
4. **航点管理**：默认弓字形全覆盖航点，支持 YAML 配置，病害检测触发时保存当前航点、分析完成后恢复

### v2.0 → v2.1（2026-06-03）
1. **新增 LiDAR 感知**：集成 STL19P 激光雷达（LDLiDAR SDK 迁移），发布 `/scan`（导航/避障）和 `/lidar/obstacle_info`（融合决策）
2. **架构升级**：由"混合架构（GPS直连 + 传感器经STM32转发）"升级为**三层解耦 + 事件驱动巡检**
   - 引入固定环境节点（STM32L072 + SX1262 LoRa），解决移动传感器间歇工作导致的环境历史断档
   - 引入植株检测节点（YOLO-Nano），实现"停-拍-判-走"事件驱动巡检
   - 融合节点由"AI触发"改为"停车事件触发"，内部持续维护LWD窗口
   - 新增预测预警节点（简化趋势外推）和农艺建议节点（YAML规则引擎）
3. **数据流重构**：由"异步订阅 + 时间戳对齐"升级为**事件驱动状态机**
   - `mission_control_node` 管理 CRUISING → APPROACHING → STOPPED → ANALYZING → ACTION → RESUME 状态机
   - `plant_detector_node` 检测到植株 → `mission_control` 停车 → `vision_diagnosis` 拍照 → `fusion_node` 融合 → `advisory_node` 建议 → 记录 → 恢复巡航
4. **消息接口重构**：废弃 `AiDiagnosis` / `FinalDiagnosis` / `SensorCombined`，统一为 8 个新消息
5. **环境数据源扩展**：移动传感器（随车，1Hz）+ 固定环境节点（田间24h连续，5min采样）双源策略
6. **作物支持扩展**：从单一番茄（10类）扩展为番茄/小麦/草莓三作物动态切换
7. **Advisory技术路线**：v2.0 使用结构化YAML规则引擎，端侧大模型后置到v3.0
8. **数据存储策略**：RDK X5 本地 ros2 bag 7天循环 + CRITICAL永久保留；InfluxDB/Grafana放办公室PC离线分析

### v1.0 → v2.0（2026-04-21）
1. 架构由"全部传感器经STM32转发"改为混合架构
2. GPS（G60）直连 RDK X5 UART6
3. 传感器数据经 STM32 打包转发
4. 定义自定义二进制帧协议（CRC16-CCITT）

---

## 1. 项目概述

- **名称**：智农哨兵
- **性质**：嵌入式比赛项目，三人团队，无机械加工条件
- **核心目标**：在无网农田环境下，实现"底盘自动巡航 + 植株检测 + 端侧AI病害识别 + 环境融合决策 + 农艺建议生成 + 本地数据记录"
- **非目标**：不追求工业级防护、不追求续航极限、Web前端放到后续版本

---

## 2. 硬件平台

### 2.1 主控与运动
| 模块 | 型号/规格 | 备注 |
|------|-----------|------|
| **AI主控** | RDK X5 | 8核A55, 8GB LPDDR4, 旭日R5 NPU (10 TOPS), 功耗~3W |
| **运动控制** | STM32F407ZGT6 | 最小系统板, 168MHz, FreeRTOS |
| **电机** | 24V 直流减速电机 ×2 | 需确认空载/堵转电流 |
| **电机驱动** | TB6612FNG（待验证） | 持续电流1.2A；若电机电流过大，更换为BTN7971B |
| **编码器** | 1000线光电编码器 ×2 | 接STM32定时器正交编码器输入 |
| **底盘** | 履带式（采购成品） | 无机械加工条件 |
| **云台** | 2-DOF舵机云台（采购成品） | 控制摄像头俯仰/偏航；yaw→Pin32(pwm0), pitch→Pin33(pwm1) |
| **运行速度** | **0.5 m/s（典型工况）** | 巡航速度 |

### 2.2 传感器与连接方式

#### 移动传感器（随车，经STM32转发）
| 传感器 | 连接方式 | 接至 | 数据项 | 周期 | ROS2 Topic |
|--------|----------|------|--------|------|------------|
| **七合一空气质量** | UART | STM32 | 温度、湿度、CO₂ | 1s (100Hz分频) | `/sensor/environment_mobile` |
| **七合一土壤** | UART | STM32 | 电导率、氮、磷、钾、温度、湿度、pH | 1s (100Hz分频) | `/sensor/soil_nutrition` |
| **GPS北斗双模** | UART | RDK X5（直连） | 经纬度、速度、航向 | 1s | `/sentry/gps/fix` |
| **MIPI摄像头** | CSI | RDK X5 | 图像/视频流 | 500ms | `/sentry/camera/image_raw` |
| **激光雷达** | UART (CP2102) | RDK X5 | 360° 距离/强度点云 | 10Hz | `/scan`, `/lidar/obstacle_info` |

#### 固定环境节点（田间24h连续，低功耗野外版）
| 传感器 | 型号 | 接口 | 数据项 | 采样周期 | 备注 |
|--------|------|------|--------|----------|------|
| **空气温湿** | SHT30 | I2C (0x44) | 温度、湿度 | 5min | 百叶箱内，冠层中部 |
| **CO₂** | SCD40 | I2C (0x62) | CO₂ | 5min | 同百叶箱 |
| **土壤参数** | RS485三合一 | UART→RS485 | 温度、湿度、EC | 5min | 根区10-15cm |
| **叶面湿度** | LWS10 | ADC 模拟 | 叶面湿度 | 5min | 代表性叶片背面 |

**固定节点通信链**：STM32L072 + SX1262 (LoRa) → LoRa网关 (ESP32-S3 + SX1262) → USB转串口 → RDK X5 → `env_bridge_node` → `/sensor/environment_fixed`

### 2.3 通信与电源
| 项目 | 方案 |
|------|------|
| **RDK ↔ STM32** | UART2（Pin 15/17），波特率 115200，自定义二进制帧 |
| **RDK ↔ GPS** | UART6（Pin 16/18），波特率 9600，NMEA-0183 协议 |
| **RDK ↔ LoRa网关** | USB转串口（TTL），波特率 115200 |
| **固定节点供电** | 10W太阳能 + 18650×2（并联4000mAh），CN3791 MPPT |
| **主电源** | 24V锂电池组 |
| **降压分配** | 24V→5V/3.3V DC-DC给RDK、STM32、传感器；24V直驱电机 |

---

## 3. 全链路时间周期与数据流

### 3.1 任务周期总表

| 节点/任务 | 所在平台 | 周期 | 频率 | 说明 |
|-----------|----------|------|------|------|
| `TaskSensor` | STM32 | 100 ms | 10 Hz | 读取空气+土壤传感器 |
| `TaskControl` | STM32 | 20 ms | 50 Hz | 电机PID闭环、编码器反馈、舵机控制 |
| `TaskComm` | STM32 | 100 ms | 10 Hz | 打包上传传感器+底盘状态，接收RDK控制指令 |
| `camera_node` | RDK X5 | 500 ms | 2 Hz | MIPI摄像头采集 |
| `plant_detector_node` | RDK X5 | 200 ms | 5 Hz | YOLO-Nano 植株检测（轻量） |
| `vision_diagnosis_node` | RDK X5 | 500 ms | 2 Hz | TFLite 作物-specific 病害分类 |
| `gps_node` | RDK X5 | 100 ms | 10 Hz | UART6 读取 G60 NMEA |
| `uart_bridge_node` | RDK X5 | 10 ms | 100 Hz | UART2 轮询读取STM32帧 |
| `env_bridge_node` | RDK X5 | 事件 | - | LoRa网关串口数据到达即解析发布 |
| `fusion_node` | RDK X5 | 内部2Hz | - | 持续维护LWD窗口；FusionResult发布与mission状态联动 |
| `forecast_node` | RDK X5 | 10 min | 0.1 Hz | 简化趋势外推，后台运行 |
| `advisory_node` | RDK X5 | 事件触发 | - | Fusion/Forecast变化时生成建议 |
| `mission_control_node` | RDK X5 | 100 ms | 10 Hz | 状态机驱动，发布cmd_vel |
| `data_logger_node` | RDK X5 | 事件 | - | 选择性bag录制 |
| `sentry_lidar` | RDK X5 | 10 Hz | - | STL19P 激光雷达驱动，发布 LaserScan + ObstacleInfo |
| `servo_driver_node` | RDK X5 | 事件 | - | 订阅 `/sentry/servo_cmd`，直接输出 PWM 驱动云台 |
| `servo_keyboard_node` | RDK X5 | 事件 | - | 键盘输入发布 `/sentry/servo_cmd`，用于手动调试 |

### 3.2 时空误差分析（基于 0.5 m/s）

| 周期 | 位移 | 影响分析 |
|------|------|----------|
| **20 ms**（控制周期） | **1 cm** | 电机PID响应足够 |
| **100 ms**（传感器周期） | **5 cm** | 传感器数据与位置匹配误差约5cm |
| **200 ms**（植株检测） | **10 cm** | 检测延迟对应10cm位移，需确保检测视野覆盖 |
| **500 ms**（AI推理周期） | **25 cm** | 停车后才触发病害分类，不影响巡航精度 |

> **关键变化**：v2.0 事件驱动模式下，AI推理在停车后执行，500ms周期不再影响巡航精度。但 plant_detector 的 200ms 延迟需在 APPROACHING 阶段补偿。

### 3.3 数据流图（v2.0 事件驱动）

```
┌─────────────────────────────────────────────────────────────────────┐
│                           STM32F407ZGT6                              │
│  ┌─────────────┐    ┌─────────────┐    ┌─────────────────────────┐  │
│  │ 空气传感器  │    │ 土壤传感器  │    │ 电机PID + 编码器 + 舵机 │  │
│  │ (UART4)     │    │ (UART5)     │    │ (TIM/PWM)               │  │
│  └──────┬──────┘    └──────┬──────┘    └───────────┬─────────────┘  │
│         │                  │                        │                │
│         └──────────────────┴────────────────────────┘                │
│                            │                                         │
│                            ▼                                         │
│                   ┌─────────────────┐                                │
│                   │   TaskSensor    │ 100ms                          │
│                   │   TaskControl   │ 20ms                           │
│                   │   TaskComm      │ 100ms                          │
│                   └────────┬────────┘                                │
│                            │ UART2_TX                                │
└────────────────────────────┼────────────────────────────────────────┘
                             │
                             ▼
┌─────────────────────────────────────────────────────────────────────┐
│                            RDK X5                                    │
│                                                                      │
│  ┌─────────────┐    ┌─────────────┐    ┌─────────────────────────┐  │
│  │   GPS G60   │    │ uart_bridge │    │  MIPI Camera            │  │
│  │  (UART6)    │    │  (UART2)    │    │  (IMX219)               │  │
│  └──────┬──────┘    └──────┬──────┘    └───────────┬─────────────┘  │
│         │                  │                        │                │
│  ┌──────────────────────────────────────────────────────────────┐   │
│  │     sentry_lidar (STL19P + CP2102, UART 230400)             │   │
│  │           → /scan  +  /lidar/obstacle_info                  │   │
│  └──────────────────────────────────────────────────────────────┘   │
│  ┌──────────────────────────────────────────────────────────────┐   │
│  │     imu_node + imu_filter_madgwick                            │   │
│  │           → /sensor/imu/data_raw → /sensor/imu/data          │   │
│  └──────────────────────────────────────────────────────────────┘   │
│         │                  │              │          │                │
│         ▼                  ▼              ▼          ▼                │
│  /sentry/gps/fix    /sensor/         /sentry/camera/    /sensor/    │
│                     environment_mobile image_raw        imu/data    │
│                     /sensor/                                         │
│                     soil_nutrition                                   │
│                                                                      │
│  ┌──────────────────────────────────────────────────────────────┐   │
│  │  USB转串口 ← LoRa网关 ← [固定环境节点: STM32L072+SX1262]   │   │
│  │         env_bridge_node → /sensor/environment_fixed        │   │
│  └──────────────────────────────────────────────────────────────┘   │
│                                                                      │
│  ┌──────────────────────────────────────────────────────────────┐   │
│  │     wheel_odom_node  (编码器脉冲 → dead reckoning)           │   │
│  │           → /wheel/odom                                       │   │
│  └──────────────────────────────────────────────────────────────┘   │
│                            │                                         │
│                            ▼                                         │
│  ┌──────────────────────────────────────────────────────────────┐   │
│  │     ekf_filter  (robot_localization)                         │   │
│  │     /wheel/odom + /sensor/imu/data → /odom                  │   │
│  └──────────────────────────────────────────────────────────────┘   │
│                            │                                         │
│                            ▼                                         │
│  ┌──────────────────────────────────────────────────────────────┐   │
│  │     Nav2 (nav2_bringup)                                       │   │
│  │     /odom + /scan → /cmd_vel (AUTO模式)                     │   │
│  │     NavfnPlanner + MPPIController (mapless)                 │   │
│  └──────────────────────────────────────────────────────────────┘   │
│         ┌──────────────────────────────────────────┐                 │
│         │         plant_detector_node              │                 │
│         │   (YOLO-Nano, 5fps, 检测植株bbox)        │                 │
│         └──────────────────┬───────────────────────┘                 │
│                            │                                         │
│                            ▼ /vision/plant_detected                  │
│         ┌──────────────────────────────────────────┐                 │
│         │         mission_control_node             │                 │
│         │   (PATROL→APPROACHING→STOPPED→          │                 │
│         │    ANALYZING→ACTION→RESUME→PATROL)      │                 │
│         │   Nav2航点巡航 + 视觉伺服 + 模式切换     │                 │
│         └──────────┬─────────────────────┬───────┘                 │
│                    │                     │                          │
│                    ▼                     ▼                          │
│              /cmd_vel (AUTO)      STOPPED触发拍照                   │
│                                                                      │
│         ┌──────────────────────────────────────────┐                 │
│         │         vision_diagnosis_node            │                 │
│         │   (TFLite, 作物-specific, 2fps)          │                 │
│         └──────────────────┬───────────────────────┘                 │
│                            │                                         │
│                            ▼ /vision/diagnosis                       │
│         ┌──────────────────────────────────────────┐                 │
│         │         fusion_node                      │                 │
│         │   (LWD滑动窗口 + 优先级门控 + 证据链)    │                 │
│         │   输入: /vision/diagnosis + 环境双源     │                 │
│         └──────────────────┬───────────────────────┘                 │
│                            │                                         │
│                            ▼ /fusion/diagnosis                       │
│         ┌──────────────────────────────────────────┐                 │
│         │         forecast_node    (10min周期)     │                 │
│         │         advisory_node    (事件触发)      │                 │
│         └──────────────────┬───────────────────────┘                 │
│                            │                                         │
│                            ▼ /forecast/alert                         │
│                            ▼ /advisory/action                        │
│         ┌──────────────────────────────────────────┐                 │
│         │         web_remote_node                  │                 │
│         │   (Flask HTTP, AUTO/MANUAL + 急停)      │                 │
│         └──────────────────────────────────────────┘                 │
│         ┌──────────────────────────────────────────┐                 │
│         │         servo_driver_node                │                 │
│         │   (RDK X5 PWM → yaw/pitch)              │                 │
│         │   ← /sentry/servo_cmd                   │                 │
│         └──────────────────────────────────────────┘                 │
│         ┌──────────────────────────────────────────┐                 │
│         │         data_logger_node                 │                 │
│         │   (ros2 bag, 7天循环, CRITICAL永久保留)  │                 │
│         └──────────────────────────────────────────┘                 │
└─────────────────────────────────────────────────────────────────────┘
```

---

## 4. 通信协议定义

### 4.1 STM32 ↔ RDK X5（UART2，115200，3.3V TTL）

采用**自定义二进制帧**。

#### 帧格式
```
[帧头2B] [类型1B] [长度1B] [载荷nB] [CRC16-CCITT 2B]
0xAA 0x55   TYPE     LEN      DATA       CRC16
```

#### 数据类型（TYPE）

| TYPE | 方向 | 含义 | 载荷内容 |
|------|------|------|----------|
| `0x01` | STM32→RDK | **传感器汇总帧** | 空气温湿度CO₂ + 土壤电导率/氮磷钾/温湿度/pH |
| `0x03` | STM32→RDK | **底盘状态帧** | 左轮速、右轮速、电池电压、报警位、编码器脉冲(L/R)、时间戳 |
| `0x81` | RDK→STM32 | **运动控制帧** | 左轮目标速、右轮目标速（mm/s） |
| `0x82` | RDK→STM32 | **云台控制帧**（可选） | 舵机俯仰角、偏航角（角度值）；仅当 `uart_bridge_node.forward_servo_cmd=True` 时转发 |
| `0x83` | RDK→STM32 | **模式切换帧** | 0x00=待机, 0x01=遥控, 0x02=自动巡航 |

#### 传感器汇总帧（TYPE=0x01）载荷定义（v2.0 不变）

```c
typedef struct {
    uint32_t timestamp_ms;      // STM32开机后的毫秒时间戳
    int16_t  air_temp_x10;      // 空气温度 ×10（0.1℃）
    uint16_t air_humi_x10;      // 空气湿度 ×10（0.1%RH）
    uint16_t air_co2;           // CO₂浓度（ppm）
    int16_t  soil_temp_x10;     // 土壤温度 ×10（0.1℃）
    uint16_t soil_humi_x10;     // 土壤湿度 ×10（0.1%RH）
    uint16_t soil_ec;           // 土壤电导率（us/cm）
    uint16_t soil_n;            // 氮含量（mg/kg）
    uint16_t soil_p;            // 磷含量（mg/kg）
    uint16_t soil_k;            // 钾含量（mg/kg）
    uint16_t soil_ph_x10;       // pH值 ×10（0.1pH）
} __attribute__((packed)) SensorFrame_t;
// 总长度：2+1+1+24+2 = 30 字节
```

**v2.0 拆分**：`uart_bridge_node` 解析 TYPE=0x01 后，拆分为两个 ROS2 消息发布：
- `/sensor/environment_mobile` (`Environment` 消息)：空气温湿/CO₂、土壤温湿/EC
- `/sensor/soil_nutrition` (`SoilNutrition` 消息)：N/P/K/pH/EC

#### CRC校验
- 算法：**CRC16-CCITT** (`0x1021`)
- 范围：从 `类型` 字节到 `载荷` 末尾
- 初始值：`0xFFFF`

### 4.2 GPS ↔ RDK X5（UART6，9600，NMEA-0183）

- **协议**：标准 NMEA-0183，GGA + RMC
- **ROS2话题**：`/sentry/gps/fix`，类型 `sensor_msgs/NavSatFix`
- **精度**：2.5m（水平），无RTK

### 4.3 固定环境节点 ↔ LoRa网关（LoRa，433MHz/470MHz）

**节点端（STM32L072）**：
- 深度睡眠，每 5 分钟唤醒采集一次
- 每小时批量发送 12 条记录，或异常时立即上报
- 数据包格式（JSON 简化）：`{"node_id":"01","t":23.5,"h":78.0,"co2":450,"st":22.1,"sh":65.0,"ec":1.2,"lw":0,"seq":123}`

**网关端（ESP32-S3）**：
- 接收 LoRa 数据包，通过 USB 转串口转发给 RDK X5
- 转发格式：JSON + `\n` 换行分隔

**RDK X5 端（env_bridge_node）**：
- 解析串口 JSON，转换为 `Environment` 消息
- `data_source` 字段设为 `FIXED_NODE_01` / `FIXED_NODE_02` / ...
- 支持多点，Fusion Node 内部取平均

---

## 5. ROS2 话题设计（RDK X5 内部）

### 5.1 感知层

| 话题名 | 类型 | 发布者 | 订阅者 | 频率 | 说明 |
|--------|------|--------|--------|------|------|
| `/sentry/camera/image_raw` | `sensor_msgs/Image` | camera_node | plant_detector, vision_diagnosis | 2Hz | 摄像头原始图像，统一为 `/sentry/camera/image_raw` |
| `/vision/plant_detected` | `PlantDetection` | plant_detector_node | mission_control | 5Hz | 植株检测结果（bbox + 置信度） |
| `/vision/diagnosis` | `Diagnosis` | vision_diagnosis_node | fusion_node | 2Hz | 病害分类结果（crop_type + class_id + probabilities） |
| `/sensor/environment_mobile` | `Environment` | uart_bridge_node | fusion_node | 1Hz | 移动传感器环境数据（data_source=MOBILE） |
| `/sensor/soil_nutrition` | `SoilNutrition` | uart_bridge_node | data_logger | 1Hz | 土壤营养分离（N/P/K/pH/EC） |
| `/sensor/environment_fixed` | `Environment` | env_bridge_node | fusion_node | 事件 | 固定环境节点（data_source=FIXED_NODE_XX） |
| `/sentry/gps/fix` | `sensor_msgs/NavSatFix` | gps_node | mission_control, data_logger | 1Hz | GPS定位 |
| `/scan` | `sensor_msgs/LaserScan` | sentry_lidar | Nav2/避障 | 10Hz | 激光雷达点云（360°） |
| `/lidar/obstacle_info` | `ObstacleInfo` | sentry_lidar | fusion_node | 10Hz | 前方扇区障碍物简化信息 |
| `/sentry/chassis/status` | `ChassisStatus` | uart_bridge_node | mission_control | 10Hz | 底盘状态 |

### 5.2 决策层

| 话题名 | 类型 | 发布者 | 订阅者 | 频率 | 说明 |
|--------|------|--------|--------|------|------|
| `/fusion/diagnosis` | `FusionResult` | fusion_node | forecast_node, advisory_node, mission_control, data_logger | 事件（状态联动） | 融合输出：risk + alert + mode + 证据链 |
| `/forecast/alert` | `ForecastAlert` | forecast_node | advisory_node, data_logger | 10min | 预测预警：24h风险序列 + trend |
| `/advisory/action` | `AdvisoryAction` | advisory_node | mission_control, data_logger | 事件 | 农艺建议：action_text + urgency + fungicide |

### 5.3 控制层

| 话题名 | 类型 | 发布者 | 订阅者 | 频率 | 说明 |
|--------|------|--------|--------|------|------|
| `/cmd_vel` | `geometry_msgs/Twist` | Nav2 / mission_control / web_remote | uart_bridge_node | 10-20Hz | 统一底盘速度指令（所有来源统一到此 topic） |
| `/sentry/servo_cmd` | `ServoCmd` | servo_keyboard_node / (未来 mission_control) | servo_driver_node / uart_bridge_node（可选） | 事件 | 云台角度指令；RDK X5 直接 PWM 驱动时 `uart_bridge_node.forward_servo_cmd=False` |
| `/mission/status` | `MissionStatus` | mission_control | data_logger | 10Hz | 巡检状态机状态 |
| `/wheel/odom` | `nav_msgs/Odometry` | wheel_odom_node | EKF, Nav2 | 20Hz | 编码器里程计（dead reckoning） |
| `/odom` | `nav_msgs/Odometry` | ekf_filter | Nav2, TF | 30Hz | EKF 融合后的里程计 |
| `/resume_navigation` | `std_msgs/Bool` | (外部) | mission_control | 事件 | 恢复导航指令 |
| `/set_auto_mode` | `std_srvs/SetBool` | web_remote | mission_control | 事件 | AUTO/MANUAL 模式切换服务 |

### 5.4 fusion_node 核心逻辑

```python
def select_mode(P_vis, env, E_norm, h_risk, t_risk, env_history):
    # ① 视觉绝对主导
    if P_vis >= 0.80:
        return "VISION_DOMINANT"

    # ② 潜伏期预警（冷启动禁用）
    if not env_history.is_cold_boot():
        lwd = env_history.get_lwd_hours()
        if lwd >= LWD_THRESHOLD[crop] and P_vis <= 0.30 and t_risk >= 0.60:
            return "LATENT_SUSPICION"

    # ③ 高湿病原爆发
    hum_threshold = 90 if env_history.is_cold_boot() else 80
    if env.humidity >= hum_threshold and 15 <= env.temperature <= 28 and P_vis >= 0.50:
        return "HIGH_HUMIDITY_PATHOGEN"

    # ④ 干旱胁迫
    if env.humidity <= 40 and env.temperature >= 30:
        return "DROUGHT_STRESS"

    # ⑤ 兜底
    return "BALANCED"
```

**融合公式**：
```
interaction   = P_vis × E_norm
trend_factor  = 1.0 + 0.2 × max(0, humidity_trend_2h)
Risk = w_v·P_vis + w_e·E_norm·trend_factor + w_i·interaction + bias
Risk = clip(Risk, 0.0, 1.0)

agreement = 1.0 - |P_vis - E_norm|
base_confidence = 0.55 + 0.45 × agreement

if COLD_BOOT:   confidence = base_confidence × 0.75
elif WARM_UP:     confidence = base_confidence × 0.90
else:             confidence = base_confidence
```

**报警分级**：
- `CRITICAL`: Risk ≥ 0.80 且 confidence ≥ 0.80（冷启动最多降级为 WARNING）
- `WARNING`: Risk ≥ 0.60
- `SUSPICION`: mode == LATENT_SUSPICION 且 Risk ≥ 0.40
- `NORMAL`: 其余

**环境数据策略**：
- 固定节点为主：LWD 24h滑动窗口（288点，5min间隔），多点取平均
- 移动节点为辅：当前环境快照，stamp > 2s 视为 stale，视觉权重兜底
- 冷启动（0-30min）：LWD不可用，回退瞬时湿度，上限0.70，禁用LATENT_SUSPICION

---

## 6. 关键约束与风险

### 6.1 已接受的简化
- **定位**：2.5m精度GPS，航点级导航，不追求精确沿垄
- **机械结构**：无机械加工，全部采购成品
- **续航/寿命**：原型机阶段不优化
- **网络**：完全离线，RDK仅开AP热点供局域网调试
- **Web前端**：v2.0不做，放到后续版本
- **外部天气**：v2.0先用本地趋势外推，天气接口预留
- **端侧大模型**：v2.0用YAML规则引擎，LLM放到v3.0

### 6.2 当前风险（需跟踪）
| 风险 | 影响 | 缓解措施 |
|------|------|----------|
| **电机驱动电流不足** | TB6612FNG烧毁 | 先用小功率测试，确认电流后决定是否更换 |
| **传感器协议未知** | STM32无法解析 | 尽快向卖家索要UART协议文档 |
| **小麦/草莓模型缺失** | 目前只有番茄模型 | v2.0先写通用框架，模型后续训练补全 |
| **固定节点硬件来不及** | 比赛前LoRa节点做不出来 | Phase 3硬件可延后，Fusion Node先用移动传感器模拟固定节点数据跑通逻辑 |
| **plant_detector模型** | YOLO-Nano训练需数据 | 先下载公开叶片检测数据集预训练，或用MobileNet-SSD替代 |
| **NPU推理延迟** | MobileNetV2实际推理可能 > 500ms | 已量化int8，必要时降分辨率 |
| **LoRa通信可靠性** | 野外丢包、延迟 | 协议层加seq序号和ACK重传，非关键数据允许丢包 |

---

## 7. 目录指引

```
smart-agri-sentry/
├── README.md                          # GitHub风格项目文档
├── CLAUDE.md                          # 开发规范
├── scheme1.md                         # v2.0详细设计方案
├── config/
│   ├── crop_profiles.yaml             # 三作物温度窗口、LWD阈值
│   ├── advisory_rules.yaml            # 农艺建议规则库
│   └── mission_params.yaml            # 巡检参数（速度、阈值等）
├── models/
│   ├── tomatoes_mobilenetv3.bin
│   ├── wheat_mobilenetv3.bin              # 占位
│   ├── strawberry_mobilenetv3.bin         # 占位
│   └── plant_detector_nano.tflite         # 植株检测
├── src/
│   ├── sentry_interfaces/             # ROS2消息定义包
│   │   ├── msg/Diagnosis.msg
│   │   ├── msg/PlantDetection.msg
│   │   ├── msg/Environment.msg
│   │   ├── msg/SoilNutrition.msg
│   │   ├── msg/FusionResult.msg
│   │   ├── msg/ForecastAlert.msg
│   │   ├── msg/AdvisoryAction.msg
│   │   └── msg/MissionStatus.msg
│   ├── sentry_bringup/                # Launch文件
│   ├── sentry_lidar/                  # STL19P 激光雷达驱动（LDLiDAR SDK 迁移）
│   ├── sentry_vision/                 # plant_detector + vision_diagnosis
│   ├── sentry_fusion/                 # fusion_node + lwd_calculator
│   ├── sentry_forecast/               # forecast_node
│   ├── sentry_advisory/               # advisory_node + rule_engine
│   ├── sentry_mission/                # mission_control_node + web_remote_node + wheel_odom_node
│   ├── sentry_sensors/                # env_bridge + uart_bridge + imu
│   ├── sentry_servo/                  # servo_driver_node + servo_keyboard(_node)，RDK X5 直接 PWM 云台驱动
│   ├── sentry_data_logger/            # data_logger_node（rosbag2 录制 + CRITICAL 快照）
│   └── sentry_hardware/               # 固定节点固件
│       └── fixed_env_node/
│           ├── stm32l072_lora/
│           └── lora_gateway/
├── firmware/                          # STM32底盘固件（CubeMX + FreeRTOS）
├── docs/
│   ├── architecture/
│   └── requirements/
├── tests/                             # 单元测试
└── tools/                             # 模型转换等工具
```

---

## 已确认技术细节（2026-04-30）

### 模型
- **番茄**：10类（bacterial_spot, early_blight, healthy, late_blight, leaf_mold, septoria_leaf_spot, spider_mites, target_spot, tomato_mosaic_virus, tomato_yellow_leaf_curl_virus）
- **小麦**：5类（healthy, wheat_powdery_mildew, wheat_scab, wheat_stripe_rust, wheat_yellow_dwarf）
- **草莓**：8类（leaf_spot, powdery_mildew_leaf, gray_mold, angular_leaf_spot, blossom_blight, powdery_mildew_fruit, anthracnose_fruit_rot, healthy）
- **模型路径**：`models/{crop}_mobilenetv3.bin`（RDK X5 上从 ONNX 转换的 bin 格式）；显式传入 `.tflite` 路径会被自动重写为 `.bin`
- **输入尺寸**：224×224
- **植株检测**：YOLO-Nano / MobileNet-SSD，输出 bbox + confidence + area_ratio

### 摄像头（MIPI-CSI）

- **型号**：IMX219，分辨率 1920×1080@30fps
- **ROS2 节点**：`mipi_camera_node`（`sentry_bringup` 包）
- **发布 Topic**：`/sentry/camera/image_raw`，`sensor_msgs/Image`，encoding=`bgr8`
- **启动命令**：`ros2 run sentry_bringup mipi_camera_node`

#### 关键技术约束

1. **`open_cam` 参数顺序**（必须小分辨率在前）：
   ```python
   # 正确 — 512x512 放第一个通道，1920x1080 放第二个
   cam.open_cam(0, -1, -1, [512, 1920], [512, 1080], 1080, 1920)
   
   # 错误 — 第一个通道放 1920x1080 会导致 vp_isp_init 失败 (ret=-10)
   ```
   原因：ISP 固件对第一个输出通道的分辨率有限制，必须小于 sensor 原始分辨率。

2. **`get_img` 通道映射**：
   - `get_img(2, 512, 512)` → 获取**第一个输出通道**（512×512，用于 AI 推理）
   - `get_img(0, 1920, 1080)` → 获取**第二个输出通道**（1920×1080，显示通道，实验性）
   - 当前节点采用 **512×512 取图 + `cv2.resize` 放大到 1920×1080** 的稳妥方案

3. **NV12 转换注意事项**：
   - `get_img` 返回的数据大小可能与请求分辨率不一致（底层 stride 对齐或返回原始分辨率）
   - 必须先判断 `len(img_buf)`，再决定是按 `w*h*1.5` 直接 reshape，还是处理 64-byte / 32-byte stride 对齐
   - 参考：`example/RDK X5 MIPI摄像头+AI检测+MIPI屏幕调试踩坑实录.md`

4. **资源释放**：
   - 节点退出时**必须**调用 `cam.close_cam()`，否则内核层 pipeline 残留会导致下次 `open_cam` 失败
   - 使用 ROS2 标准 `destroy_node()` 生命周期管理，**不要**自定义 `signal.signal(SIGINT)` handler，避免与 ROS2 shutdown 竞态

#### 常见问题排查

| 现象 | 原因 | 解决 |
|------|------|------|
| `vp_isp_init failed, ret(-10)` | `open_cam` 第一个通道分辨率太大 | 改为 `[512, 1920]` / `[512, 1080]` |
| `hbn_vflow_stop failed, ret(-11)` | `close_cam()` 被重复调用 | `destroy_node()` 中加 `self.cam = None` 防重入 |
| `RuntimeError: Context must be initialized...` | `rclpy.shutdown()` 重复执行 | 删除自定义 signal handler，`finally` 中加 `if rclpy.ok():` |
| 画面条纹/花屏 | NV12 按错误分辨率解析 | 根据 `len(img_buf)` 实际大小判断真实分辨率 |
| 节点启动失败，sensor 已识别 | 上次崩溃未释放 MIPI | `sudo reboot` 后再试 |

### LiDAR（STL19P）
- **型号**：STL19P（LDLiDAR），360° 二维激光雷达
- **连接方式**：UART (CP2102 USB转串口)，波特率 230400
- **ROS2 包**：`sentry_lidar`（C++，ament_cmake），从 LDLiDAR SDK 迁移
- **发布话题**：
  - `/scan` (`sensor_msgs/LaserScan`)：标准点云，供导航/避障
  - `/lidar/obstacle_info` (`sentry_interfaces/ObstacleInfo`)：前方扇区简化信息，供融合决策
- **TF**：`base_link` → `laser`，z=0.18m
- **udev 规则**：`99-cp2102-lidar.rules`，创建 `/dev/wheeltec_lidar` 软链接
- **启动命令**：`ros2 launch sentry_lidar stl19p.launch.py`
- **参数配置**：`src/sentry_lidar/config/stl19p.yaml`
  - `product_name`: `LDLiDAR_LD19`
  - `port_name`: `/dev/wheeltec_lidar`
  - `port_baudrate`: `230400`
  - `front_sector_half_angle`: `30.0`（前方扇区半角）
  - `danger_threshold`: `0.5`（障碍物危险阈值，单位 m）
- **前方扇区预处理**：提取 `[360°-half, 360°] ∪ [0, half]` 范围内的点，计算 `front_min_distance`、`front_avg_distance`、`obstacle_detected`
- **驱动协议**：LDLiDAR 私有协议，定长数据包（header `0x54`，ver_len `0x2C`，每包12点），CRC8 校验
- **支持模式**：串口模式（默认）+ 网络模式（UDP/TCP，预留）

### 云台舵机（2-DOF，RDK X5 直接 PWM）
- **硬件**：HiWonder LFD-01M 或同类 180° 舵机 ×2
- **接线**：
  - yaw（水平）→ RDK X5 40pin **Pin 32** → `/sys/class/pwm/pwmchip0/pwm0`
  - pitch（俯仰）→ RDK X5 40pin **Pin 33** → `/sys/class/pwm/pwmchip0/pwm1`
- **PWM 参数**：50 Hz，500–2500 µs 脉宽，对应 0–180°
- **ROS2 包**：`sentry_servo`
  - `servo_driver_node`：订阅 `/sentry/servo_cmd`，写 sysfs PWM
  - `servo_keyboard_node`：键盘 → `/sentry/servo_cmd`
  - `servo_keyboard`：独立脚本，不依赖 ROS2
- **配置**：`src/sentry_servo/config/servo_config.yaml`
  - yaw：channel 0，0–180°，初始 90°，步进 5°
  - pitch：channel 1，30–150°，初始 90°，步进 5°
- **权限**：用户需属于 `gpio` 组；导出后的 sysfs 文件为 `root:gpio rw-rw-r--`
- **避免冲突**：`uart_bridge_node` 默认 `forward_servo_cmd=False`，不再把 `/sentry/servo_cmd` 转发给 STM32

### GPS
- 输出频率：1 Hz（GGA + RMC）

### 底盘传感器
- 下位机 STM32F4 的 TYPE_SENSOR 为 1Hz（100Hz主循环分频发送）
- Fusion Node 消费时以消息 stamp 为准，超过2秒未更新视为 stale

### 固定环境节点
- 硬件：STM32L072 + SX1262 LoRa，低功耗野外版
- 传感器：SHT30（空气温湿）+ SCD40（CO₂）+ RS485土壤（温湿+EC）+ LWS10（叶面湿度）
- 采样：5分钟周期，深度睡眠
- 通信：LoRa → 网关（ESP32-S3）→ USB串口 → RDK X5
- 供电：10W太阳能 + 18650×2

### 固定环境节点土壤传感器
- 可测温湿、N/P/K、EC、pH（但N/P/K/pH分离到SoilNutrition Topic，Fusion不需要）
- 空气传感器可测温湿、CO₂

### 多点策略
- 框架支持多点输入（FIXED_NODE_01/02/...）
- Fusion Node 内部取平均

### Advisory
- v2.0 使用结构化YAML规则引擎
- 端侧大模型后置到v3.0

### 数据存储
- ros2 bag 选择性录制核心topic，7天循环覆盖
- CRITICAL事件前后5分钟永久保留到 `records/critical/`
- InfluxDB + Grafana 放办公室PC，回库后离线分析

### STM32 协议
- 自主设计自定义二进制帧，v2.0 已冻结
- uart_bridge_node 已按此实现

### 融合触发
- 事件驱动：mission_control 状态机进入 STOPPED 后触发视觉推理+融合
- Fusion Node 内部持续运行维护LWD窗口，发布与状态联动

---

## 8. 待确认事项（TODO）

- [ ] 确认24V减速电机的**额定电流和堵转电流**，验证TB6612FNG是否够用
- [ ] 向传感器卖家索要**七合一空气传感器**和**七合一土壤传感器**的UART通信协议文档
- [ ] 确认是否有**24V→5V大功率DC-DC降压模块**（给RDK X5供电，建议≥5A）
- [ ] 确认比赛规则是否**强制要求机械臂/土壤采样动作**
- [ ] **plant_detector 模型训练**：获取叶片/植株检测数据集，训练YOLO-Nano或MobileNet-SSD
- [ ] **小麦/草莓病害模型训练**：收集数据集，训练TFLite模型
- [ ] **LoRa参数确认**：确认频段（433MHz/470MHz）、扩频因子、网关与节点距离
- [ ] **固定环境节点外壳**：确认IP65防水盒尺寸、太阳能板安装方式

---

## 9. 实施阶段（比赛导向）

| 阶段 | 目标 | 产出 |
|------|------|------|
| **Phase 1** | 消息接口 + plant_detector + vision_diagnosis + fusion + mission_control | **最小可用产品**：小车能停-拍-判-走，Fusion能出风险 |
| **Phase 2** | forecast + advisory + data_logger | **已完成**：三个节点已落地，`sentry_v2.launch.py` 已注册，RDK X5 通过 24 tests |
| **Phase 3** | 固定环境节点硬件固件 + env_bridge | 24h LWD真正跑起来（硬件来不及可先用模拟数据跑通逻辑） |
| **Phase 4** | 外部天气 + Web前端 + InfluxDB离线分析 | 后续完善 |

---

> **使用建议**：将此文件放在 `.claude/PROJECT_CONTEXT.md`，与Claude Code对话时直接 `@.claude/PROJECT_CONTEXT.md`，Claude会基于这些约束生成符合你硬件现实的代码。
