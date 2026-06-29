# 底盘控制项目集成设计

> 日期：2026-06-29  
> 范围：把 `test/Intelligent_Agricultural_Robot_Car/` 提升为项目正式底盘固件  
> 状态：设计已确认，待实现

---

## 1. 背景与目标

`test/Intelligent_Agricultural_Robot_Car/` 是一套已验证的 STM32F407ZGT6 底盘运动控制代码，具备：

- 双路直流电机 PWM + H 桥方向控制
- 双路编码器测速 + 累计脉冲里程
- 100 Hz 位置式 PID 速度闭环
- 与 RDK X5 的自定义串口协议
- USART1 调试命令行、电池检测、看门狗

本设计目标是把该测试项目**集成进主项目**，成为 `firmware/chassis/` 下的正式固件，并使其协议、话题接口与项目现有 ROS2 架构对齐。

---

## 2. 关键决策

| 决策项 | 选择 | 说明 |
|--------|------|------|
| 硬件基线 | 现有测试硬件 | MG540 电机 + 13 PPR 编码器，轮距 0.23 m，每米 11035 脉冲 |
| 项目位置 | `firmware/chassis/` | 正式固件目录；`test/Intelligent_Agricultural_Robot_Car/` 保留为历史参考 |
| RTOS | 裸机主循环 | 本次只负责底盘控制，不上环境传感器；用时间片轮询即可 |
| 通信协议 | 项目规范协议 | `0xAA 0x55 + TYPE + LEN + DATA + CRC16-CCITT` |
| 速度输入 | `/cmd_vel` | RDK 侧 `uart_bridge_node` 订阅，转成左右轮 mm/s 下发 |
| 速度输出 | `/sentry/chassis/status` | 上传 m/s、累计脉冲、电池电压、报警位、时间戳 |
| 协议发送 | DMA | 避免阻塞主循环，保证 PID 实时性 |

---

## 3. 目录结构

```text
firmware/
└── chassis/
    ├── Core/                      # CubeMX 生成的 HAL 代码（迁移）
    │   ├── Inc/
    │   └── Src/
    ├── Drivers/                   # STM32F4xx HAL + CMSIS
    ├── Users/                     # 应用层 BSP
    │   ├── bsp_motor.*            # 电机 PWM + 方向（不变）
    │   ├── bsp_encoder.*          # 编码器读取 + 累积脉冲（不变）
    │   ├── pid.*                  # PID + 摩擦自适应（不变）
    │   ├── bsp_adc.*              # 电池检测（不变）
    │   ├── bsp_debug.*            # USART1 命令行（小改）
    │   ├── bsp_diag.*             # 诊断 LED（不变）
    │   └── bsp_protocol.*         # 重写：规范协议 + CRC16
    ├── Tests/
    │   └── host/
    │       └── test_protocol.py   # 主机端协议离线测试
    ├── MDK-ARM/                   # Keil 工程
    ├── .mxproject
    ├── Intelligent_Agricultural_Robot_Car.ioc  # 可后续改名为 chassis.ioc
    └── README.md                  # 新增：接线、编译、协议说明
```

---

## 4. 通信协议

### 4.1 帧格式

```text
[0xAA] [0x55] [TYPE] [LEN] [DATA ...] [CRC16-CCITT]
```

- CRC 范围：TYPE 字节到 DATA 末尾（含 TYPE、LEN、DATA）
- CRC16-CCITT：多项式 `0x1021`，初始值 `0xFFFF`

### 4.2 RDK → STM32：运动控制帧（TYPE = 0x81）

```c
typedef struct __attribute__((packed)) {
    int16_t left_speed_mm_s;   // 左轮目标速度，mm/s
    int16_t right_speed_mm_s;  // 右轮目标速度，mm/s
} MotionCmd_t;  // LEN = 4
```

STM32 收到后转换为内部单位 pulses/10ms：

```c
target_speed_left  = left_speed_mm_s  * 11035.0f / 100000.0f;
target_speed_right = right_speed_mm_s * 11035.0f / 100000.0f;
```

> 当前硬件左右电机通道存在物理交叉（电机驱动线与编码器线在 PCB 上左右对调）。协议层保持“上位机视角：左=左、右=右”，在 `bsp_protocol.c` 收到 0x81 帧时做左右交换赋值给 PID；在发送 0x03 帧时再做一次交换，使上位机收到正确的左右分配。

### 4.3 STM32 → RDK：底盘状态帧（TYPE = 0x03）

```c
typedef struct __attribute__((packed)) {
    float    left_speed;        // 左轮速度，m/s
    float    right_speed;       // 右轮速度，m/s
    float    battery_voltage;   // 电池电压，V
    uint8_t  alarm_bits;        // 报警位
    int32_t  left_pulse;        // 左轮编码器累计脉冲（有符号，可正可负）
    int32_t  right_pulse;       // 右轮编码器累计脉冲（有符号，可正可负）
    uint32_t timestamp_ms;      // STM32 开机毫秒
} ChassisStatusFrame_t;  // LEN = 19，整帧 25 字节
```

> `sentry_interfaces/msg/ChassisStatus.msg` 当前 `left_pulse/right_pulse` 为 `uint32`，需同步改为 `int32` 以支持倒车累计脉冲为负。

```c
left_speed_m_s  = speed_left  * 0.14137167f / 1560.0f * 100.0f;
right_speed_m_s = speed_right * 0.14137167f / 1560.0f * 100.0f;
```

### 4.4 报警位定义

```c
#define CHASSIS_ALARM_NONE          0x00
#define CHASSIS_ALARM_LOW_BATTERY   0x01  // 电池低压（后续实现）
#define CHASSIS_ALARM_MOTOR_STALL   0x02  // 电机堵转（后续实现）
#define CHASSIS_ALARM_COMM_ERROR    0x04  // 通信 CRC 连续错误（本次实现）
#define CHASSIS_ALARM_WATCHDOG      0x08  // 保留
```

---

## 5. STM32 主循环

保持裸机时间片轮询，周期如下：

| 任务 | 周期 | 说明 |
|------|------|------|
| 协议接收解析 | 轮询 | `Protocol_Process()` 解析 IDLE+DMA 收到的帧 |
| PID 闭环 | 10 ms | 编码器 → PID → PWM |
| 协议发送 | 100 ms | DMA 发送底盘状态帧 |
| 心跳 LED | 500 ms | PA8 翻转 |
| 状态打印 | 5000 ms | USART1 调试输出 |
| 看门狗刷新 | 100 ms | IWDG |

---

## 6. RDK X5 侧改动

### 6.1 `uart_bridge_node`

- **订阅**：`/cmd_vel`（`geometry_msgs/msg/Twist`）
- **转换**：差速模型，轮距 `0.23 m`

```python
left_speed_m_s  = msg.linear.x - msg.angular.z * wheel_base / 2.0
right_speed_m_s = msg.linear.x + msg.angular.z * wheel_base / 2.0
left_mm_s  = int(left_speed_m_s  * 1000)
right_mm_s = int(right_speed_m_s * 1000)
```

- **下发**：打包 TYPE=0x81 帧，通过 UART2 发送给 STM32
- **解析**：接收 TYPE=0x03 帧，发布 `/sentry/chassis/status`
- **错误处理**：
  - STM32 端：连续 CRC 错误时设置 `alarm_bits |= CHASSIS_ALARM_COMM_ERROR`
  - RDK 端：超过 1 秒未收到帧时，在发布的 `ChassisStatus` 中置自定义超时标志（或日志告警），不覆盖 STM32 传来的 `alarm_bits`

### 6.2 `wheel_odom_node`

已存在：`src/sentry_mission/sentry_mission/wheel_odom_node.py`

只需调整默认参数：

| 参数 | 当前默认值 | 新默认值 |
|------|-----------|----------|
| `wheel_base` | 0.4 | **0.23** |
| `pulses_per_meter` | 1000 | **11035** |

---

## 7. 数据流

```text
┌─────────────────────────────────────────────────────────────┐
│ RDK X5                                                      │
│  Nav2 / web_remote / mission_control                        │
│         │                                                   │
│         ▼                                                   │
│  /cmd_vel (geometry_msgs/Twist)                             │
│         │                                                   │
│         ▼                                                   │
│  uart_bridge_node ──UART2──► STM32                          │
│         ▲                      │                            │
│         │                      ▼                            │
│  /sentry/chassis/status   bsp_protocol                      │
│         │                      │                            │
│         ▼                      ▼                            │
│  wheel_odom_node ◄──── encoder + PID + motor               │
│         │                                                   │
│         ▼                                                   │
│  /wheel/odom ──► ekf_filter ──► /odom                       │
└─────────────────────────────────────────────────────────────┘
```

---

## 8. 测试计划

| 阶段 | 测试内容 | 通过标准 |
|------|----------|----------|
| 离线协议 | `Tests/host/test_protocol.py` 构造/解析 0x81、0x03 帧 | CRC 正确，字段解析无误 |
| STM32 单机 | USART1 命令 `L100`、`R100`、`STOP`、`STATUS` | 电机按指令转动/停止，状态打印正确 |
| 协议连通 | RDK `ros2 topic echo /sentry/chassis/status` | 10 Hz 收到数据，累积脉冲随车轮转动增长 |
| 闭环控制 | `ros2 topic pub /cmd_vel ...` 发 0.2 m/s | 车直线前进，状态帧速度回显接近 0.2 m/s |
| 里程计校准 | 让车实际走 1 m | `/wheel/odom` 位移接近 1 m，必要时微调 `pulses_per_meter` |

---

## 9. 文档更新

- `docs/HARDWARE.md`
  - 电机/编码器型号改为 MG540 + 13 PPR
  - 轮距改为 0.23 m
  - 每米脉冲改为 11035
  - FreeRTOS 任务表改为裸机时间片表
- `docs/ROS2.md`
  - 更新 `/sentry/chassis/status` 字段说明
  - 明确 `/cmd_vel` 作为底盘输入
  - `wheel_odom_node` 参数说明
- 新增 `firmware/chassis/README.md`
  - 接线图、CubeMX/Keil 编译、烧录步骤、协议速查

---

## 10. 风险与回退

| 风险 | 缓解措施 |
|------|----------|
| 协议解析不一致 | 先用主机脚本离线验证帧格式，再联调 |
| 单位切换导致车速偏差 | 用调试命令分别验证 pulses/10ms 与 mm/s 换算 |
| DMA 发送异常 | 保留阻塞发送代码分支，可快速 `#if` 切换回退 |
| 左右方向反 | 低速单轮给速测试，必要时在 `bsp_protocol.c` 或 `bsp_motor.c` 调整交换 |

---

## 11. 后续可扩展（不在本次范围）

- 增加空气/土壤传感器，届时再评估是否切 FreeRTOS
- 报警位逐步扩展：低压检测、电机堵转、通信超时
- 速度平滑 / 加速度限制
- LiDAR SLAM 建图导航对接
