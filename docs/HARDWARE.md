# 硬件平台

> 更新日期：2026-06-30

---

## 1. 主控与运动

| 模块 | 型号/规格 | 备注 |
|---|---|---|
| **AI 主控** | RDK X5 | 8 核 A55, 8 GB LPDDR4, 旭日 R5 NPU (10 TOPS), 功耗约 3 W |
| **运动控制** | STM32F407ZGT6 | 最小系统板, 168 MHz, 裸机主循环；仅负责底盘运动控制，不转发环境传感器 |
| **电机** | MG540 直流减速电机 ×2 | 13 PPR 霍尔编码器 |
| **电机驱动** | 当前测试驱动板 | 与 STM32 直连 |
| **编码器** | 13 PPR，减速比 1:30，四倍频 | 每米 11035 脉冲 |
| **底盘** | 履带式 | 轮距 0.23 m |
| **云台** | 2-DOF 舵机云台（采购成品） | 控制摄像头俯仰/偏航；RDK X5 直接 PWM 驱动 |
| **运行速度** | **0.5 m/s（典型工况）** | 巡航速度 |

---

## 2. 传感器与连接方式

### 2.1 RDK X5 直连传感器

| 传感器 | 连接方式 | 接至 | 数据项 | 周期 | ROS2 Topic |
|---|---|---|---|---|---|
| **MIPI 摄像头** | CSI | RDK X5 | 图像/视频流 | 500 ms | `/sentry/camera/image_raw` |
| **激光雷达** | UART (CP2102) | RDK X5 | 360° 距离/强度点云 | 10 Hz | `/scan`, `/lidar/obstacle_info` |
| **IMU** | USB (CH340) | RDK X5 | 姿态/角速度/加速度 | 100 Hz | `/sensor/imu/data_raw`, `/sensor/imu/data` |

### 2.2 固定环境节点（田间 24 h 连续，低功耗野外版）

> 环境数据全部由固定节点采集并通过 LoRa 回传，底盘 STM32F407 不再转发移动传感器。
> 固件源码见 `test/stm32_cj702_lora_hal/`（STM32F103RCT6，HAL 库，Makefile + MDK-ARM 双构建）。

| 传感器 | 型号 | 接口 | STM32 引脚 | 数据项 | 采样周期 | 备注 |
|---|---|---|---|---|---|---|
| **空气七合一** | CJ702 | UART3 (TTL, 9600bps) | PB11 TX / PB10 RX | CO₂, HCHO, TVOC, PM2.5, PM10, 温度, 湿度 | 1 s（每秒推帧） | 固定 17 字节协议帧，含校验和 |
| **叶面温湿度** | RS485 ModBus | UART1 (RS485, 4800bps) | PA9 TX / PA10 RX | 叶面温度、叶面湿度 | 每 CJ702 帧触发 | 地址 0x01，03 功能码读 2 寄存器 |
| **土壤七合一** | TTL ModBus | UART4 (TTL, 9600bps) | PC10 TX / PC11 RX | 温度、湿度、EC、pH、N、P、K、盐分 | 每 CJ702 帧触发 | 地址自动探针 0x01→0x02→0x03，03 功能码读 8 寄存器 |

**固定节点通信链**：
- 节点端：STM32F103RCT6 采集三传感器 → UART3→CJ702 / UART1→叶面 RS485 / UART4→土壤 TTL → 秒级聚合（60s 窗口取平均）→ UART2 → E22-400TBH-SC（内置 STM32F103CBT6）→ SX1262 LoRa 发送
- 网关端：E22-400TBH-SC（内置 STM32F103CBT6）接收 → UART → RDK X5（USB 转串口）→ `env_bridge_node` → `/sensor/environment_fixed`
- 固件构建：`make -j`（GCC ARM Embedded）或 Keil MDK-ARM

---

## 3. 通信与电源

| 项目 | 方案 |
|---|---|
| **RDK ↔ STM32** | UART2（Pin 8 TX / Pin 10 RX，交叉连接），波特率 115200，自定义二进制帧；仅传输底盘运动控制与状态 |
| **RDK ↔ LoRa 网关** | USB 转串口（TTL），波特率 115200 |
| **固定节点供电** | 10 W 太阳能 + 18650×2（并联 4000 mAh），CN3791 MPPT |
| **主电源** | 24 V 锂电池组 |
| **降压分配** | 24 V→5 V/3.3 V DC-DC 给 RDK、STM32、传感器；24 V 直驱电机 |

---

## 4. 裸机主循环周期

| 任务 | 周期 | 频率 | 说明 |
|---|---|---|---|
| Protocol_Process | 主循环 | ~1000 Hz | 处理 USART2 RDK X5 RX 帧 |
| PID 闭环 | 10 ms | 100 Hz | 编码器反馈 → PID → 电机 PWM |
| 底盘状态上报 | 100 ms | 10 Hz | 发送 project-standard 底盘状态帧 |
| 心跳 LED | 500 ms | 2 Hz | PA8 心跳指示灯 |
| 状态打印 | 5000 ms | 0.2 Hz | USART1 调试控制台摘要 |
| 看门狗刷新 | 100 ms | 10 Hz | IWDG 喂狗 |

---

## 5. 通信协议定义

### 5.1 STM32 ↔ RDK X5（UART2，115200，3.3 V TTL）

接线（交叉）：
- STM32 PA2 (USART2_TX) → RDK 40Pin **10** (UART2_RXD)
- STM32 PA3 (USART2_RX) → RDK 40Pin **8** (UART2_TXD)
- GND 共地

RDK 设备节点：`/dev/ttyS1`（UART2 对应 `34070000.serial`，物理 40Pin Pin8 TX / Pin10 RX）。

采用**自定义二进制帧**：

```
[帧头 2B] [类型 1B] [长度 1B] [载荷 nB] [CRC16-CCITT 2B]
0xAA 0x55   TYPE     LEN      DATA       CRC16
```

#### 数据类型（TYPE）

> **注意**：`TYPE=0x01` 传感器汇总帧（空气温湿度 CO₂ + 土壤电导率/氮磷钾/温湿度/pH）已随移动传感器移除而弃用，保留在旧版协议中仅作兼容性参考。

| TYPE | 方向 | 含义 | 载荷内容 |
|---|---|---|---|
| `0x03` | STM32→RDK | **底盘状态帧** | 左轮速、右轮速、电池电压、报警位、编码器脉冲(L/R)、时间戳 |
| `0x81` | RDK→STM32 | **运动控制帧** | 左轮目标速、右轮目标速（mm/s） |
| `0x82` | RDK→STM32 | **云台控制帧**（可选） | 舵机俯仰角、偏航角（角度值）；仅当 `uart_bridge_node.forward_servo_cmd=True` 时转发 |
| `0x83` | RDK→STM32 | **模式切换帧** | 0x00=待机, 0x01=遥控, 0x02=自动巡航 |

#### 底盘状态帧（TYPE=0x03）载荷定义（v2.1 扩展）

```c
typedef struct {
    int16_t  left_speed_x1000;   // mm/s
    int16_t  right_speed_x1000;  // mm/s
    int16_t  battery_voltage_x100; // 0.01 V
    uint8_t  alarm_bits;          // 报警位
    int32_t  left_pulse;          // 左轮编码器累计脉冲（有符号）
    int32_t  right_pulse;         // 右轮编码器累计脉冲（有符号）
    uint32_t encoder_timestamp;   // STM32 时间戳 ms
} __attribute__((packed)) ChassisStatusFrame_t;
// 总长度：2+1+1+19+2 = 25 字节
```

> 注意：`left_pulse` / `right_pulse` 为有符号 int32，倒车时数值为负。

#### CRC 校验

- 算法：**CRC16-CCITT** (`0x1021`)
- 范围：从 `类型` 字节到 `载荷` 末尾
- 初始值：`0xFFFF`

### 5.2 固定环境节点 ↔ LoRa 网关（LoRa，433 MHz/470 MHz）

**帧格式**（自定义二进制帧，与 RDK↔STM32 协议同源）：

```
[0xAA] [0x55] [device_id 1B] [msg_type 1B] [payload_len 1B] [payload N bytes] [CRC8 Maxim 1B]
```

- `msg_type=0x01`：数据帧，payload 为 24 字节 sensor payload（12 个 big-endian int16 字段：CO₂/HCHO/TVOC/PM2.5/PM10/air_temp/air_hum/soil_temp/soil_hum/EC/leaf_wetness/leaf_temp）
- `msg_type=0xFF`：错误帧，payload 为 1 字节 error_code（`0x01`=timeout, `0x02`=incomplete）
- CRC8 Maxim：多项式 `0x31`，校验范围从 header 到 payload 末尾

**节点端（STM32F103RCT6）**：

- `app_fsm` 状态机：`ACCUMULATE`（60s 窗口采样聚合）→ `TX`（打包发送）→ `WAIT`
- 每 60s 发送一帧数据（含 60 个样本的均值）或错误帧
- 当前未实现低功耗睡眠——采样期间持续运行
- 硬件串口分配：UART1=叶面 RS485 / UART2=LoRa 模块 / UART3=CJ702 空气 / UART4=土壤 TTL
- 提供 OpenOCD 可读调试变量（`g_dbg_*`），覆盖所有传感器原始值、状态位、土壤探针地址
- 源码位置：`test/stm32_cj702_lora_hal/`

**网关端（STM32F103CBT6 / E22-400TBH-SC 内置）**：
- 接收 LoRa 数据包，通过 USB 转串口透明转发给 RDK X5
- 转发内容为原始二进制帧，由 RDK X5 的 `lora_bridge_node` 解析

**RDK X5 端（lora_bridge_node）**：
- 解析串口二进制帧，转换为 `Environment` 消息（含 air_temp/humidity/CO₂/soil_temp/soil_humidity/EC/leaf_wetness/leaf_temp 及土壤 NPK/pH 扩展字段）
- `data_source` 字段设为 `FIXED_NODE_01` / `FIXED_NODE_02` / ...

---

## 6. LiDAR（STL19P）

- **型号**：STL19P（LDLiDAR），360° 二维激光雷达
- **连接方式**：UART (CP2102 USB 转串口)，波特率 230400
- **设备节点**：`/dev/wheeltec_lidar`（udev 规则 `99-cp2102-lidar.rules`）
- **ROS2 包**：`sentry_lidar`（C++，ament_cmake）
- **发布话题**：
  - `/scan` (`sensor_msgs/LaserScan`)：标准点云，供导航/避障
  - `/lidar/obstacle_info` (`sentry_interfaces/ObstacleInfo`)：前方扇区简化信息，供融合决策
- **TF**：`base_link` → `laser`，z = 0.18 m
- **参数配置**：`src/sentry_lidar/config/stl19p.yaml`
  - `product_name`: `LDLiDAR_LD19`
  - `port_name`: `/dev/wheeltec_lidar`
  - `port_baudrate`: `230400`
  - `front_sector_half_angle`: `30.0`（前方扇区半角）
  - `danger_threshold`: `0.5`（障碍物危险阈值，单位 m）
- **前方扇区预处理**：提取 `[360°-half, 360°] ∪ [0, half]` 范围内的点，计算 `front_min_distance`、`front_avg_distance`、`obstacle_detected`
- **驱动协议**：LDLiDAR 私有协议，定长数据包（header `0x54`，ver_len `0x2C`，每包 12 点），CRC8 校验

---

## 7. 云台舵机（2-DOF，RDK X5 直接 PWM）

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

---

## 8. MIPI 摄像头（IMX219）

- **型号**：IMX219，分辨率 1920×1080@30 fps
- **ROS2 节点**：`mipi_camera_node`（`sentry_bringup` 包）
- **发布 Topic**：`/sentry/camera/image_raw`，`sensor_msgs/Image`，encoding=`bgr8`
- **启动命令**：`ros2 run sentry_bringup mipi_camera_node`

### 关键技术约束

1. **`open_cam` 参数顺序**（必须小分辨率在前）：
   ```python
   # 正确 — 512x512 放第一个通道，1920x1080 放第二个
   cam.open_cam(0, -1, -1, [512, 1920], [512, 1080], 1080, 1920)
   ```

2. **`get_img` 通道映射**：
   - `get_img(2, 512, 512)` → 获取**第一个输出通道**（512×512，用于 AI 推理）
   - `get_img(0, 1920, 1080)` → 获取**第二个输出通道**（1920×1080，显示通道）

3. **资源释放**：节点退出时**必须**调用 `cam.close_cam()`，否则内核层 pipeline 残留会导致下次 `open_cam` 失败。

### 常见问题排查

| 现象 | 原因 | 解决 |
|---|---|---|
| `vp_isp_init failed, ret(-10)` | `open_cam` 第一个通道分辨率太大 | 改为 `[512, 1920]` / `[512, 1080]` |
| `hbn_vflow_stop failed, ret(-11)` | `close_cam()` 被重复调用 | `destroy_node()` 中加 `self.cam = None` 防重入 |
| 画面条纹/花屏 | NV12 按错误分辨率解析 | 根据 `len(img_buf)` 实际大小判断真实分辨率 |
| 节点启动失败，sensor 已识别 | 上次崩溃未释放 MIPI | `sudo reboot` 后再试 |

---

## 9. IMU（YB-IMU）

- **硬件**：YB-IMU，CH340 USB 转串口
- **设备节点**：`/dev/myimu`
- **数据流**：`imu_node` → `/sensor/imu/data_raw` → `imu_filter_madgwick` → `/sensor/imu/data`
- **关键配置**：Madgwick `publish_tf: false`，由 EKF 发布 `odom → base_link` TF
- **频率**：100 Hz

---

## 10. 固定环境节点

- **主控**：STM32F103RCT6（采集传感器、运行低功耗逻辑）
- **LoRa 模块**：E22-400TBH-SC（内置 STM32F103CBT6，负责 LoRa 收发）
- **主控 ↔ LoRa 模块**：UART（RCT6 发送数据给模块内置 CBT6，CBT6 通过 SX1262 发出）
- **传感器**：SHT30（空气温湿）+ SCD40（CO₂）+ RS485 土壤（温湿+EC）+ LWS10（叶面湿度）
- **采样**：5 分钟周期，深度睡眠
- **供电**：10 W 太阳能 + 18650×2

**RDK 侧接收链**：
- 网关端 E22-400TBH-SC（内置 CBT6）接收到 LoRa 数据后，通过 UART 经 USB 转串口发送给 RDK X5
- RDK X5 上 `env_bridge_node` 解析后发布 `/sensor/environment_fixed`

---

## 11. 已移除硬件

### GPS 模块（已弃用）

项目不再使用 GPS 模块。遗留代码中的 `gps_node` 仅存在于旧版启动文件 `sentry_bringup/launch/sentry.launch.py` 中，未在 `sentry_v2.launch.py` 中使用。
