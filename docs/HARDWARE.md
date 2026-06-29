# 硬件平台

> 更新日期：2026-06-25

---

## 1. 主控与运动

| 模块 | 型号/规格 | 备注 |
|---|---|---|
| **AI 主控** | RDK X5 | 8 核 A55, 8 GB LPDDR4, 旭日 R5 NPU (10 TOPS), 功耗约 3 W |
| **运动控制** | STM32F407ZGT6 | 最小系统板, 168 MHz, 裸机主循环 |
| **电机** | MG540 直流减速电机 ×2 | 13 PPR 霍尔编码器 |
| **电机驱动** | 当前测试驱动板 | 与 STM32 直连 |
| **编码器** | 13 PPR，减速比 1:30，四倍频 | 每米 11035 脉冲 |
| **底盘** | 履带式 | 轮距 0.23 m |
| **云台** | 2-DOF 舵机云台（采购成品） | 控制摄像头俯仰/偏航；RDK X5 直接 PWM 驱动 |
| **运行速度** | **0.5 m/s（典型工况）** | 巡航速度 |

---

## 2. 传感器与连接方式

### 2.1 移动传感器（随车，经 STM32 转发）

| 传感器 | 连接方式 | 接至 | 数据项 | 周期 | ROS2 Topic |
|---|---|---|---|---|---|
| **七合一空气质量** | UART | STM32 | 温度、湿度、CO₂ | 1 s | `/sensor/environment_mobile` |
| **七合一土壤** | UART | STM32 | 电导率、氮、磷、钾、温度、湿度、pH | 1 s | `/sensor/soil_nutrition` |
| **MIPI 摄像头** | CSI | RDK X5 | 图像/视频流 | 500 ms | `/sentry/camera/image_raw` |
| **激光雷达** | UART (CP2102) | RDK X5 | 360° 距离/强度点云 | 10 Hz | `/scan`, `/lidar/obstacle_info` |
| **IMU** | USB (CH340) | RDK X5 | 姿态/角速度/加速度 | 100 Hz | `/sensor/imu/data_raw`, `/sensor/imu/data` |

### 2.2 固定环境节点（田间 24 h 连续，低功耗野外版）

| 传感器 | 型号 | 接口 | 数据项 | 采样周期 | 备注 |
|---|---|---|---|---|---|
| **空气温湿** | SHT30 | I²C (0x44) | 温度、湿度 | 5 min | 百叶箱内，冠层中部 |
| **CO₂** | SCD40 | I²C (0x62) | CO₂ | 5 min | 同百叶箱 |
| **土壤参数** | RS485 三合一 | UART→RS485 | 温度、湿度、EC | 5 min | 根区 10–15 cm |
| **叶面湿度** | LWS10 | ADC 模拟 | 叶面湿度 | 5 min | 代表性叶片背面 |

**固定节点通信链**：
- 节点端：STM32F103RCT6 采集传感器 → UART → E22-400TBH-SC 内置 STM32F103CBT6 → SX1262 LoRa 发送
- 网关端：E22-400TBH-SC（内置 STM32F103CBT6）接收 → UART → RDK X5（USB 转串口）→ `env_bridge_node` → `/sensor/environment_fixed`

---

## 3. 通信与电源

| 项目 | 方案 |
|---|---|
| **RDK ↔ STM32** | UART2（Pin 15 TX / Pin 22 RX，交叉连接），波特率 115200，自定义二进制帧 |
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
- STM32 PA2 (USART2_TX) → RDK 40Pin **22** (UART2_RXD)
- STM32 PA3 (USART2_RX) → RDK 40Pin **15** (UART2_TXD)
- GND 共地

RDK 设备节点：`/dev/ttyS5`（UART2 对应 `341a0000.serial`，物理 40Pin Pin15 TX / Pin22 RX）。

采用**自定义二进制帧**：

```
[帧头 2B] [类型 1B] [长度 1B] [载荷 nB] [CRC16-CCITT 2B]
0xAA 0x55   TYPE     LEN      DATA       CRC16
```

#### 数据类型（TYPE）

| TYPE | 方向 | 含义 | 载荷内容 |
|---|---|---|---|
| `0x01` | STM32→RDK | **传感器汇总帧** | 空气温湿度 CO₂ + 土壤电导率/氮磷钾/温湿度/pH |
| `0x03` | STM32→RDK | **底盘状态帧** | 左轮速、右轮速、电池电压、报警位、编码器脉冲(L/R)、时间戳 |
| `0x81` | RDK→STM32 | **运动控制帧** | 左轮目标速、右轮目标速（mm/s） |
| `0x82` | RDK→STM32 | **云台控制帧**（可选） | 舵机俯仰角、偏航角（角度值）；仅当 `uart_bridge_node.forward_servo_cmd=True` 时转发 |
| `0x83` | RDK→STM32 | **模式切换帧** | 0x00=待机, 0x01=遥控, 0x02=自动巡航 |

#### 传感器汇总帧（TYPE=0x01）载荷定义

```c
typedef struct {
    uint32_t timestamp_ms;      // STM32 开机后的毫秒时间戳
    int16_t  air_temp_x10;      // 空气温度 ×10（0.1 ℃）
    uint16_t air_humi_x10;      // 空气湿度 ×10（0.1 %RH）
    uint16_t air_co2;           // CO₂ 浓度（ppm）
    int16_t  soil_temp_x10;     // 土壤温度 ×10（0.1 ℃）
    uint16_t soil_humi_x10;     // 土壤湿度 ×10（0.1 %RH）
    uint16_t soil_ec;           // 土壤电导率（us/cm）
    uint16_t soil_n;            // 氮含量（mg/kg）
    uint16_t soil_p;            // 磷含量（mg/kg）
    uint16_t soil_k;            // 钾含量（mg/kg）
    uint16_t soil_ph_x10;       // pH 值 ×10（0.1 pH）
} __attribute__((packed)) SensorFrame_t;
// 总长度：2+1+1+24+2 = 30 字节
```

`uart_bridge_node` 解析 TYPE=0x01 后，拆分为两个 ROS2 消息发布：
- `/sensor/environment_mobile` (`Environment` 消息)：空气温湿/CO₂、土壤温湿/EC
- `/sensor/soil_nutrition` (`SoilNutrition` 消息)：N/P/K/pH/EC

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

**节点端（STM32F103RCT6）**：

- 深度睡眠，每 5 分钟唤醒采集一次
- 每小时批量发送 12 条记录，或异常时立即上报
- 数据包格式（JSON 简化）：`{"node_id":"01","t":23.5,"h":78.0,"co2":450,"st":22.1,"sh":65.0,"ec":1.2,"lw":0,"seq":123}`

**网关端（STM32F103CBT6）**：
- 接收 LoRa 数据包，通过 USB 转串口转发给 RDK X5
- 转发格式：JSON + `\n` 换行分隔

**RDK X5 端（env_bridge_node）**：
- 解析串口 JSON，转换为 `Environment` 消息
- `data_source` 字段设为 `FIXED_NODE_01` / `FIXED_NODE_02` / ...
- 支持多点，Fusion Node 内部取平均

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
