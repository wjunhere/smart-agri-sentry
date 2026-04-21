# 智农哨兵 - 项目上下文（PROJECT_CONTEXT）

> 文档用途：供 Claude Code / 团队成员快速理解项目约束、接口定义和技术栈
> 更新日期：2026-04-21
> 适用场景：嵌入式比赛项目，原型样机阶段
> **架构版本：v2.0 混合架构（GPS直连 + 传感器经STM32转发）**

---

## 版本变更记录

### v2.0 → 本次更新（2026-04-21）
1. **架构变更**：由"全部传感器经STM32转发"改为**混合架构**
   - GPS（G60）直连 RDK X5 UART6，由 ROS2 `gps_node` 直接解析 NMEA
   - 七合一空气传感器 + 七合一土壤传感器仍由 STM32 采集，经 UART2 打包转发给 RDK
   - STM32 与 RDK 的通信帧中**移除 GPS 相关字段**，仅包含传感器 + 底盘状态 + 控制指令
2. **数据流重构**：由"同步等待"改为**异步订阅 + 时间戳对齐**
   - 各节点独立运行，通过 ROS2 Topic 松耦合
   - `fusion_node` 以 AI 推理结果到达为触发事件，传感器数据作为上下文缓存
   - 加入超时降级机制：传感器失联时，AI 单模态继续工作
3. **时间周期明确**：定义了全链路任务周期和时序约束
4. **运动学基准**：明确底盘速度 **0.5 m/s**，据此计算数据时空误差

---

## 1. 项目概述

- **名称**：智农哨兵
- **性质**：嵌入式比赛项目，三人团队，无机械加工条件
- **核心目标**：在无网农田环境下，实现"底盘遥控/自动巡航 + 传感器采集 + 端侧AI病害识别 + 本地Web展示"
- **非目标**：不追求续航、不追求元器件寿命、不追求工业级防护

---

## 2. 硬件平台

### 2.1 主控与运动
| 模块 | 型号/规格 | 备注 |
|------|-----------|------|
| **AI主控** | RDK X5 | 8核A55, 8GB LPDDR4, 旭日R5 NPU (10 TOPS), 功耗~3W |
| **运动控制** | STM32F407ZGT6 | 最小系统板, 168MHz, FreeRTOS |
| **电机** | 24V 直流减速电机 ×2 | 需确认空载/堵转电流，以验证驱动器选型 |
| **电机驱动** | TB6612FNG（待验证） | 持续电流1.2A；若电机电流过大，更换为BTN7971B |
| **编码器** | 1000线光电编码器 ×2 | 接STM32定时器正交编码器输入 |
| **底盘** | 履带式（采购成品） | 无机械加工条件，直接采购橡胶履带底盘套件 |
| **云台** | 2-DOF舵机云台（采购成品） | 控制摄像头俯仰/偏航 |
| **运行速度** | **0.5 m/s（典型工况）** | 据此计算各周期下的物理位移 |

### 2.2 传感器与连接方式（混合架构）
| 传感器 | 连接方式 | 接至 | 数据项 | 周期 | 关键待确认 |
|--------|----------|------|--------|------|------------|
| **七合一空气质量** | UART | **STM32** | 温度、湿度、CO₂ | 100ms | 波特率、数据帧格式 |
| **七合一土壤** | UART | **STM32** | 电导率、氮、磷、钾、温度、湿度、pH | 100ms | 同上 |
| **GPS北斗双模** | UART | **RDK X5（直连）** | 经纬度、速度、航向 | 100ms | G60模块，2.5m精度 |
| **MIPI摄像头** | CSI | RDK X5 | 图像/视频流 | 500ms | 确认RDK支持的摄像头型号 |

### 2.3 通信与电源
| 项目 | 方案 |
|------|------|
| **RDK ↔ STM32** | UART2（Pin 15/17），波特率 115200，自定义二进制帧 |
| **RDK ↔ GPS** | UART6（Pin 16/18），波特率 9600，NMEA-0183 协议 |
| **RDK ↔ 手机** | RDK开启WiFi AP模式，手机/平板连接后访问Web页面 |
| **主电源** | 24V锂电池组（容量无严格要求，原型机够用即可） |
| **降压分配** | 24V→5V/3.3V DC-DC给RDK、STM32、传感器；24V直驱电机 |

---

## 3. 全链路时间周期与数据流

### 3.1 任务周期总表

| 节点/任务 | 所在平台 | 周期 | 频率 | 说明 |
|-----------|----------|------|------|------|
| `TaskSensor` | STM32 | **100 ms** | 10 Hz | 读取空气+土壤传感器 |
| `TaskControl` | STM32 | **20 ms** | 50 Hz | 电机PID闭环、编码器反馈、舵机控制 |
| `TaskComm` | STM32 | **100 ms** | 10 Hz | 打包上传传感器数据，接收RDK控制指令 |
| `camera_node` | RDK X5 | **500 ms** | 2 Hz | MIPI摄像头采集、图像预处理 |
| `ai_inference_node` | RDK X5 | **500 ms** | 2 Hz | MobileNetV2 TFLite 推理 |
| `gps_node` | RDK X5 | **100 ms** | 10 Hz | UART6 读取 G60 NMEA 数据并解析 |
| `uart_bridge_node` | RDK X5 | **100 ms** | 10 Hz | UART2 读取 STM32 数据帧并发布ROS话题 |
| `fusion_node` | RDK X5 | **事件触发** | ≤2 Hz | 由 `/sentry/ai/diagnosis` 触发融合 |
| `nav_node` | RDK X5 | **200 ms** | 5 Hz | GPS航点导航、路径跟踪 |
| `web_server` | RDK X5 | **200 ms** | 5 Hz | FastAPI WebSocket 推送 |

### 3.2 时空误差分析（基于 0.5 m/s）

底盘以 **0.5 m/s** 匀速行驶时，各周期对应的物理位移：

| 周期 | 位移 | 影响分析 |
|------|------|----------|
| **20 ms**（控制周期） | **1 cm** | 电机PID响应足够，无感知延迟 |
| **100 ms**（传感器/GPS周期） | **5 cm** | 传感器数据与位置匹配误差约5cm，可接受 |
| **500 ms**（AI推理周期） | **25 cm** | 单次推理覆盖约25cm行进距离，需确保摄像头视野 > 30cm |
| **2.5 m**（GPS精度） | — | 航点导航允许2.5m半径到达即算成功，不适合精确沿垄 |

> **结论**：AI 推理的 500ms 周期是系统瓶颈。摄像头应朝下倾斜 30°~45°，确保在 25cm 行进距离内，叶片始终处于画面中心区域。

### 3.3 数据流图（混合架构 + 异步订阅）

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
│  ┌─────────────┐         ┌─────────────────┐         ┌───────────┐  │
│  │   GPS G60   │         │  uart_bridge_   │         │  MIPI     │  │
│  │  (UART6)    │         │    node         │         │  Camera   │  │
│  └──────┬──────┘         │  (UART2)        │         └─────┬─────┘  │
│         │                └────────┬────────┘               │        │
│         │                         │                        │        │
│         ▼                         ▼                        ▼        │
│  /sentry/gps/fix        /sentry/sensors/combined    /sentry/camera/ │
│  (sensor_msgs/          (自定义msg)                  image_raw      │
│   NavSatFix)                                                   │    │
│         │                         │                        │        │
│         │                         │                        ▼        │
│         │                         │               ┌──────────────┐  │
│         │                         │               │ ai_inference │  │
│         │                         │               │    _node     │  │
│         │                         │               └──────┬───────┘  │
│         │                         │                      │           │
│         │                         ▼                      ▼           │
│         │                ┌─────────────────────────────────┐        │
│         │                │         fusion_node             │        │
│         │                │   （事件触发：由AI诊断结果触发）   │        │
│         │                │   时间戳对齐 + 超时降级 + 加权融合  │        │
│         │                └──────────────┬──────────────────┘        │
│         │                               │                           │
│         ▼                               ▼                           │
│  ┌─────────────┐              /sentry/ai/final_diagnosis            │
│  │   nav_node  │                      │                            │
│  │  (航点导航)  │                      │                            │
│  └──────┬──────┘                      │                            │
│         │                             ▼                            │
│         │                    ┌─────────────────┐                   │
│         │                    │   web_server    │                   │
│         │                    │ (FastAPI+前端)   │                   │
│         │                    └─────────────────┘                   │
│         │                                                          │
│         └──────────────────────→ /sentry/cmd_vel                   │
│                                    (geometry_msgs/Twist)           │
│                                          │                         │
│                                          ▼                         │
│                                   ┌─────────────┐                  │
│                                   │  UART2_TX   │                  │
│                                   │ (下发控制)   │                  │
│                                   └─────────────┘                  │
└─────────────────────────────────────────────────────────────────────┘
```

---

## 4. 通信协议定义

### 4.1 STM32 ↔ RDK X5（UART2，115200，3.3V TTL）

采用**自定义二进制帧**，便于STM32解析和ROS2节点处理。

#### 帧格式
```
[帧头2B] [类型1B] [长度1B] [载荷nB] [CRC16-CCITT 2B]
0xAA 0x55   TYPE     LEN      DATA       CRC16
```

#### 数据类型（TYPE）

| TYPE | 方向 | 含义 | 载荷内容 |
|------|------|------|----------|
| `0x01` | STM32→RDK | **传感器汇总帧** | 空气温湿度CO₂ + 土壤电导率/氮磷钾/温湿度/pH |
| `0x03` | STM32→RDK | **底盘状态帧** | 左轮速、右轮速、电池电压、报警位 |
| `0x81` | RDK→STM32 | **运动控制帧** | 左轮目标速、右轮目标速（mm/s） |
| `0x82` | RDK→STM32 | **云台控制帧** | 舵机俯仰角、偏航角（角度值） |
| `0x83` | RDK→STM32 | **模式切换帧** | 0x00=待机, 0x01=遥控, 0x02=自动巡航 |

> **v2.0 改动**：移除 `0x02` GPS定位帧（GPS已直连RDK），STM32不再转发GPS数据。

#### 传感器汇总帧（TYPE=0x01）载荷定义

```c
// 建议结构体定义（STM32端）
typedef struct {
    uint32_t timestamp_ms;      // STM32开机后的毫秒时间戳（用于RDK端时间戳对齐）
    int16_t  air_temp_x10;      // 空气温度 ×10（单位：0.1℃）
    uint16_t air_humi_x10;      // 空气湿度 ×10（单位：0.1%RH）
    uint16_t air_co2;           // CO₂浓度（单位：ppm）
    int16_t  soil_temp_x10;     // 土壤温度 ×10（单位：0.1℃）
    uint16_t soil_humi_x10;     // 土壤湿度 ×10（单位：0.1%RH）
    uint16_t soil_ec;           // 土壤电导率（单位：us/cm）
    uint16_t soil_n;            // 氮含量（单位：mg/kg）
    uint16_t soil_p;            // 磷含量（单位：mg/kg）
    uint16_t soil_k;            // 钾含量（单位：mg/kg）
    uint16_t soil_ph_x10;       // pH值 ×10（单位：0.1pH）
} __attribute__((packed)) SensorFrame_t;
// 总长度：2+1+1+24+2 = 30 字节（含帧头长度类型CRC）
```

#### CRC校验
- 算法：**CRC16-CCITT** (`0x1021`)
- 范围：从 `类型` 字节到 `载荷` 末尾
- 初始值：`0xFFFF`

### 4.2 GPS ↔ RDK X5（UART6，9600，NMEA-0183）

- **协议**：标准 NMEA-0183，GGA + RMC 语句
- **解析**：RDK端使用 `gps_node`（Python `pynmea2` 库或自研解析器）
- **ROS2话题**：`/sentry/gps/fix`，类型 `sensor_msgs/NavSatFix`
- **精度**：2.5m（水平），无RTK，仅用于航点级导航

---

## 5. ROS2 话题设计（RDK X5 内部）

| 话题名 | 类型 | 发布者 | 订阅者 | 频率 | 说明 |
|--------|------|--------|--------|------|------|
| `/sentry/camera/image_raw` | `sensor_msgs/Image` | camera_node | ai_inference_node | 2Hz | 摄像头原始图像 |
| `/sentry/ai/diagnosis` | 自定义msg | ai_inference_node | **fusion_node** | 2Hz | 病害类型+视觉置信度 |
| `/sentry/sensors/combined` | 自定义msg | uart_bridge_node | **fusion_node** | 10Hz | STM32转发的空气+土壤数据 |
| `/sentry/gps/fix` | `sensor_msgs/NavSatFix` | gps_node | nav_node, web_server | 10Hz | GPS定位（直连） |
| `/sentry/ai/final_diagnosis` | 自定义msg | **fusion_node** | web_server | ≤2Hz | **融合后最终诊断结果** |
| `/sentry/cmd_vel` | `geometry_msgs/Twist` | nav_node / web_control_node | uart_bridge_node | 5Hz | 底盘速度指令（下发STM32） |
| `/sentry/servo_cmd` | 自定义msg | web_control_node | uart_bridge_node | 5Hz | 云台角度指令（下发STM32） |
| `/sentry/mode` | `std_msgs/UInt8` | web_control_node | uart_bridge_node | 事件 | 模式切换指令 |

### 5.1 fusion_node 融合逻辑（异步订阅 + 时间戳对齐）

```python
# 核心策略伪代码
class FusionNode(Node):
    def __init__(self):
        self.vision_result = None      # 最新AI诊断结果
        self.sensor_result = None      # 最新传感器汇总数据
        self.VISION_TIMEOUT = 2.0      # 视觉数据有效期：2秒
        self.SENSOR_TIMEOUT = 3.0      # 传感器数据有效期：3秒

    def on_vision(self, msg):
        """AI推理结果到达时触发"""
        self.vision_result = msg
        self.try_fuse()

    def on_sensor(self, msg):
        """传感器数据到达时仅更新缓存"""
        self.sensor_result = msg
        # 不触发融合，等待AI结果

    def try_fuse(self):
        if self.vision_result is None:
            return  # 尚无AI结果，不决策

        vision_age = now() - self.vision_result.header.stamp
        if vision_age > self.VISION_TIMEOUT:
            return  # AI结果过期，丢弃

        # 融合权重：视觉0.6 + 环境0.4（来自原项目方案）
        if self.sensor_result and (now() - self.sensor_result.timestamp) < self.SENSOR_TIMEOUT:
            # 双模态融合
            risk = 0.6 * self.vision_result.disease_prob + \
                   0.4 * self.sensor_result.leaf_humidity_factor
            confidence = "HIGH"
        else:
            # 传感器超时或缺失：降级为视觉单模态
            risk = self.vision_result.disease_prob
            confidence = "MEDIUM"
            self.get_logger().warn("传感器数据超时，启用视觉单模态降级决策")

        self.publish_final_diagnosis(risk, confidence)
```

> **关键原则**：以 AI 推理（慢速、核心）为融合触发事件，传感器数据（快速、辅助）作为上下文缓存。两者解耦运行，不存在"等待阻塞"。

---

## 6. 关键约束与风险

### 6.1 已接受的简化
- **定位**：使用2.5m精度GPS，放弃厘米级RTK；导航策略为"航点巡航"而非"精确沿垄"
- **机械结构**：无机械加工条件，底盘/云台/机械臂（如有）全部采购成品
- **续航/寿命**：原型机阶段不优化，电池够用即可
- **网络**：完全离线，RDK仅开AP热点供局域网内手机访问
- **速度**：0.5 m/s 匀速，AI 500ms周期对应25cm位移，要求摄像头视野覆盖该范围

### 6.2 当前风险（需跟踪）
| 风险 | 影响 | 缓解措施 |
|------|------|----------|
| **电机驱动电流不足** | TB6612FNG烧毁或带不动 | 先用小功率测试，确认电流后决定是否更换驱动 |
| **传感器协议未知** | STM32无法解析数据 | 尽快向卖家索要UART协议文档（波特率、帧格式） |
| **AI训练数据缺失** | 模型准确率无法达标 | 先下载PlantVillage公开数据集做预训练 |
| **GPS精度不足** | 自动巡航时偏离田垄 | 放宽导航精度要求，以"到达大致区域"为目标 |
| **机械臂复杂度** | 3人团队机械工作量过大 | **建议第一阶段取消机械臂**，将土壤传感器固定安装在底盘腹部 |
| **NPU推理延迟** | MobileNetV2在RDK上实际推理时间可能 > 500ms | 预留性能余量，必要时降低输入分辨率至224×224 |

---

## 7. 目录指引（建议的代码结构）

```
smart-agri-sentry/
├── docs/
│   ├── requirements/          # 项目方案、需求文档
│   └── hardware_refs/         # 传感器协议、GPS模块说明、RDK手册、40pin引脚图
├── firmware/                # STM32代码（CubeMX + FreeRTOS）
│   ├── Core/
│   ├── FreeRTOS/
│   ├── App/
│   │   ├── sensor_task.c     # 七合一传感器读取（空气+土壤）
│   │   ├── motor_control.c   # PID + 编码器
│   │   ├── servo_control.c   # 云台舵机控制
│   │   └── rdk_protocol.c    # 与RDK的通信协议编解码（v2.0：无GPS转发）
│   └── CMakeLists.txt
├── software/                # RDK X5代码（ROS2 + Python）
│   ├── src/
│   │   ├── uart_bridge_node.py   # UART2 ↔ ROS2话题桥接
│   │   ├── gps_node.py           # UART6 GPS NMEA解析
│   │   ├── ai_inference_node.py  # TFLite推理
│   │   ├── camera_node.py        # MIPI摄像头采集
│   │   ├── fusion_node.py        # 多传感器异步融合（v2.0核心）
│   │   ├── nav_node.py           # GPS航点导航
│   │   └── web_server/           # FastAPI + 前端
│   ├── msg/
│   │   ├── AiDiagnosis.msg       # AI诊断结果
│   │   ├── SensorCombined.msg    # 传感器汇总
│   │   └── FinalDiagnosis.msg    # 融合后最终诊断
│   └── config/
├── models/                  # TFLite量化模型
└── tests/                   # 单元测试、联调脚本
```

---

## 已确认技术细节（2026-04-22）

### 模型
- 路径：`models/finetuned_mobilenetv2_int8.tflite`
- 格式：原生 TFLite（CPU 推理），后续可转 NPU
- 输入尺寸：224×224
- 输出类别（10 类）：bacterial_spot, early_blight, healthy, late_blight, leaf_mold, septoria_leaf_spot, spider_mites_two-spotted_spider_mite, target_spot, tomato_mosaic_virus, tomato_yellow_leaf_curl_virus

### 摄像头
- 型号：IMX219（MIPI-CSI）

### GPS
- 输出频率：1 Hz（GGA + RMC）

### 前端架构
- Vue 直连 `rosbridge_server` WebSocket
- FastAPI 负责：航点管理、SQLite、MJPEG 视频流代理

### WebSocket 分层推送（最终版）
| 数据类型 | 推送频率 |
|---|---|
| 传感器环境数据 | 1 Hz（巡检）/ 5 Hz（精细监测） |
| AI 诊断结果 | 2 Hz |
| 底盘状态 | 2 Hz |
| 紧急报警 | 事件触发 + 1 Hz 确认 |
| 远程控制指令 | 20-50 Hz |
| 视频流 | 独立 HTTP MJPEG，15-20 fps |

### 融合策略
- Demo 版：固定规则（环境条件触发权重调整）
- 长期：条件门控 + 自适应权重 + AHP + 逻辑回归训练

### 导航
- Batch 1 仅生成占位框架，不实现纯追踪算法
- 航点存储：YAML 模板 + SQLite 当前执行 + Web 可视化编辑
- 策略：改良版纯追踪 + 路径点/任务点分层

### STM32 协议
- 由我自主设计，当前 v2.0 自定义二进制帧已冻结
- uart_bridge_node 已按此实现

---

## 8. 待确认事项（TODO）

- [ ] 确认24V减速电机的**额定电流和堵转电流**，验证TB6612FNG是否够用
- [ ] 向传感器卖家索要**七合一空气传感器**和**七合一土壤传感器**的UART通信协议文档（波特率、数据帧格式、主动/被动发送）
- [x] 确认RDK X5支持的**MIPI摄像头具体型号** → IMX219
- [x] 确认RDK X5官方推荐的**ROS2版本** → Humble
- [ ] 确认是否有**24V→5V大功率DC-DC降压模块**（给RDK X5供电，建议≥5A）
- [ ] 确认比赛规则是否**强制要求机械臂/土壤采样动作**，若否，建议第一阶段取消机械臂以保进度
- [x] 确认AI病害识别的**数据集来源** → PlantVillage公开数据集
- [x] 确认G60 GPS模块的**实际输出频率** → 1Hz（GGA+RMC）

---

## 9. 三人分工参考（与CONTEXT配套）

| 角色 | 负责模块 | 核心交付物 |
|------|----------|------------|
| **嵌入式控制** | STM32固件全栈（传感器采集、电机PID、通信协议） | `firmware/`仓库，底盘可动，数据可采，协议帧稳定 |
| **AI与软件** | RDK X5系统（ROS2节点、AI推理、融合算法、Web后端） | `software/`仓库，异步数据流跑通，Web可看数据，AI可推理 |
| **机械与集成** | 采购底盘/云台、整机装配、GPS安装、联调牵头 | 物理样机，三地联调通过，摄像头视野覆盖25cm位移 |

---

> **使用建议**：将此文件放在 `.claude/PROJECT_CONTEXT.md`，与Claude Code对话时直接 `@.claude/PROJECT_CONTEXT.md`，Claude会基于这些约束生成符合你硬件现实的代码。
