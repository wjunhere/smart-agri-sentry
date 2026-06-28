# RDK X5 接收固定 LoRa 环境节点并接入 ROS2 设计

## 1. 背景与目标

在固定环境节点（STM32F103RCT6 + CJ-702 空气传感器 + 土壤/叶面温湿度传感器）采集环境数据后，通过 E22-400T30S LoRa 模块发送；由自带 STM32F103C8T6 的 E22-400TBH-SC 接收模块接收，并经 USB CDC 虚拟串口透传给 RDK X5。

本设计目标是在 RDK X5 侧新增一个 ROS2 节点，解析 LoRa 自定义二进制帧，把固定节点的空气、土壤、叶面数据发布为 ROS2 话题，供融合、决策、数据记录等下游节点消费。

## 2. 硬件连接

```text
固定环境 MCU (STM32F103RCT6)
        │
        ├── CJ-702 空气传感器       → msg_type 0x01
        ├── 土壤传感器              → msg_type 0x02
        └── 叶面温湿度传感器        → msg_type 0x03
        │
        ▼  USART3
    E22-400T30S 发送模块
        │
        ▼  433MHz LoRa
    E22-400TBH-SC 接收模块
        │
        ▼  USB CDC 虚拟串口
    RDK X5
```

- 接收端使用 `test/E22_400TBH_SC` 工程默认的 `FUNC_FEATURE1` 透传模式：LoRa 收到 1 字节就经 USB CDC 转发 1 字节。
- RDK X5 上识别为 `/dev/ttyACM0`（或 `/dev/ttyUSB0`，具体以 udev 规则为准）。
- 串口波特率：**9600 bps**（与 E22 透传串口波特率一致）。

## 3. 软件架构

### 3.1 进程划分

新增独立节点 `lora_bridge_node`，**不改动**现有 `uart_bridge_node`：

| 节点 | 负责硬件 | 说明 |
|---|---|---|
| `uart_bridge_node` | 移动底盘/移动环境传感器 UART | 现有逻辑保持不变 |
| `lora_bridge_node` | 固定 LoRa 接收模块 USB CDC | 本设计新增 |

### 3.2 数据流

```text
USB CDC 字节流
        │
        ▼
  按 0xAA 0x55 切帧
        │
        ▼
  CRC-8/MAXIM 校验
        │
        ▼
  按 msg_type 分发
        ├── 0x01 → parse_air()    → /sensor/environment_fixed
        │                            /sensor/air_quality_fixed
        ├── 0x02 → parse_soil()   → /sensor/soil_nutrition_fixed
        └── 0x03 → parse_leaf()   → /sensor/leaf_wetness_fixed
                                     /sensor/environment_fixed (仅 leaf_wetness)
```

## 4. 统一 LoRa 帧格式

所有固定节点数据共用同一套外帧格式，通过 `msg_type` 区分传感器类型：

| 字节 | 字段 | 说明 |
|---|---|---|
| 0~1 | 帧头 | `0xAA 0x55` |
| 2 | device_id | 节点编号，默认 `0x01`；多地点部署时可递增 |
| 3 | msg_type | `0x01`=空气，`0x02`=土壤，`0x03`=叶面，`0xFF`=错误 |
| 4 | len | payload 长度 N |
| 5 ~ 5+N-1 | payload | 传感器数据 |
| 5+N | CRC8 | CRC-8/MAXIM，覆盖范围：帧头到 payload 全部字节 |

CRC-8/MAXIM 参数：多项式 `0x31`，初始值 `0x00`，无输入输出反转。

### 4.1 msg_type 0x01：空气传感器（CJ-702）

Payload 长度：**14 字节**。

| 字节 | 字段 | 类型 | 单位/说明 |
|---|---|---|---|
| 0~1 | co2 | uint16 | ppm |
| 2~3 | hcho | uint16 | raw |
| 4~5 | tvoc | uint16 | ppb |
| 6~7 | pm25 | uint16 | μg/m³ |
| 8~9 | pm10 | uint16 | μg/m³ |
| 10~11 | temp | int16 | 0.01 °C |
| 12~13 | humidity | uint16 | 0.01 %RH |

### 4.2 msg_type 0x02：土壤传感器

Payload 长度：**6 字节**。土壤传感器本身可输出 N/P/K，但 LoRa 发送端不发送；土壤 pH 也不在检测范围内。

| 字节 | 字段 | 类型 | 单位/说明 |
|---|---|---|---|
| 0~1 | soil_temp | int16 | 0.01 °C |
| 2~3 | soil_humidity | uint16 | 0.01 %RH |
| 4~5 | ec | uint16 | mS/cm 或 raw |

### 4.3 msg_type 0x03：叶面温湿度传感器

Payload 长度：**4 字节**。

| 字节 | 字段 | 类型 | 单位/说明 |
|---|---|---|---|
| 0~1 | leaf_temp | int16 | 0.01 °C |
| 2~3 | leaf_humidity | uint16 | 0.01 %RH |

### 4.4 msg_type 0xFF：错误帧

Payload 长度：**1 字节**。

| 值 | 含义 |
|---|---|
| 0x01 | 传感器通信超时 |
| 0x02 | 数据不完整 / 连续校验失败 |

## 5. ROS2 消息与话题映射

### 5.1 新建消息类型

#### `AirQuality.msg`

```msg
std_msgs/Header header
float32 hcho
float32 tvoc
float32 pm25
float32 pm10
string data_source
```

#### `LeafEnvironment.msg`

```msg
std_msgs/Header header
float32 leaf_temp
float32 leaf_humidity
string data_source
```

### 5.2 现有消息复用

- `Environment.msg`：
  - `msg_type 0x01` 空气数据：发布到 `/sensor/environment_fixed`，填入 `air_temp`、`air_humidity`、`air_co2`。
  - `msg_type 0x03` 叶面数据：发布到 `/sensor/environment_fixed`，仅填入 `leaf_wetness`，其余字段填 `NaN`。
  - 这样 `/sensor/environment_fixed` 上会出现“部分字段有效”的消息；下游消费者若做平均/融合，需过滤 `NaN` 字段。
- `SoilNutrition.msg`：用于 `/sensor/soil_nutrition_fixed`，`soil_temp`、`soil_humidity`、`ec` 按解析值填入；`n`、`p`、`k`、`ph` 填 `0`（传感器不输出）。

### 5.3 话题列表

| 话题 | 消息类型 | 来源 msg_type | 说明 |
|---|---|---|---|
| `/sensor/environment_fixed` | `Environment` | 0x01 | 固定节点空气温湿度/CO2/叶面湿度 |
| `/sensor/air_quality_fixed` | `AirQuality` | 0x01 | 固定节点空气质量 |
| `/sensor/soil_nutrition_fixed` | `SoilNutrition` | 0x02 | 固定节点土壤温湿度/EC |
| `/sensor/leaf_wetness_fixed` | `LeafEnvironment` | 0x03 | 固定节点叶面温湿度 |

所有消息 `data_source` 固定填 `'FIXED_LORA'`。

## 6. RDK 节点设计

### 6.1 文件位置

```text
src/sentry_sensors/
├── sentry_sensors/
│   ├── __init__.py
│   ├── uart_bridge_node.py       # 不变
│   └── lora_bridge_node.py       # 新增
├── msg/  (位于 src/sentry_interfaces/)
│   ├── AirQuality.msg            # 新增
│   └── LeafEnvironment.msg       # 新增
├── launch/
│   └── lora_bridge.launch.py     # 新增
├── setup.py                      # 增加入口点
└── package.xml                   # 增加 sentry_interfaces 依赖
```

> 注：`src/sentry_sensors/sentry_sensors/` 是 ament_python 包的标准 Python 模块目录，与现有 `uart_bridge_node.py` 同层。

### 6.2 节点参数

| 参数 | 类型 | 默认值 | 说明 |
|---|---|---|---|
| `uart_port` | string | `/dev/ttyACM0` | LoRa 接收模块 USB CDC 串口 |
| `baudrate` | int | `9600` | 串口波特率 |

### 6.3 核心类 `LoraBridgeNode`

```python
class LoraBridgeNode(Node):
    def __init__(self, **kwargs):
        # 声明参数、打开串口
        # 创建发布者
        # 创建 10ms 定时器 rx_tick

    def rx_tick(self):
        # 读取串口可用字节，追加到接收缓冲区
        # 循环查找 0xAA 0x55 帧头
        # 提取完整帧，校验 CRC，按 msg_type 分发

    def _parse_air(self, payload: bytes, ts: Time):
        # 解析 14 字节，发布 Environment + AirQuality

    def _parse_soil(self, payload: bytes, ts: Time):
        # 解析 6 字节，发布 SoilNutrition

    def _parse_leaf(self, payload: bytes, ts: Time):
        # 解析 4 字节，发布 LeafEnvironment
        # 同时发布一份 Environment，只填 leaf_wetness
```

### 6.4 数值缩放

所有 payload 多字节字段采用**大端字节序**；解析时按如下规则转换：

- 有符号温度：`int16(value) / 100.0` → °C
- 湿度：`uint16(value) / 100.0` → %RH
- CO2/HCHO/TVOC/PM2.5/PM10/EC：直接作为 uint16 使用，单位保持不变

## 7. 错误处理

| 场景 | 行为 |
|---|---|
| CRC 校验失败 | 丢弃该帧，`log warn` |
| 帧长度不足 | 保留在接收缓冲区，等待后续字节 |
| 未知 msg_type | 丢弃，`log warn` |
| payload 长度与 msg_type 期望不符 | 丢弃，`log warn` |
| 串口打开失败 | `log error`，节点继续运行，定时重试 |
| 串口读取异常 | `log error`，清空缓冲区，尝试恢复 |

## 8. 启动配置

新增 `src/sentry_sensors/launch/lora_bridge.launch.py`：

```python
Node(
    package='sentry_sensors',
    executable='lora_bridge_node',
    name='lora_bridge_node',
    parameters=[{
        'uart_port': '/dev/ttyACM0',
        'baudrate': 9600,
    }],
)
```

udev 规则（可选）：为 LoRa 接收模块固定 symlink，例如 `/dev/lora_fixed_env`。

## 9. 测试计划

### 9.1 单元测试

在 `src/sentry_sensors/tests/test_lora_bridge_node.py` 中：

- 构造合法空气/土壤/叶面帧，验证 CRC 计算正确。
- 验证各 parser 输出的 ROS2 消息字段值正确。
- 构造 CRC 错误、未知 msg_type、长度不足帧，验证节点正确丢弃并记录日志。

### 9.2 集成测试

- 在 RDK 上启动 `lora_bridge_node`，连接 E22-400TBH-SC 接收模块。
- 使用 `ros2 topic echo` 检查 `/sensor/environment_fixed`、`/sensor/air_quality_fixed`、`/sensor/soil_nutrition_fixed`、`/sensor/leaf_wetness_fixed` 数据正确。
- 验证 `fusion_node` 能正常订阅 `/sensor/environment_fixed`。

### 9.3 长期测试

- 连续运行 10 分钟以上，观察是否有内存增长、话题中断、CRC 误报。

## 10. 依赖与影响

### 10.1 新增依赖

- `sentry_interfaces` 新增 `AirQuality.msg`、`LeafEnvironment.msg`。
- `sentry_sensors` 新增对 `sentry_interfaces` 的依赖（`package.xml` 中已存在，无需新增）。

### 10.2 不影响的模块

- `uart_bridge_node.py` 逻辑不变。
- `Environment.msg` 不改字段。
- 移动节点话题 `/sensor/environment_mobile`、`/sensor/soil_nutrition` 不变。

## 11. 决策记录

| 决策 | 选项 | 理由 |
|---|---|---|
| 独立节点 vs 复用 `uart_bridge_node` | 独立 `lora_bridge_node` | 后续还要加入土壤、叶面传感器，独立节点职责更清晰，扩展方便 |
| 连接方式 | USB CDC | `test/E22_400TBH_SC` 已支持 USB CDC 透传，无需改 STM32 代码 |
| 消息类型 | 新建 `AirQuality.msg`、`LeafEnvironment.msg` | 现有 `Environment.msg` 字段不足，且融合算法不需要叶面温度 |
| 叶面湿度发布 | 同时发 `Environment.leaf_wetness` 和 `LeafEnvironment` | 既满足融合节点输入，又保留完整叶面温湿度数据 |
| 土壤 N/P/K/pH | 填 0 | LoRa 发送端不发送 N/P/K，传感器不检测 pH |
