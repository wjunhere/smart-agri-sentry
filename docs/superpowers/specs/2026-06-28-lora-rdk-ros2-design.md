# RDK X5 接收固定 LoRa 环境节点并接入 ROS2 设计

## 1. 背景与目标

在固定环境节点（STM32F103RCT6 + CJ-702 空气传感器 + 土壤/叶面温湿度传感器）采集环境数据后，通过 E22-400T30S LoRa 模块发送；由自带 STM32F103C8T6 的 E22-400TBH-SC 接收模块接收，并经 USB CDC 虚拟串口透传给 RDK X5。

本设计目标是在 RDK X5 侧新增一个 ROS2 节点，解析 LoRa 自定义二进制帧，把固定节点的空气、土壤、叶面数据发布为 ROS2 话题，供融合、决策、数据记录等下游节点消费。

## 2. 硬件连接

```text
固定环境 MCU (STM32F103RCT6)
        │
        ├── CJ-702 空气传感器
        ├── 土壤传感器
        └── 叶面温湿度传感器
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

- 固定环境 MCU 在本地把三类传感器数据聚合成一帧后，通过 USART3 发送给 E22-400T30S。
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
  msg_type 0x01 → parse_environment()
        │
        ▼
  /sensor/environment_fixed
```

固定环境 MCU 在发送前已经把空气、土壤、叶面三类传感器数据聚合成一帧；RDK 侧只需解析一次，发布一个完整的 `Environment.msg`。

## 4. 统一 LoRa 帧格式

固定环境 MCU 先把三类传感器数据聚合成一帧，再通过 LoRa 发送。RDK 侧只解析一种数据帧：

| 字节 | 字段 | 说明 |
|---|---|---|
| 0~1 | 帧头 | `0xAA 0x55` |
| 2 | device_id | 节点编号，默认 `0x01`；多地点部署时可递增 |
| 3 | msg_type | `0x01`=聚合环境数据，`0xFF`=错误；`0x02`/`0x03` 预留 |
| 4 | len | payload 长度 N |
| 5 ~ 5+N-1 | payload | 传感器数据 |
| 5+N | CRC8 | CRC-8/MAXIM，覆盖范围：帧头到 payload 全部字节 |

CRC-8/MAXIM 参数：多项式 `0x31`，初始值 `0x00`，无输入输出反转。

### 4.1 msg_type 0x01：聚合环境数据帧

Payload 长度：**24 字节**。

| 字节 | 字段 | 类型 | 单位/说明 |
|---|---|---|---|
| 0~1 | co2 | uint16 | ppm |
| 2~3 | hcho | uint16 | raw |
| 4~5 | tvoc | uint16 | ppb |
| 6~7 | pm25 | uint16 | μg/m³ |
| 8~9 | pm10 | uint16 | μg/m³ |
| 10~11 | air_temp | int16 | 0.01 °C |
| 12~13 | air_humidity | uint16 | 0.01 %RH |
| 14~15 | soil_temp | int16 | 0.01 °C |
| 16~17 | soil_humidity | uint16 | 0.01 %RH |
| 18~19 | ec | uint16 | mS/cm 或 raw |
| 20~21 | leaf_wetness | uint16 | 0.01 %RH（叶面湿度传感器输出的湿度值） |
| 22~23 | leaf_temp | int16 | 0.01 °C |

> 说明：土壤传感器不检测 pH，也不通过 LoRa 发送 N/P/K；若后续需要，可改用 `msg_type 0x02` 单独发送土壤营养帧。

### 4.2 msg_type 0xFF：错误帧

Payload 长度：**1 字节**。

| 值 | 含义 |
|---|---|
| 0x01 | 传感器通信超时 |
| 0x02 | 数据不完整 / 连续校验失败 |

## 5. ROS2 消息与话题映射

### 5.1 扩展 `Environment.msg`

在现有 `Environment.msg` 基础上新增空气质量和叶面温度字段：

```msg
std_msgs/Header header
float32 air_temp
float32 air_humidity
float32 air_co2
float32 soil_temp
float32 soil_humidity
float32 leaf_wetness      # 叶面湿度（来自叶面温湿度传感器）
float32 hcho              # 新增：甲醛 raw
float32 tvoc              # 新增：TVOC ppb
float32 pm25              # 新增：PM2.5 μg/m³
float32 pm10              # 新增：PM10 μg/m³
float32 leaf_temp         # 新增：叶面温度 °C
float32 ec                # 新增：土壤电导率
string data_source
```

新增字段放在末尾，保持与现有代码的向后兼容；未初始化的字段默认值为 `0.0`。

### 5.2 话题列表

| 话题 | 消息类型 | 说明 |
|---|---|---|
| `/sensor/environment_fixed` | `Environment` | 固定节点完整环境数据 |

`data_source` 固定填 `'FIXED_LORA'`。

### 5.3 对下游节点的影响

- `fusion_node.py` 已订阅 `/sensor/environment_fixed` 并做平均；新增字段默认 `0.0`，不影响现有计算。
- 若后续 `fusion_node` 需要把 PM2.5/PM10/TVOC/HCHO 纳入决策，可直接读取扩展后的字段。

## 6. RDK 节点设计

### 6.1 文件位置

```text
src/sentry_sensors/
├── sentry_sensors/
│   ├── __init__.py
│   ├── uart_bridge_node.py       # 不变
│   └── lora_bridge_node.py       # 新增
├── msg/  (位于 src/sentry_interfaces/)
│   └── (Environment.msg 扩展，不新建消息)
├── launch/
│   └── lora_bridge.launch.py     # 新增
├── setup.py                      # 增加入口点
└── package.xml                   # 无需新增依赖
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
        # 创建 /sensor/environment_fixed 发布者
        # 创建 10ms 定时器 rx_tick

    def rx_tick(self):
        # 读取串口可用字节，追加到接收缓冲区
        # 循环查找 0xAA 0x55 帧头
        # 提取完整帧，校验 CRC，解析 msg_type 0x01

    def _parse_environment(self, payload: bytes, ts: Time):
        # 解析 24 字节，填充 Environment.msg 全部字段
        # 发布到 /sensor/environment_fixed
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

- 构造合法聚合环境帧（24 字节 payload），验证 CRC 计算正确。
- 验证解析后的 `Environment.msg` 所有字段值正确。
- 构造 CRC 错误、未知 msg_type、长度不足帧，验证节点正确丢弃并记录日志。

### 9.2 集成测试

- 在 RDK 上启动 `lora_bridge_node`，连接 E22-400TBH-SC 接收模块。
- 使用 `ros2 topic echo /sensor/environment_fixed` 检查所有字段数据正确。
- 验证 `fusion_node` 能正常订阅 `/sensor/environment_fixed`，且新增字段默认 `0.0` 不影响融合。

### 9.3 长期测试

- 连续运行 10 分钟以上，观察是否有内存增长、话题中断、CRC 误报。

## 10. 依赖与影响

### 10.1 变更

- `sentry_interfaces/msg/Environment.msg` 新增 `hcho`、`tvoc`、`pm25`、`pm10`、`leaf_temp`、`ec` 字段。
- `sentry_sensors` 新增 `lora_bridge_node.py` 及启动文件。

### 10.2 不影响的模块

- `uart_bridge_node.py` 逻辑不变。
- 移动节点话题 `/sensor/environment_mobile`、`/sensor/soil_nutrition` 不变。
- 不新建额外消息类型。

## 11. 决策记录

| 决策 | 选项 | 理由 |
|---|---|---|
| 独立节点 vs 复用 `uart_bridge_node` | 独立 `lora_bridge_node` | 职责清晰，LoRa 固定节点与移动 UART 节点分离 |
| 连接方式 | USB CDC | `test/E22_400TBH_SC` 已支持 USB CDC 透传，无需改 STM32 接收端代码 |
| 数据聚合位置 | 发送端 MCU 内部聚合 | 用户明确倾向；RDK 侧只需解析一帧，发布一个完整 `Environment.msg` |
| 消息类型 | 扩展 `Environment.msg` | 把所有关键环境量集中在一条消息里，下游消费者（fusion/advisory/forecast）无需订阅多个话题 |
| 土壤 N/P/K/pH | 不发送 | 土壤传感器不检测 pH，LoRa 发送端也不发送 N/P/K；仅发送温度/湿度/EC |
| 叶面湿度字段 | 映射到 `Environment.leaf_wetness` | 复用现有字段，无需新建消息；叶面温度新增 `leaf_temp` 字段 |
