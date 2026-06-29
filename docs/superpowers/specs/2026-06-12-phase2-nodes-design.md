# Phase 2 节点设计 — forecast / advisory / data_logger

> 日期：2026-06-12  
> 对应阶段：Phase 2（完整决策闭环：预测 + 建议 + 记录）  
> 架构版本：v2.1 导航增强 + 事件驱动巡检

---

## 1. 设计目标

在 Phase 1 已实现“停-拍-判-走”的基础上，Phase 2 补齐：

- **forecast_node**：基于融合结果与环境历史趋势，对未来 24h 病害风险做简化外推预测。
- **advisory_node**：基于融合结果、预测警报与 YAML 规则生成可执行的农艺建议。
- **data_logger_node**：以 `rosbag2_py` API 持续录制核心 topic，7 天循环覆盖，CRITICAL 事件永久保留。

三者共同构成“感知 → 融合 → 预测 → 建议 → 记录”的完整闭环。

---

## 2. 包与文件结构

新增 3 个 ROS2 Python 包，风格与现有 `sentry_fusion`、`sentry_mission` 保持一致。

```text
src/
├── sentry_forecast/
│   ├── sentry_forecast/
│   │   ├── __init__.py
│   │   └── forecast_node.py
│   ├── config/
│   │   └── forecast_params.yaml
│   ├── tests/
│   │   └── test_forecast_node.py
│   └── setup.py
├── sentry_advisory/
│   ├── sentry_advisory/
│   │   ├── __init__.py
│   │   ├── advisory_node.py
│   │   └── rule_engine.py
│   ├── config/
│   │   └── advisory_rules.yaml
│   ├── tests/
│   │   └── test_advisory_node.py
│   └── setup.py
└── sentry_data_logger/
    ├── sentry_data_logger/
    │   ├── __init__.py
    │   ├── data_logger_node.py
    │   └── bag_writer.py
    ├── config/
    │   └── data_logger_params.yaml
    ├── tests/
    │   └── test_data_logger_node.py
    └── setup.py
```

新增/补充配置：

```text
config/
├── crop_profiles.yaml          # 已存在，复用 LWD 阈值等
├── advisory_rules.yaml         # 新增：农艺建议规则库
├── forecast_params.yaml        # 新增：预测参数
└── data_logger_params.yaml     # 新增：录制与保留策略
```

---

## 3. 节点职责与数据流

### 3.1 forecast_node

**订阅话题**

| 话题 | 类型 | 用途 |
|------|------|------|
| `/fusion/diagnosis` | `FusionResult` | 获取 `risk_score`、`lwd_hours`、`confidence`、`mode` |
| `/sensor/environment_mobile` | `Environment` | 获取当前空气温湿度，补充趋势 |

**内部状态**

- 维护最近 6h 的 `(timestamp, risk_score, humidity, lwd_hours)` 环形缓冲区，每次新样本到达时追加。

**触发方式**

- 10 分钟定时器触发一次预测；收到 `/fusion/diagnosis` 时只更新内部缓冲，不立刻发布。

**预测逻辑**

1. 对 `risk_score` 做线性趋势外推：
   ```
   risk_trend = (last_risk - first_risk) / history_hours
   predicted_risk = clip(last_risk + risk_trend * prediction_hours, 0, 1)
   ```
2. 对空气湿度做同样外推，得到 `predicted_humidity`。
3. 分类预警：
   - `RISING_RISK`：`predicted_risk >= risk_threshold` 且趋势上升。
   - `LATENT_OUTBREAK`：当前 `lwd_hours` 距作物 LWD 阈值不足 `lwd_margin_hours`，且湿度趋势上升。
   - `DROUGHT_STRESS`：`humidity <= 40` 且 `temperature >= 30` 持续存在。
   - `NONE`：以上都不满足。

**发布话题**

| 话题 | 类型 | 说明 |
|------|------|------|
| `/forecast/alert` | `ForecastAlert` | `active`、`alert_type`、`probability`、`description`、`hours_ahead=24` |

---

### 3.2 advisory_node

**订阅话题**

| 话题 | 类型 | 用途 |
|------|------|------|
| `/fusion/diagnosis` | `FusionResult` | 当前风险等级、模式、证据链 |
| `/forecast/alert` | `ForecastAlert` | 未来风险趋势 |
| `/sensor/environment_mobile` | `Environment` | 当前环境快照 |

**内部逻辑**

- `rule_engine.py` 启动时加载 `advisory_rules.yaml`。
- 任意订阅触发时，按规则顺序匹配，命中第一条即停止。
- 匹配维度包括：`crop_type`、`alert_level`、`mode`、`alert_type`、`risk_score` 区间、环境阈值。

**发布话题**

| 话题 | 类型 | 说明 |
|------|------|------|
| `/advisory/action` | `AdvisoryAction` | `action_type`、`description`、`priority`、`steps` |

`action_type` 枚举：

- `SPRAY`：喷洒药剂
- `IRRIGATE`：灌溉
- `PRUNE`：修剪病叶
- `MONITOR`：加强监测
- `NONE`：兜底，无需动作

---

### 3.3 data_logger_node

**订阅话题**

| 话题 | 类型 | 用途 |
|------|------|------|
| `/fusion/diagnosis` | `FusionResult` | 检测 CRITICAL 事件 |
| `/mission/status` | `MissionStatus` | 记录状态机 |
| `/forecast/alert` | `ForecastAlert` | 记录预测事件 |
| `/advisory/action` | `AdvisoryAction` | 记录建议事件 |

**行为**

- 启动后使用 `rosbag2_py` 持续录制配置列表中的 topic。
- 常规 bag 存放于 `bags/<YYYY-MM-DD>/`，按 `split_duration_sec`（默认 900s）或 `split_max_size_mb`（默认 1024MB）切片。
- 启动时清理超过 `retention_days`（默认 7 天）的旧 bag 目录。
- 当收到 `FusionResult.alert_level == CRITICAL` 时：
  1. 记录触发时间戳与原因。
  2. 将当前 bag 切片以及前后 `critical_retention_sec`（默认 300s）时间窗口内的切片复制到 `records/critical/<timestamp>/`。
  3. 同时写入一个 JSON 元数据文件，包含触发时间、作物类型、风险分数、建议动作、最近的 GPS/状态信息（从已有 topic 中缓存获得）。

**实现**

- `bag_writer.py` 封装 `rosbag2_py`，优先尝试 `Recorder`；若不可用则退到 `SequentialWriter` 手动写入。
- 所有消息序列化与类型信息从 ROS2 消息对象自动推导，避免硬编码序列化。

---

## 4. 配置设计

### 4.1 `config/forecast_params.yaml`

```yaml
forecast_node:
  timer_period_sec: 600          # 预测周期 10 分钟
  history_hours: 6               # 趋势回看窗口
  prediction_hours: 24           # 预测未来时长
  risk_threshold: 0.7            # RISING_RISK 触发阈值
  lwd_margin_hours: 2.0          # 距 LWD 阈值多近算“接近”
  humidity_trend_threshold: 0.3  # 湿度上升趋势阈值（-1 ~ +1）
```

### 4.2 `config/advisory_rules.yaml`

```yaml
rules:
  - name: critical_late_blight
    conditions:
      crop_type: tomato
      alert_level: CRITICAL
      mode: VISION_DOMINANT
    action:
      action_type: SPRAY
      priority: CRITICAL
      description: "检测到晚疫病高风险，建议立即喷洒杀菌剂。"
      steps:
        - "停车并确认植株编号"
        - "使用对应杀菌剂喷洒"
        - "记录处理位置"

  - name: latent_outbreak
    conditions:
      crop_type: tomato
      alert_type: LATENT_OUTBREAK
    action:
      action_type: MONITOR
      priority: HIGH
      description: "环境条件利于病害爆发，建议增加巡检频次。"

  - name: drought_stress
    conditions:
      humidity_max: 40
      temperature_min: 30
    action:
      action_type: IRRIGATE
      priority: MEDIUM
      description: "干旱胁迫风险，建议适时灌溉。"
```

匹配规则：

- 按文件中的顺序自上而下匹配。
- 条件字段为“与”关系；未指定的字段视为“不限”。
- 没有任何规则命中时，发布 `action_type=NONE`、`priority=LOW` 的兜底动作。

### 4.3 `config/data_logger_params.yaml`

```yaml
data_logger_node:
  topics:
    - /fusion/diagnosis
    - /mission/status
    - /forecast/alert
    - /advisory/action
    - /sensor/environment_mobile
    - /vision/diagnosis
  bag_base_dir: bags/
  split_duration_sec: 900
  split_max_size_mb: 1024
  retention_days: 7
  critical_retention_sec: 300
  record_metadata: true
```

---

## 5. 错误处理与可靠性

| 场景 | 处理策略 |
|------|----------|
| 配置文件缺失 | 使用内建默认值，打印 warn，不崩溃 |
| 环境数据过期（>2s） | 不用于趋势计算 |
| 融合结果过期（>30s） | 不触发预测与建议更新 |
| 历史缓冲不足 | `forecast_node` 输出 `active=false` 的占位警报 |
| 规则未命中 | 发布 `action_type=NONE` 的兜底建议 |
| bag 目录不可写 | 报错并停止录制，不影响节点其他功能 |
| 磁盘满 | 停止录制并打印 error |
| `rosbag2_py` 不可用时 | 降级为只写 JSON 事件日志，保证演示可用 |
| 节点退出 | 正确关闭 writer/recorder，释放资源 |

---

## 6. 接口汇总

### 新增/使用的 ROS2 消息

| 消息 | 来源 | 说明 |
|------|------|------|
| `sentry_interfaces/FusionResult` | 已定义 | 风险分数、模式、证据链、LWD、置信度 |
| `sentry_interfaces/ForecastAlert` | 已定义 | 预测警报 |
| `sentry_interfaces/AdvisoryAction` | 已定义 | 农艺建议 |
| `sentry_interfaces/MissionStatus` | 已定义 | 状态机状态 |
| `sentry_interfaces/Environment` | 已定义 | 环境数据 |
| `sentry_interfaces/Diagnosis` | 已定义 | 病害分类结果 |

### 节点输入/输出

| 节点 | 输入 | 输出 |
|------|------|------|
| forecast_node | `/fusion/diagnosis`, `/sensor/environment_mobile` | `/forecast/alert` |
| advisory_node | `/fusion/diagnosis`, `/forecast/alert`, `/sensor/environment_mobile` | `/advisory/action` |
| data_logger_node | `/fusion/diagnosis`, `/mission/status`, `/forecast/alert`, `/advisory/action` | 磁盘 bag + JSON 元数据 |

---

## 7. 测试策略（TDD）

按项目约定，先写测试再写实现。

### `test_forecast_node.py`

- 模拟 `/fusion/diagnosis` 输入，验证风险线性外推结果。
- 验证冷启动/缓冲不足时 `active=false`。
- 验证 LWD 接近阈值且湿度上升时触发 `LATENT_OUTBREAK`。
- 验证干旱条件触发 `DROUGHT_STRESS`。

### `test_advisory_node.py`

- mock `FusionResult` + `ForecastAlert`，验证规则命中与 `steps` 内容。
- 验证规则优先级顺序。
- 验证无规则命中时的兜底行为。

### `test_data_logger_node.py`

- mock `bag_writer.py`，验证收到 CRITICAL 时调用 `snapshot_critical`。
- 验证普通事件只调用 `write` 不触发 snapshot。
- 验证旧 bag 清理逻辑。

测试使用 `pytest` + `unittest.mock`，不依赖真实硬件或真实 rosbag2 后端。

---

## 8. 依赖关系

### runtime 依赖

- `rclpy`
- `rosbag2_py`（RDK X5 上建议验证是否已安装）
- `std_msgs`, `sensor_msgs`, `geometry_msgs`
- `sentry_interfaces`
- `PyYAML`

### build 依赖

- `ament_python`
- `setuptools`

---

## 9. 后续扩展（Phase 3/4 预留）

- **固定环境节点数据接入**：`forecast_node` 已按 `/sensor/environment_fixed` 设计为可选输入，后续只需添加订阅即可。
- **外部天气接入**：预测算法 `_predict()` 函数可扩展为接收外部天气 forecast 数组。
- **端侧大模型**：`advisory_node` 的 `rule_engine.py` 与 `description` 字段保持可替换，Phase 4 可接入本地 LLM 生成更自然语言建议。
- **Web 前端回放**：`data_logger_node` 保留的 CRITICAL bag 可直接用于后续 Web 可视化。

---

## 10. 决策记录

| 决策 | 选项 | 选择 | 原因 |
|------|------|------|------|
| 包组织 | A 按节点拆包 / B 合并决策包 / C 最小化单包 | **A** | 与现有代码结构一致，便于独立测试与替换 |
| 预测算法 | 线性外推 / 启发式规则 / 可插拔模型 | **线性外推 + 启发式兜底** | 符合上下文“简化趋势外推”，无额外依赖 |
| 数据记录 | `rosbag2_py` API / 子进程 / 仅标记 | **`rosbag2_py` API** | 事件触发切片与 CRITICAL snapshot 控制更精确 |
| 规则引擎 | YAML 规则 / 硬编码 / 大模型 | **YAML 规则** | Phase 2 先跑通闭环，大模型留到 Phase 4 |
