# 天气接入病害预测增强 · 设计文档

> 日期：2026-07-10 · 状态：待实施 · 关联分支：TBD

---

## 1. 目标

接入中国气象局 API，获取未来 7 天逐小时预报与灾害预警数据，与现有本地传感器融合管线混合，增强病害预测和农艺建议能力。

**范围**：天气数据采集 → 混合预测 → 天气感知建议 → 前端配置展示。
**非范围**：多天气源切换、历史天气数据分析、端侧大模型生成建议。

---

## 2. 架构

```
                          ┌──────────────────────┐
                          │   中国气象局 API       │
                          │  (格点预报 + 灾害预警)  │
                          └──────────┬───────────┘
                                     │ HTTP (CMA Open API)
                          ┌──────────▼───────────┐
                          │   sentry_weather      │  ← 新包
                          │   weather_node        │
                          │   - 定时拉取 (3h)      │
                          │   - 本地缓存 (JSON)    │
                          │   - /weather/forecast  │
                          └──────────┬───────────┘
                                     │ WeatherForecast
          ┌──────────────────────────┼──────────────────────────┐
          │                          │                          │
  ┌───────▼───────┐  ┌───────────────▼──────────┐  ┌───────────▼──────────┐
  │ fusion_node   │  │  forecast_node (增强)     │  │ frontend             │
  │ (不改)        │  │  + 本地趋势 (40%)         │  │ - 位置配置            │
  │               │  │  + 天气模型 (60%)         │  │ - 天气预报展示         │
  │ /fusion/      │  │  = 混合预测               │  │ - 建议卡片            │
  │ diagnosis     │  │  /forecast/alert           │  └──────────────────────┘
  └───────┬───────┘  └───────────────┬──────────┘
          │                          │
  ┌───────▼──────────────────────────▼───────────┐
  │          advisory_node (增强)                 │
  │          规则引擎 + 天气条件                   │
  │          如: "未来3天有暴雨 → 提前喷药"        │
  │          /advisory/action                     │
  └──────────────────────────────────────────────┘
```

**数据流说明**：
- `sentry_weather` → `/weather/forecast`：天气原始数据（逐小时+逐日+灾害）
- `sentry_forecast` 订阅 `/weather/forecast` + 本地传感器历史 → 混合预测 → `/forecast/alert`
- `sentry_advisory` 订阅 `/forecast/alert` + `/fusion/diagnosis` → 天气感知建议 → `/advisory/action`
- 前端通过 ROS2 参数服务修改经纬度配置，并展示预报和建议

---

## 3. 新增包：`sentry_weather`

### 3.1 文件结构

```
src/sentry_weather/
├── setup.py
├── sentry_weather/
│   ├── __init__.py
│   ├── weather_node.py      # ROS2 节点主逻辑
│   ├── cma_client.py         # 中国气象局 API 客户端
│   ├── cache_manager.py      # 本地缓存 (JSON 文件)
├── tests/
│   ├── test_cma_client.py
│   ├── test_weather_node.py
│   └── test_cache_manager.py
├── config/
│   └── weather_params.yaml
```

### 3.2 新增消息

**WeatherDay.msg** — 单日预报：
```
uint8 day_offset         # 0=今天, 1=明天 ...
float32 temp_high        # 最高温 °C
float32 temp_low         # 最低温 °C
float32 humidity         # 平均湿度 %
float32 precipitation    # 降水量 mm
float32 wind_speed       # 风速 m/s
string weather_desc      # 晴/多云/雨/雪 等
```

**WeatherHour.msg** — 逐小时预报：
```
uint8 hour_offset        # 距当前小时数
float32 temperature
float32 humidity
float32 precipitation
float32 wind_speed
```

**WeatherForecast.msg** — 完整预报：
```
std_msgs/Header header
string city
float64 lat
float64 lon
WeatherDay[7] days
WeatherHour[] hours         # 可变长度，最多 168
string[] disaster_alerts    # 如 ["暴雨蓝色预警", "大风黄色预警"]
bool stale                   # true = 使用缓存数据（API 拉取失败）
```

### 3.3 节点参数

| 参数 | 类型 | 默认值 | 说明 |
|---|---|---|---|
| `lat` | float64 | 39.9 | 纬度（前端可改） |
| `lon` | float64 | 116.4 | 经度（前端可改） |
| `city` | string | "" | 城市名（可选，用于展示） |
| `fetch_interval_sec` | int | 10800 | 拉取间隔秒（3h） |
| `cache_path` | string | `/tmp/sentry_weather_cache.json` | 缓存文件路径 |
| `api_key` | string | "" | CMA API key |
| `api_base_url` | string | CMA 格点预报 URL | 可切换 |
| `mock_mode` | bool | false | 本地开发 mock |

### 3.4 CMA 客户端 (`cma_client.py`)

- 封装 CMA API 认证和请求逻辑
- `fetch_grid_forecast(lat, lon)` → 解析格点预报返回结构化数据
- `fetch_disaster_warning(lat, lon)` → 解析灾害预警
- 异常处理：网络超时(30s)、HTTP 错误码、JSON 解析失败均返回 `None`
- Mock 模式返回合成数据（随机天气，无灾害）

### 3.5 缓存管理器 (`cache_manager.py`)

- 每次 API 拉取成功后覆盖写入 JSON 文件
- 节点启动时从缓存文件恢复上次数据
- `is_valid()` → 缓存文件存在且修改时间 < 24h 返回 `True`
- 网络失败时读取缓存，`WeatherForecast.stale = True`
- 缓存超过 24h → 视为不可用，不发布数据

### 3.6 节点行为

1. **启动** → 读取缓存文件恢复上次数据 → 立即发布（stale=True）→ 触发首次 API 拉取
2. **定时** → 每 3h 拉取一次 CMA API → 拉取成功覆盖缓存 → 发布（stale=False）
3. **拉取失败** → 日志 warn → 检查缓存有效性 → 有效则发布缓存数据（stale=True），无效则跳过
4. **位置变更** → 前端修改 `lat`/`lon` 参数 → 节点检测参数变化 → 立即触发拉取

---

## 4. 增强包：`sentry_forecast`（修改）

### 4.1 混合预测引擎

现有 `forecast_node` 仅做本地传感器历史线性外推。增强后新增：

```
混合预测 = blend(
    local_risk  = linear_trend(history, prediction_hours),   # 权重 0.4
    weather_risk = weather_model(weather_data, prediction_hours),  # 权重 0.6
    disaster_boost = disaster_factor(weather_data)            # 0~0.3 加成
)
```

权重可通过 `config/forecast_params.yaml` 调整。

### 4.2 天气→风险模型 (`weather_risk_model`)

从逐小时预报数据计算风险分数（0~1）：

| 条件 | 风险贡献 | 说明 |
|---|---|---|
| RH > 85% 且 15°C < T < 25°C 持续 > 6h | +0.2~0.4 | 真菌爆发窗口 |
| 连续降水 > 1mm/h 超过 12h | +0.2~0.3 | LWD 模拟升高 |
| 连续降水 > 2天 | +0.1~0.2 | 长期湿润 |
| T > 35°C 持续 > 6h | +0.2~0.3 | 高温胁迫 |
| T < 5°C 出现 | +0.2~0.4 | 冻害风险（作物相关）|
| 灾害预警命中 | +0.3 | 直接提级 |

`disaster_factor`：逐小时条件累加，封顶 0.3。

### 4.3 新增告警类型

```
现有: NONE / RISING_RISK / LATENT_OUTBREAK / DROUGHT_STRESS
新增: STORM_WARNING    # 暴雨/大风预警
      FROST_WARNING    # 霜冻预警
      HEAT_STRESS      # 持续高温胁迫
```

### 4.4 ForecastAlert 消息扩展

现有字段保持不变，新增：
```
string alert_source    # LOCAL / WEATHER / HYBRID
```
- `LOCAL`：仅本地传感器触发
- `WEATHER`：仅天气数据触发（如灾害预警）
- `HYBRID`：混合触发

### 4.5 配置文件扩展 (`config/forecast_params.yaml`)

```yaml
forecast_node:
  timer_period_sec: 600
  history_hours: 6
  prediction_hours: 24
  risk_threshold: 0.7
  lwd_margin_hours: 2.0
  humidity_trend_threshold: 0.3
  # 新增
  blend_weight_local: 0.4
  blend_weight_weather: 0.6
  disaster_boost_cap: 0.3
  weather_stale_sec: 21600     # 天气数据过期时间 (6h)
```

---

## 5. 增强包：`sentry_advisory`（修改）

### 5.1 规则引擎扩展

`AdvisoryRule.conditions` 新增天气条件匹配字段：

| 条件字段 | 类型 | 示例值 | 说明 |
|---|---|---|---|
| `disaster_alert_contains` | string | "暴雨" | 灾害预警关键词匹配 |
| `forecast_high_gt` | float | 35 | 未来N天最高温超过 |
| `forecast_low_lt` | float | 5 | 未来N天最低温低于 |
| `forecast_rain_days` | int | 2 | 连续降水天数 ≥ |
| `forecast_temp_range` | bool | true | 适温高湿窗口检测 |

`_match_conditions()` 新增参数 `weather`（WeatherForecast），通过天气条件时查询对应字段。

### 5.2 新增规则样例

```yaml
rules:
  - name: pre_storm_spray
    conditions:
      alert_type: STORM_WARNING
      disaster_alert_contains: "暴雨"
    action:
      action_type: SPRAY
      priority: CRITICAL
      description: "未来有暴雨，建议雨前完成喷药，避免冲刷药效。"
      steps: ["确认药剂库存", "优先喷洒高风险区", "暴雨前6h完成"]

  - name: heat_irrigation
    conditions:
      alert_type: HEAT_STRESS
      forecast_high_gt: 35
      forecast_days: 3
    action:
      action_type: IRRIGATE
      priority: HIGH
      description: "未来3天持续高温超过35°C，建议增加早晚灌溉频次。"
      steps: ["清晨或傍晚灌溉", "检查土壤墒情", "叶片喷水降温"]

  - name: frost_protection
    conditions:
      crop_type: strawberry
      alert_type: FROST_WARNING
    action:
      action_type: PROTECT
      priority: CRITICAL
      description: "霜冻预警，草莓需覆盖保温，必要时大棚加温。"
      steps: ["覆盖无纺布或草帘", "检查大棚密封", "准备加温设备"]
```

### 5.3 `_match_conditions()` 签名变更

```python
# 旧
def _match_conditions(self, cond, fusion, forecast, env, crop_type):
# 新
def _match_conditions(self, cond, fusion, forecast, env, weather, crop_type):
```

---

## 6. 前端增强

### 6.1 位置配置

- 在现有设置面板新增 **"农田位置"** 配置项：纬度/经度输入框
- 修改后通过 ROS2 参数服务写入 `weather_node` 的 `lat`/`lon` 参数
- 参数变更由 `weather_node` 监听并触发立即拉取

### 6.2 天气预报展示

- 在 `env-data-bar` 下方新增可折叠面板：**"未来天气"**
- 展示未来 7 天逐日：天气图标、温度范围、降水量柱状图
- 灾害预警以红色标签形式突出显示

### 6.3 建议卡片增强

- 现有诊断结果卡片中，天气相关建议标注 `[天气]` 来源标签
- 灾害预警类建议卡片优先级置顶

---

## 7. 实施计划要点

1. **新分支** `feat/weather-disease-prediction`
2. 实施顺序：
   - Phase 1: `sentry_interfaces` 新增消息 + `sentry_weather` 包 (独立可测)
   - Phase 2: `sentry_forecast` 混合预测增强
   - Phase 3: `sentry_advisory` 规则引擎扩展 + 新规则
   - Phase 4: 前端配置与展示
3. TDD：每 phase 先写测试再实现
4. 本地开发用 mock_mode=true，板端测试用真实 API

---

## 8. 风险与边界

- **API 配额**：CMA 免费 API 有请求频率限制，3h 拉取一次不会超限
- **网络不可靠**：缓存兜底，stale 标记，不超过 24h
- **预测准确性**：混合模型权重为经验值，后续可根据实测数据调优
- **前端 mock**：沿用现有 `ros.js` 的 `injectMock()` 模式，新增天气 mock 数据
