# 云端 LLM 农情分析 · 设计文档

> 版本 v1.0 · 2026-07-11
> 状态：设计确认，待实现

---

## 1. 目标

在现有规则引擎 + 数学模型基础上，引入云端 LLM（DeepSeek）做综合农情分析，结合传感器、病害诊断、天气预报、趋势预测等多维数据，生成自然语言综合报告和可操作建议。

---

## 2. 决策总览

| 决策 | 选择 |
|------|------|
| LLM 平台 | DeepSeek API (`deepseek-chat`) |
| 触发方式 | 自动轮询（精简 prompt，10min）+ 手动触发（标准 prompt） |
| 展示位置 | 分析页（Tab 3）新增「AI 综合分析」卡片 |
| 架构 | 新建独立 `sentry_llm` ROS2 包，`llm_advisor_node` 节点 |

---

## 3. 系统架构

```
┌─────────────────────────────────────────────────────┐
│  RDK X5                                              │
│                                                      │
│  ┌──────────────────────────────────────────────┐   │
│  │  sentry_llm (新增)                            │   │
│  │  llm_advisor_node                             │   │
│  │  ├─ 订阅: /vision/diagnosis                   │   │
│  │  ├─ 订阅: /sensor/environment_mobile           │   │
│  │  ├─ 订阅: /weather/forecast                   │   │
│  │  ├─ 订阅: /forecast/alert                     │   │
│  │  ├─ 订阅: /fusion/diagnosis                   │   │
│  │  ├─ 订阅: /advisory/action                    │   │
│  │  ├─ Timer: 10min 自动触发 (精简)              │   │
│  │  ├─ Service: /llm/analyze (手动触发, 标准)    │   │
│  │  ├─ Publisher: /llm/analysis                  │   │
│  │  └─ DeepSeek API HTTP Client                  │   │
│  └──────────────────┬───────────────────────────┘   │
│                     │ /llm/analysis                  │
│  ┌──────────────────▼───────────────────────────┐   │
│  │  miniprogram_bridge_node (修改)               │   │
│  │  ├─ 新增订阅 /llm/analysis → WS 'llm' 频道    │   │
│  │  └─ 新增 POST /api/llm/analyze               │   │
│  └──────────────────┬───────────────────────────┘   │
│                     │ WS / HTTP                      │
└─────────────────────┼───────────────────────────────┘
                      │
               ┌──────▼──────┐
               │  微信小程序   │
               │  分析页新增   │
               │  AI 综合卡片 │
               └─────────────┘
```

- `llm_advisor_node` 独立管理 LLM 调用，不阻塞其他节点
- 手动触发走 `POST /api/llm/analyze` → bridge → ROS2 Service → llm_advisor_node
- 自动结果通过 ROS2 topic → bridge WS 推送到小程序
- DeepSeek API Key 通过 ROS2 参数 `api_key` 传入

---

## 4. Prompt 设计

### 4.1 自动轮询（精简版，~500 tokens）

```
你是一个农业AI助手。基于以下当前农田数据做简要分析（100字以内），
给出1-2条最关键的农事建议。

当前时间: {timestamp}
作物: {crop_type}
病害诊断: {disease} (置信度 {confidence}%)
环境: 温度{air_temp}°C 湿度{air_humidity}% CO₂ {co2}ppm
土壤: 温度{soil_temp}°C 湿度{soil_humidity}% N{soil_n} P{soil_p} K{soil_k}
天气: {weather_summary}
风险评分: {risk_score}/1.0
预警: {alert_summary}

请返回JSON: {"summary": "...", "suggestions": ["..."], "risk_level": "low|medium|high"}
```

### 4.2 手动触发（标准版，~1500 tokens）

```
你是一个农业AI助手。基于以下完整农田数据做深入分析（200-300字），
给出具体可操作的农事建议。

当前时间: {timestamp}
作物: {crop_type}

【病害诊断】
诊断结果: {disease} (置信度 {confidence}%)
分类概率: {probabilities_list}

【环境传感器】(移动节点)
空气: 温度{air_temp}°C 湿度{air_humidity}% CO₂ {co2}ppm
土壤: 温度{soil_temp}°C 湿度{soil_humidity}% N{soil_n} P{soil_p} K{soil_k}
叶面湿度: {leaf_wetness}%

【天气预报】
当前: {current_weather}
未来7日: {7day_summary}
未来24h逐时: {24h_summary}
灾害预警: {disaster_alerts}

【趋势预测】
风险评分: {risk_score}/1.0
趋势: {trend_direction}
预警类型: {alert_type}
描述: {forecast_description}

【规则引擎建议】
{advisory_text}

请返回JSON:
{
  "summary": "综合分析...",
  "suggestions": ["具体建议1", "具体建议2", ...],
  "risk_level": "low|medium|high",
  "focus_areas": ["需要重点关注的方面"],
  "next_check": "建议下次检查时间"
}
```

---

## 5. ROS2 消息定义

**新增 `sentry_interfaces/msg/LLMAnalysis.msg`:**

```
std_msgs/Header header
string status          # "ok" | "timeout" | "error" | "parse_error"
string summary         # 综合分析文本
string[] suggestions   # 建议列表
string risk_level      # "low" | "medium" | "high"
string[] focus_areas   # 关注重点
string next_check      # 建议下次检查时间
string raw_text        # 原始响应 (parse_error时使用)
string trigger         # "auto" | "manual"
```

---

## 6. DeepSeek API 配置

```
Endpoint: https://api.deepseek.com/v1/chat/completions
Model: deepseek-chat
Timeout: 30s (精简) / 60s (标准)
Max tokens: 300 (精简) / 800 (标准)
Temperature: 0.7
```

**错误处理：**
| 场景 | status | 行为 |
|------|--------|------|
| API 超时 | `timeout` | 前端显示"AI 请求超时，稍后重试" |
| HTTP 错误 (4xx/5xx) | `error` | 前端显示错误信息，点击重试 |
| JSON 解析失败 | `parse_error` | 降级展示 `raw_text` |

---

## 7. 小程序改动

### 分析页（Tab 3）新增 AI 综合分析卡片

**卡片结构：**
- 顶部状态栏：指示灯（绿=最新 / 黄=加载中 / 红=失败）+ 触发方式（AI 分析 · 自动/X分钟前）+ 风险等级胶囊
- 摘要正文区域（自然语言分析报告）
- 建议列表（有序编号，每条单独一行）
- 关注重点标签行
- 底部「🔍 深度分析」按钮（手动触发标准版分析）
- 加载中显示旋转动画 + "AI 分析中..."

**数据绑定：**
```
自动推送: stores.llmAnalysis → 更新卡片（状态灯变绿）
手动触发: POST /api/llm/analyze → 卡片进入加载态 → 返回后更新
```

---

## 8. 桥接节点改动（`miniprogram_bridge_node`）

- 新增 ROS2 订阅 `LLMAnalysis` on `/llm/analysis` → 推 WS `llm` 频道
- 新增 `POST /api/llm/analyze` → 调用 ROS2 Service `/llm/analyze` → 返回结果

---

## 9. 测试策略

| 层级 | 策略 |
|------|------|
| 消息定义 | `ros2 interface show` 验证 |
| llm_advisor_node | pytest mock DeepSeek API 响应，验证 prompt 拼装和 JSON 解析 |
| bridge 转发 | 验证 WS 推送 'llm' 频道 + REST endpoint 联通 |
| 前端 | 模拟 `llm` 频道数据渲染卡片 |
| 端到端 | 手动触发深度分析，对比自动分析结果 |

---

## 10. 风险

1. **DeepSeek API 可用性**：依赖云端服务，网络中断时降级为规则引擎（现有 advisory/forecast 不受影响）
2. **Token 消耗**：自动模式 10min/次，每次 ~500 tokens，估算月消耗约 ~2M tokens（DeepSeek 百万 token 约 ¥1-2）
3. **延迟**：手动触发需等待 LLM 响应（5-15s），卡片需明确加载态
4. **幻觉**：LLM 可能给出不合理建议，前端提示「AI 建议仅供参考」
