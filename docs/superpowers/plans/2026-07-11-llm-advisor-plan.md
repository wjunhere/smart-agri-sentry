# LLM 农情分析 · 实施计划

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 新增 `sentry_llm` ROS2 包，通过 DeepSeek API 做云端农情分析，自动+手动两种触发模式，结果在微信小程序分析页展示。

**Architecture:** 新建 `llm_advisor_node` 订阅 6 个 ROS2 话题，定时 10min 自动调 LLM（精简 prompt），通过 ROS2 Service 响应手动触发（标准 prompt），结果发布 `/llm/analysis`。`miniprogram_bridge_node` 新增订阅和 REST 端点转发。小程序分析页新增 AI 综合分析卡片。

**Tech Stack:** Python rclpy + `httpx` (async HTTP) + DeepSeek API；TypeScript/less/WXML

---

## File Structure

```
# === 后端 (新增 + 修改) ===
src/sentry_interfaces/msg/LLMAnalysis.msg          # [新增] 消息定义
src/sentry_llm/                                      # [新增] 新 ROS2 包
├── package.xml
├── setup.py / setup.cfg
├── resource/sentry_llm
├── sentry_llm/
│   ├── __init__.py
│   └── llm_advisor_node.py
└── test/
    ├── __init__.py
    └── test_llm_advisor.py
src/sentry_miniprogram/sentry_miniprogram/
    miniprogram_bridge_node.py                       # [修改] 加 LLM 订阅 + Service 端点
src/sentry_bringup/launch/
    miniprogram_bridge.launch.py                     # [修改] 加 sentry_llm 节点

# === 前端 (修改) ===
wechat/miniprogram/pages/analysis/
    analysis.ts                                      # [修改] 加 LLM 数据绑定 + 手动触发
    analysis.wxml                                    # [修改] 加 AI 卡片
    analysis.less                                    # [修改] 加 AI 卡片样式
wechat/miniprogram/services/
    store.ts                                         # [修改] 加 LLM 状态字段
    ws.ts                                            # [修改] 加 llm 消息处理
    api.ts                                           # [修改] 加 llmAnalyze 方法
```

---

### Task 1: 新增 LLMAnalysis 消息定义

**Files:**
- Create: `src/sentry_interfaces/msg/LLMAnalysis.msg`

- [ ] **Step 1: 创建消息文件**

```msg
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

- [ ] **Step 2: 验证消息格式**

```bash
# 语法检查（需板端执行）
# 后续 colcon build 会自动编译消息
```

- [ ] **Step 3: 提交**

```bash
git add src/sentry_interfaces/msg/LLMAnalysis.msg
git commit -m "feat: add LLMAnalysis message for LLM advisor results"
```

---

### Task 2: 创建 sentry_llm 包骨架

**Files:**
- Create: `src/sentry_llm/package.xml`
- Create: `src/sentry_llm/setup.py`
- Create: `src/sentry_llm/setup.cfg`
- Create: `src/sentry_llm/sentry_llm/__init__.py`
- Create: `src/sentry_llm/test/__init__.py`

- [ ] **Step 1: 创建目录结构**

```bash
mkdir -p src/sentry_llm/sentry_llm src/sentry_llm/test src/sentry_llm/resource
touch src/sentry_llm/sentry_llm/__init__.py src/sentry_llm/test/__init__.py
touch src/sentry_llm/resource/sentry_llm
```

- [ ] **Step 2: 创建 package.xml**

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>sentry_llm</name>
  <version>0.1.0</version>
  <description>Cloud LLM advisor node — DeepSeek-powered agricultural analysis</description>
  <maintainer email="wjun@example.com">wjun</maintainer>
  <license>MIT</license>

  <depend>rclpy</depend>
  <depend>sentry_interfaces</depend>
  <depend>std_msgs</depend>
  <depend>std_srvs</depend>

  <test_depend>python3-pytest</test_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
```

- [ ] **Step 3: 创建 setup.py**

```python
from setuptools import setup

package_name = 'sentry_llm'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='wjun',
    maintainer_email='wjun@example.com',
    description='Cloud LLM advisor node for agricultural analysis',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'llm_advisor_node = sentry_llm.llm_advisor_node:main',
        ],
    },
)
```

- [ ] **Step 4: 创建 setup.cfg**

```ini
[develop]
script_dir=$base/lib/sentry_llm
[install]
install_scripts=$base/lib/sentry_llm
```

- [ ] **Step 5: 提交**

```bash
git add src/sentry_llm/
git commit -m "feat: scaffold sentry_llm ROS2 package"
```

---

### Task 3: 实现 llm_advisor_node 核心逻辑

**Files:**
- Create: `src/sentry_llm/sentry_llm/llm_advisor_node.py`

- [ ] **Step 1: 写测试 — 验证 prompt 构建和 JSON 解析**

```python
# test/test_llm_advisor.py
import pytest
from unittest.mock import MagicMock, patch
import json

# Mock before import
import sys
mock_rclpy = MagicMock()
mock_rclpy.node = MagicMock()
sys.modules['rclpy'] = mock_rclpy
sys.modules['rclpy.node'] = MagicMock()
sys.modules['std_msgs'] = MagicMock()
sys.modules['std_msgs.msg'] = MagicMock()
sys.modules['std_srvs'] = MagicMock()
sys.modules['std_srvs.srv'] = MagicMock()
sys.modules['sentry_interfaces'] = MagicMock()
sys.modules['sentry_interfaces.msg'] = MagicMock()
sys.modules['sentry_interfaces.srv'] = MagicMock()
sys.modules['httpx'] = MagicMock()

from sentry_llm.llm_advisor_node import (
    build_compact_prompt,
    build_standard_prompt,
    parse_llm_response,
    LLMAdvisorNode,
)


def test_build_compact_prompt():
    data = {
        'crop_type': 'tomato',
        'disease': '早疫病',
        'confidence': 0.942,
        'air_temp': 26.3,
        'air_humidity': 68.5,
        'co2': 412,
        'soil_temp': 22.1,
        'soil_humidity': 42.3,
        'soil_n': 45.0,
        'soil_p': 32.0,
        'soil_k': 58.0,
        'weather_summary': '晴 28°C',
        'risk_score': 0.7,
        'alert_summary': 'RISING_RISK',
    }
    prompt = build_compact_prompt(data)
    assert 'tomato' in prompt
    assert '早疫病' in prompt
    assert '26.3' in prompt
    assert '100字以内' in prompt
    assert 'JSON' in prompt


def test_build_standard_prompt():
    data = {
        'crop_type': 'tomato',
        'disease': '早疫病',
        'confidence': 0.942,
        'probabilities_list': '早疫病:94.2%, 晚疫病:3.1%, 白粉病:1.2%',
        'air_temp': 26.3,
        'air_humidity': 68.5,
        'co2': 412,
        'soil_temp': 22.1,
        'soil_humidity': 42.3,
        'soil_n': 45.0,
        'soil_p': 32.0,
        'soil_k': 58.0,
        'leaf_wetness': 34.2,
        'current_weather': '晴 28°C 湿度45%',
        'day7_summary': '周一晴28° 周二阴26°...',
        'hour24_summary': '14:00晴29° 15:00阴28°...',
        'disaster_alerts': '暴雨蓝色预警',
        'risk_score': 0.7,
        'trend_direction': '上升',
        'alert_type': 'RISING_RISK',
        'forecast_description': '24h风险0.7呈上升趋势',
        'advisory_text': '喷施代森锰锌600倍液',
    }
    prompt = build_standard_prompt(data)
    assert 'tomato' in prompt
    assert '早疫病' in prompt
    assert '300字' in prompt or '深入分析' in prompt
    assert '代森锰锌' in prompt
    assert 'focus_areas' in prompt


def test_parse_llm_response_ok():
    raw = '{"summary": "当前早疫病风险较高", "suggestions": ["立即施药", "加强通风"], "risk_level": "high"}'
    result = parse_llm_response(raw)
    assert result['status'] == 'ok'
    assert result['summary'] == '当前早疫病风险较高'
    assert len(result['suggestions']) == 2
    assert result['risk_level'] == 'high'


def test_parse_llm_response_invalid_json():
    raw = 'not json at all'
    result = parse_llm_response(raw)
    assert result['status'] == 'parse_error'
    assert result['raw_text'] == 'not json at all'
```

- [ ] **Step 2: 运行测试 — 验证失败**

```bash
cd src/sentry_llm && python -m pytest test/test_llm_advisor.py -v
```
Expected: all 4 tests FAIL with ImportError (node doesn't exist yet)

- [ ] **Step 3: 实现 llm_advisor_node.py**

```python
#!/usr/bin/env python3
"""LLM Advisor Node — DeepSeek-powered agricultural analysis.

Triggers:
- Auto (10min timer): compact prompt, result published to /llm/analysis
- Manual (ROS2 Service): standard prompt, result returned in response
"""

import json
import os
import threading
import time

import httpx
import rclpy
from rclpy.node import Node
from rclpy.service import Service

from sentry_interfaces.msg import LLMAnalysis
from sentry_interfaces.srv import LLMAnalyze


DEEPSEEK_URL = 'https://api.deepseek.com/v1/chat/completions'
MODEL = 'deepseek-chat'
COMPACT_MAX_TOKENS = 300
STANDARD_MAX_TOKENS = 800
TIMEOUT_COMPACT = 30.0
TIMEOUT_STANDARD = 60.0


def build_compact_prompt(data: dict) -> str:
    return f"""你是一个农业AI助手。基于以下当前农田数据做简要分析（100字以内），
给出1-2条最关键的农事建议。

当前时间: {data.get('timestamp', '')}
作物: {data.get('crop_type', '未知')}
病害诊断: {data.get('disease', '无')} (置信度 {data.get('confidence', 0)*100:.1f}%)
环境: 温度{data.get('air_temp', '--')}°C 湿度{data.get('air_humidity', '--')}% CO₂ {data.get('co2', '--')}ppm
土壤: 温度{data.get('soil_temp', '--')}°C 湿度{data.get('soil_humidity', '--')}% N{data.get('soil_n', '--')} P{data.get('soil_p', '--')} K{data.get('soil_k', '--')}
天气: {data.get('weather_summary', '无')}
风险评分: {data.get('risk_score', 0)}/1.0
预警: {data.get('alert_summary', '无')}

请返回JSON: {{"summary": "...", "suggestions": ["..."], "risk_level": "low|medium|high"}}"""


def build_standard_prompt(data: dict) -> str:
    return f"""你是一个农业AI助手。基于以下完整农田数据做深入分析（200-300字），
给出具体可操作的农事建议。

当前时间: {data.get('timestamp', '')}
作物: {data.get('crop_type', '未知')}

【病害诊断】
诊断结果: {data.get('disease', '无')} (置信度 {data.get('confidence', 0)*100:.1f}%)
分类概率: {data.get('probabilities_list', '无')}

【环境传感器】(移动节点)
空气: 温度{data.get('air_temp', '--')}°C 湿度{data.get('air_humidity', '--')}% CO₂ {data.get('co2', '--')}ppm
土壤: 温度{data.get('soil_temp', '--')}°C 湿度{data.get('soil_humidity', '--')}% N{data.get('soil_n', '--')} P{data.get('soil_p', '--')} K{data.get('soil_k', '--')}
叶面湿度: {data.get('leaf_wetness', '--')}%

【天气预报】
当前: {data.get('current_weather', '无')}
未来7日: {data.get('day7_summary', '无')}
未来24h逐时: {data.get('hour24_summary', '无')}
灾害预警: {data.get('disaster_alerts', '无')}

【趋势预测】
风险评分: {data.get('risk_score', 0)}/1.0
趋势: {data.get('trend_direction', '平稳')}
预警类型: {data.get('alert_type', '无')}
描述: {data.get('forecast_description', '无')}

【规则引擎建议】
{data.get('advisory_text', '无')}

请返回JSON:
{{
  "summary": "综合分析...",
  "suggestions": ["具体建议1", "具体建议2", ...],
  "risk_level": "low|medium|high",
  "focus_areas": ["需要重点关注的方面"],
  "next_check": "建议下次检查时间"
}}"""


def parse_llm_response(raw: str) -> dict:
    """Parse LLM JSON response. Falls back to raw_text on failure."""
    try:
        # Handle markdown code fences: ```json ... ```
        text = raw.strip()
        if text.startswith('```'):
            lines = text.split('\n')
            text = '\n'.join(lines[1:-1])
        result = json.loads(text)
        result['status'] = 'ok'
        return result
    except (json.JSONDecodeError, Exception):
        return {
            'status': 'parse_error',
            'summary': '',
            'suggestions': [],
            'risk_level': 'low',
            'focus_areas': [],
            'next_check': '',
            'raw_text': raw,
        }


class LLMAdvisorNode(Node):
    def __init__(self):
        super().__init__('llm_advisor_node')

        self.declare_parameter('api_key', '')
        self.declare_parameter('auto_period_sec', 600)

        self.api_key = self.get_parameter('api_key').value
        if not self.api_key:
            self.get_logger().warn('No DeepSeek API key configured. Set api_key parameter.')

        # Data cache
        self._diagnosis = None
        self._env = None
        self._weather = None
        self._forecast = None
        self._fusion = None
        self._advisory = None
        self._lock = threading.Lock()

        self._setup_subscriptions()

        # Publisher
        self.pub = self.create_publisher(LLMAnalysis, '/llm/analysis', 10)

        # Service for manual trigger
        self.srv = self.create_service(
            LLMAnalyze, '/llm/analyze', self._on_manual_trigger)

        # Auto timer
        period = self.get_parameter('auto_period_sec').value
        self.timer = self.create_timer(float(period), self._on_auto_tick)

        self._client = httpx.Client(timeout=TIMEOUT_STANDARD)

        self.get_logger().info('LLM advisor node ready')

    def _setup_subscriptions(self):
        from sentry_interfaces.msg import (
            Diagnosis, Environment, WeatherForecast,
            ForecastAlert, FusionResult, AdvisoryAction,
        )
        self.create_subscription(Diagnosis, '/vision/diagnosis', self._on_diagnosis, 10)
        qos = rclpy.qos.QoSProfile(depth=10, reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(Environment, '/sensor/environment_mobile', self._on_env, qos)
        self.create_subscription(WeatherForecast, '/weather/forecast', self._on_weather, 10)
        self.create_subscription(ForecastAlert, '/forecast/alert', self._on_forecast, 10)
        self.create_subscription(FusionResult, '/fusion/diagnosis', self._on_fusion, 10)
        self.create_subscription(AdvisoryAction, '/advisory/action', self._on_advisory, 10)

    def _on_diagnosis(self, msg): self._diagnosis = msg
    def _on_env(self, msg): self._env = msg
    def _on_weather(self, msg): self._weather = msg
    def _on_forecast(self, msg): self._forecast = msg
    def _on_fusion(self, msg): self._fusion = msg
    def _on_advisory(self, msg): self._advisory = msg

    def _collect_data(self) -> dict:
        """Gather latest data snapshot for prompt building."""
        d = {
            'timestamp': time.strftime('%Y-%m-%d %H:%M'),
            'crop_type': '未知',
            'disease': '无',
            'confidence': 0.0,
            'air_temp': '--',
            'air_humidity': '--',
            'co2': '--',
            'soil_temp': '--',
            'soil_humidity': '--',
            'soil_n': '--',
            'soil_p': '--',
            'soil_k': '--',
            'leaf_wetness': '--',
            'weather_summary': '无',
            'current_weather': '无',
            'day7_summary': '无',
            'hour24_summary': '无',
            'disaster_alerts': '无',
            'risk_score': 0.0,
            'trend_direction': '平稳',
            'alert_type': '无',
            'forecast_description': '无',
            'advisory_text': '无',
            'probabilities_list': '无',
        }

        if self._diagnosis:
            d.update({
                'crop_type': self._diagnosis.crop_type,
                'disease': self._diagnosis.disease_class,
                'confidence': self._diagnosis.confidence,
                'probabilities_list': ', '.join(
                    f'{p:.1%}' for p in (self._diagnosis.probabilities or [])),
            })

        if self._env:
            d.update({
                'air_temp': f'{self._env.air_temp:.1f}',
                'air_humidity': f'{self._env.air_humidity:.1f}',
                'co2': f'{self._env.air_co2:.0f}',
                'soil_temp': f'{self._env.soil_temp:.1f}',
                'soil_humidity': f'{self._env.soil_humidity:.1f}',
                'leaf_wetness': f'{self._env.leaf_wetness:.1f}',
            })

        if self._weather:
            w = self._weather
            d['weather_summary'] = w.days[0].desc if w.days else '无'
            d['current_weather'] = f'{w.days[0].desc} {w.days[0].high}-{w.days[0].low}°C' if w.days else '无'
            d['day7_summary'] = ', '.join(f'{day.desc}{day.high}°' for day in w.days)
            d['hour24_summary'] = ', '.join(f'{h.time}{h.temp}°{h.desc}' for h in w.hours[:24])
            d['disaster_alerts'] = ', '.join(w.disaster_alerts) if w.disaster_alerts else '无'

        if self._forecast:
            d.update({
                'risk_score': f'{self._forecast.probability:.2f}',
                'trend_direction': '上升' if self._forecast.probability > 0.5 else '下降',
                'alert_type': self._forecast.alert_type,
                'forecast_description': self._forecast.description,
            })

        if self._fusion:
            d['risk_score'] = f'{self._fusion.risk_score:.2f}'

        if self._advisory:
            d['advisory_text'] = self._advisory.description

        return d

    def _call_llm(self, prompt: str, max_tokens: int, timeout: float) -> dict:
        """Call DeepSeek API, return parsed result."""
        if not self.api_key:
            return {'status': 'error', 'summary': 'API Key 未配置', 'suggestions': [],
                    'risk_level': 'low', 'focus_areas': [], 'next_check': '', 'raw_text': ''}

        try:
            resp = self._client.post(
                DEEPSEEK_URL,
                headers={
                    'Authorization': f'Bearer {self.api_key}',
                    'Content-Type': 'application/json',
                },
                json={
                    'model': MODEL,
                    'messages': [{'role': 'user', 'content': prompt}],
                    'max_tokens': max_tokens,
                    'temperature': 0.7,
                },
                timeout=timeout,
            )
            if resp.status_code != 200:
                return {'status': 'error', 'summary': f'API 错误 {resp.status_code}',
                        'suggestions': [], 'risk_level': 'low',
                        'focus_areas': [], 'next_check': '', 'raw_text': resp.text}

            content = resp.json()['choices'][0]['message']['content']
            result = parse_llm_response(content)
            return result

        except httpx.TimeoutException:
            return {'status': 'timeout', 'summary': 'AI 请求超时', 'suggestions': [],
                    'risk_level': 'low', 'focus_areas': [], 'next_check': '', 'raw_text': ''}
        except Exception as e:
            return {'status': 'error', 'summary': f'请求异常: {str(e)}', 'suggestions': [],
                    'risk_level': 'low', 'focus_areas': [], 'next_check': '', 'raw_text': ''}

    def _publish_result(self, result: dict, trigger: str):
        msg = LLMAnalysis()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.status = result.get('status', 'error')
        msg.summary = result.get('summary', '')
        msg.suggestions = result.get('suggestions', [])
        msg.risk_level = result.get('risk_level', 'low')
        msg.focus_areas = result.get('focus_areas', [])
        msg.next_check = result.get('next_check', '')
        msg.raw_text = result.get('raw_text', '')
        msg.trigger = trigger
        self.pub.publish(msg)

    def _on_auto_tick(self):
        data = self._collect_data()
        prompt = build_compact_prompt(data)
        self.get_logger().info('Auto LLM analysis triggered')
        result = self._call_llm(prompt, COMPACT_MAX_TOKENS, TIMEOUT_COMPACT)
        self._publish_result(result, 'auto')

    def _on_manual_trigger(self, request, response):
        data = self._collect_data()
        prompt = build_standard_prompt(data)
        self.get_logger().info('Manual LLM analysis triggered')
        result = self._call_llm(prompt, STANDARD_MAX_TOKENS, TIMEOUT_STANDARD)

        response.status = result.get('status', 'error')
        response.summary = result.get('summary', '')
        response.suggestions = result.get('suggestions', [])
        response.risk_level = result.get('risk_level', 'low')
        response.focus_areas = result.get('focus_areas', [])
        response.next_check = result.get('next_check', '')
        response.raw_text = result.get('raw_text', '')
        response.trigger = 'manual'
        return response

    def destroy_node(self):
        self._client.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = LLMAdvisorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
```

- [ ] **Step 4: 运行测试 — 验证通过**

```bash
cd src/sentry_llm && python -m pytest test/test_llm_advisor.py -v
```
Expected: all 4 tests PASS

- [ ] **Step 5: 提交**

```bash
git add src/sentry_llm/
git commit -m "feat: implement llm_advisor_node with DeepSeek API integration"
```

---

### Task 4: 新增 LLMAnalyze Service 定义

**Files:**
- Create: `src/sentry_interfaces/srv/LLMAnalyze.srv`

- [ ] **Step 1: 创建 Service 文件**

```srv
---
string status          # "ok" | "timeout" | "error" | "parse_error"
string summary
string[] suggestions
string risk_level
string[] focus_areas
string next_check
string raw_text
string trigger
```

- [ ] **Step 2: 提交**

```bash
git add src/sentry_interfaces/srv/LLMAnalyze.srv
git commit -m "feat: add LLMAnalyze service for manual LLM trigger"
```

---

### Task 5: 修改 miniprogram_bridge_node 对接 LLM

**Files:**
- Modify: `src/sentry_miniprogram/sentry_miniprogram/miniprogram_bridge_node.py`

- [ ] **Step 1: 在 `_setup_subscriptions` 末尾加 LLM 订阅**

在 `_setup_subscriptions` 方法末尾加入：
```python
        from sentry_interfaces.msg import LLMAnalysis as LLMAnalysisMsg
        self.create_subscription(
            LLMAnalysisMsg, '/llm/analysis',
            self._on_llm_analysis, 10)
```

- [ ] **Step 2: 加 LLM 回调**

在 `MiniProgramBridgeNode` 类中加入：
```python
    def _on_llm_analysis(self, msg):
        self._push_ws({
            'type': 'llm',
            'ts': self._now_ms(),
            'data': {
                'status': msg.status,
                'summary': msg.summary,
                'suggestions': list(msg.suggestions),
                'risk_level': msg.risk_level,
                'focus_areas': list(msg.focus_areas),
                'next_check': msg.next_check,
                'trigger': msg.trigger,
            }
        })
```

- [ ] **Step 3: 在 `get_app()` 中加手动触发端点**

在 `get_app()` 函数中，`@_app.get('/api/camera')` 之前加入：
```python
    @_app.post('/api/llm/analyze')
    async def api_llm_analyze():
        """Manually trigger a standard-depth LLM analysis via ROS2 Service."""
        if _node is None:
            return {'status': 'error', 'summary': 'Bridge node not ready'}
        # Call /llm/analyze service
        from sentry_interfaces.srv import LLMAnalyze
        if not hasattr(_node, '_llm_srv_client') or _node._llm_srv_client is None:
            _node._llm_srv_client = _node.create_client(LLMAnalyze, '/llm/analyze')

        srv = _node._llm_srv_client
        if not srv.wait_for_service(timeout_sec=5.0):
            return {'status': 'error', 'summary': 'LLM service not available'}

        req = LLMAnalyze.Request()
        future = srv.call_async(req)
        event = threading.Event()
        result = {}

        def done_cb(fut):
            try:
                resp = fut.result()
                result['status'] = resp.status
                result['summary'] = resp.summary
                result['suggestions'] = list(resp.suggestions)
                result['risk_level'] = resp.risk_level
                result['focus_areas'] = list(resp.focus_areas)
                result['next_check'] = resp.next_check
                result['trigger'] = resp.trigger
            except Exception as e:
                result['status'] = 'error'
                result['summary'] = str(e)
            finally:
                event.set()

        future.add_done_callback(done_cb)
        if not event.wait(timeout=65.0):
            return {'status': 'timeout', 'summary': 'LLM request timed out'}
        return result
```

- [ ] **Step 4: 语法检查**

```bash
python -c "import py_compile; py_compile.compile('src/sentry_miniprogram/sentry_miniprogram/miniprogram_bridge_node.py', doraise=True)" && echo "SYNTAX OK"
```

- [ ] **Step 5: 提交**

```bash
git add src/sentry_miniprogram/
git commit -m "feat: add LLM subscription and manual trigger endpoint to bridge node"
```

---

### Task 6: 前端 — store/ws/api 加 LLM 支持

**Files:**
- Modify: `wechat/miniprogram/services/store.ts`
- Modify: `wechat/miniprogram/services/ws.ts`
- Modify: `wechat/miniprogram/services/api.ts`

- [ ] **Step 1: store.ts 加 LLM 状态字段**

在 `store` 对象中（`cropType: 'tomato',` 的下一行）加入：
```typescript
  // LLM analysis
  llmStatus: '',
  llmSummary: '',
  llmSuggestions: [] as string[],
  llmRiskLevel: 'low',
  llmFocusAreas: [] as string[],
  llmNextCheck: '',
  llmTrigger: '',
  llmLoading: false,
```

- [ ] **Step 2: ws.ts 加 llm 消息处理**

在 `handleMessage` 的 `switch` 块中（`case 'alert':` 之后）加入：
```typescript
    case 'llm':
      updateStore({
        llmStatus: data.status,
        llmSummary: data.summary,
        llmSuggestions: data.suggestions || [],
        llmRiskLevel: data.risk_level,
        llmFocusAreas: data.focus_areas || [],
        llmNextCheck: data.next_check,
        llmTrigger: data.trigger,
        llmLoading: false,
      });
      break;
```

- [ ] **Step 3: api.ts 加 llmAnalyze 方法**

在 `api.ts` 末尾加入：
```typescript
export function apiLLMAnalyze() {
  return request<any>('POST', '/api/llm/analyze');
}
```

- [ ] **Step 4: 提交**

```bash
git add wechat/miniprogram/services/
git commit -m "feat: add LLM state, WS handler, and API method for analysis"
```

---

### Task 7: 前端 — 分析页加 AI 综合分析卡片

**Files:**
- Modify: `wechat/miniprogram/pages/analysis/analysis.ts`
- Modify: `wechat/miniprogram/pages/analysis/analysis.wxml`
- Modify: `wechat/miniprogram/pages/analysis/analysis.less`

- [ ] **Step 1: 修改 analysis.json 注册组件**

将 `analysis.json` 改为：
```json
{ "usingComponents": { "alert-bar": "/components/alert-bar/alert-bar" } }
```
（无需改动，alert-bar 已注册）

- [ ] **Step 2: 修改 analysis.ts 加 LLM 数据绑定和手动触发**

在 `data` 中加入：
```typescript
    llmStatus: '',
    llmSummary: '',
    llmSuggestions: [] as string[],
    llmRiskLevel: 'low',
    llmFocusAreas: [] as string[],
    llmNextCheck: '',
    llmTrigger: '',
    llmLoading: false,
```

在 `sync` 方法中加入：
```typescript
        llmStatus: s.llmStatus,
        llmSummary: s.llmSummary,
        llmSuggestions: s.llmSuggestions || [],
        llmRiskLevel: s.llmRiskLevel,
        llmFocusAreas: s.llmFocusAreas || [],
        llmNextCheck: s.llmNextCheck,
        llmTrigger: s.llmTrigger,
        llmLoading: s.llmLoading,
```

在 `methods` 中加入：
```typescript
    onDeepAnalysis() {
      if (this.data.llmLoading) return;
      this.setData({ llmLoading: true });
      const { apiLLMAnalyze } = require('../../services/api');
      apiLLMAnalyze().then((res: any) => {
        this.setData({
          llmLoading: false,
          llmStatus: res.status || 'error',
          llmSummary: res.summary || '',
          llmSuggestions: res.suggestions || [],
          llmRiskLevel: res.risk_level || 'low',
          llmFocusAreas: res.focus_areas || [],
          llmNextCheck: res.next_check || '',
          llmTrigger: 'manual',
        });
      }).catch(() => {
        this.setData({ llmLoading: false, llmStatus: 'error', llmSummary: '请求失败，请重试' });
      });
    },
```

- [ ] **Step 3: 修改 analysis.wxml 加 AI 卡片**

在 `</view>` 闭合标签之前（`alert-bar` 之后）加入：
```xml
  <view class="card llm-card" wx:if="{{llmSummary || llmLoading}}">
    <view class="llm-header">
      <text class="card-header">🤖 AI 综合分析</text>
      <view class="llm-status">
        <view class="llm-dot {{llmLoading ? 'llm-dot-loading' : (llmStatus === 'ok' ? 'llm-dot-ok' : 'llm-dot-err')}}"></view>
        <text wx:if="{{llmLoading}}" class="amber">分析中...</text>
        <text wx:elif="{{llmStatus === 'ok'}}" class="green">最新分析</text>
        <text wx:else class="red">分析失败</text>
        <text wx:if="{{llmTrigger}}" class="muted" style="margin-left:8rpx">· {{llmTrigger === 'auto' ? '自动' : '手动'}}</text>
      </view>
    </view>

    <block wx:if="{{llmLoading}}">
      <view class="llm-loading">
        <text class="mono dim">AI 分析中，请稍候...</text>
      </view>
    </block>

    <block wx:elif="{{llmStatus === 'ok'}}">
      <text class="llm-summary">{{llmSummary}}</text>
      <view class="llm-suggestions" wx:if="{{llmSuggestions.length}}">
        <text class="dim" style="font-size:22rpx;font-weight:600;margin-bottom:8rpx">📋 建议</text>
        <text class="llm-step" wx:for="{{llmSuggestions}}" wx:key="*this">{{index+1}}. {{item}}</text>
      </view>
      <view class="llm-tags" wx:if="{{llmFocusAreas.length}}">
        <text class="llm-tag" wx:for="{{llmFocusAreas}}" wx:key="*this">{{item}}</text>
      </view>
      <view class="llm-footer" wx:if="{{llmNextCheck}}">
        <text class="muted" style="font-size:20rpx">⏰ {{llmNextCheck}}</text>
      </view>
    </block>

    <block wx:else>
      <text class="muted" style="font-size:22rpx">{{llmSummary || '分析失败，请重试'}}</text>
      <view class="llm-retry" bindtap="onDeepAnalysis">
        <text class="blue" style="font-size:22rpx">🔄 点击重试</text>
      </view>
    </block>

    <view class="llm-action" wx:if="{{!llmLoading && llmStatus === 'ok'}}">
      <view class="btn-deep" bindtap="onDeepAnalysis">
        <text style="font-weight:600">🔍 深度分析</text>
      </view>
    </view>
  </view>
```

- [ ] **Step 4: 修改 analysis.less 加 AI 卡片样式**

在文件末尾加入：
```less
.llm-card { margin-top: 0; border-left: 3px solid #A78BFA; }
.llm-header { display: flex; justify-content: space-between; align-items: center; margin-bottom: 12rpx; }
.llm-status { display: flex; align-items: center; gap: 8rpx; font-size: 20rpx; }
.llm-dot { width: 12rpx; height: 12rpx; border-radius: 50%; }
.llm-dot-ok { background: var(--green); }
.llm-dot-err { background: var(--red); }
.llm-dot-loading { background: var(--amber); animation: pulse 1s ease-in-out infinite; }

.llm-loading {
  display: flex; align-items: center; justify-content: center;
  padding: 40rpx; font-size: 24rpx;
}

.llm-summary {
  font-size: 26rpx; line-height: 1.7; color: var(--text);
  display: block; margin-bottom: 16rpx;
}

.llm-suggestions { margin-bottom: 16rpx; }
.llm-step {
  font-size: 24rpx; color: var(--text-dim); line-height: 1.8;
  display: block; padding-left: 8rpx;
}

.llm-tags { display: flex; flex-wrap: wrap; gap: 8rpx; margin-bottom: 12rpx; }
.llm-tag {
  font-size: 18rpx; padding: 4rpx 12rpx; border-radius: var(--radius-pill);
  background: rgba(167,139,250,0.1); color: var(--purple);
}

.llm-footer { margin-bottom: 8rpx; }

.llm-retry { padding: 16rpx; text-align: center; }

.llm-action { text-align: center; margin-top: 16rpx; }
.btn-deep {
  display: inline-block; padding: 16rpx 48rpx;
  background: rgba(167,139,250,0.1); border: 1px solid var(--purple);
  border-radius: var(--radius); color: var(--purple);
}

@keyframes pulse {
  0%, 100% { opacity: 1; }
  50% { opacity: 0.3; }
}
```

- [ ] **Step 5: 提交**

```bash
git add wechat/miniprogram/pages/analysis/
git commit -m "feat: add AI comprehensive analysis card to analysis page"
```

---

### Task 8: Launch 文件 + Bringup 集成

**Files:**
- Modify: `src/sentry_bringup/launch/miniprogram_bridge.launch.py`

- [ ] **Step 1: 修改 launch 文件，加 llm_advisor_node**

```python
"""Launch miniprogram_bridge_node + llm_advisor_node."""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='sentry_miniprogram',
            executable='miniprogram_bridge_node',
            name='miniprogram_bridge_node',
            output='screen',
            parameters=[],
        ),
        Node(
            package='sentry_llm',
            executable='llm_advisor_node',
            name='llm_advisor_node',
            output='screen',
            parameters=[{
                'api_key': '',
                'auto_period_sec': 600,
            }],
        ),
    ])
```

- [ ] **Step 2: 提交**

```bash
git add src/sentry_bringup/launch/miniprogram_bridge.launch.py
git commit -m "feat: add llm_advisor_node to miniprogram bridge launch"
```

---

### Task 9: 板端部署 + 验证

- [ ] **Step 1: 推送代码**

```bash
git push origin feature/wechat-miniprogram
```

- [ ] **Step 2: 板端拉取 + 构建**

```bash
ssh rdk "export PATH=\$PATH:/home/sunrise/.local/bin && cd ~/dev_ws && git pull origin feature/wechat-miniprogram && source /opt/tros/humble/setup.bash && colcon build --packages-select sentry_interfaces sentry_llm sentry_miniprogram --symlink-install"
```

- [ ] **Step 3: 在板端安装 httpx**

```bash
ssh rdk "pip3 install httpx"
```

- [ ] **Step 4: 设置 API Key 并启动测试**

```bash
# 编辑 launch 文件填入 api_key，或启动时传入参数
ssh rdk "export PATH=\$PATH:/home/sunrise/.local/bin && cd ~/dev_ws && source install/setup.bash && ros2 run sentry_llm llm_advisor_node --ros-args -p api_key:='sk-xxx'"
```

- [ ] **Step 5: 验证消息发布**

```bash
ssh rdk "source /opt/tros/humble/setup.bash && ros2 topic echo /llm/analysis --once"
```

Expected: 手动调用 Service 后能看到分析结果

---

### Task 10: 测试 — llm_advisor_node 集成测试

**Files:**
- Modify: `src/sentry_llm/test/test_llm_advisor.py`（追加）

在已有测试文件末尾追加：
```python
def test_build_compact_prompt_missing_data():
    data = {'crop_type': 'wheat'}
    prompt = build_compact_prompt(data)
    assert 'wheat' in prompt
    assert '--' in prompt  # missing values use '--'


def test_build_standard_prompt_minimal():
    data = {'crop_type': 'strawberry'}
    prompt = build_standard_prompt(data)
    assert 'strawberry' in prompt
    assert '无' in prompt


def test_parse_llm_response_with_fences():
    raw = '```json\n{"summary": "test", "suggestions": [], "risk_level": "low"}\n```'
    result = parse_llm_response(raw)
    assert result['status'] == 'ok'
    assert result['summary'] == 'test'
```

运行：
```bash
cd src/sentry_llm && python -m pytest test/ -v
```
Expected: all 7 tests PASS

---

## Self-Review

**Spec coverage:**
| Spec Section | Task |
|---|---|
| 3. Architecture | Tasks 1-5, 8 |
| 4. Prompt design | Task 3 (build_compact_prompt, build_standard_prompt) |
| 5. Message definition | Task 1, 4 |
| 6. DeepSeek config | Task 3 (_call_llm) |
| 7. Mini-program changes | Tasks 6, 7 |
| 8. Bridge node changes | Task 5 |
| 9. Testing | Tasks 3, 10 |

**Placeholder scan:** No TBDs. All code is concrete. API key placeholder `sk-xxx` is intentional (user fills in).

**Type consistency:** 
- `LLMAnalysis.msg` fields match `_publish_result()` usage
- `LLMAnalyze.srv` response fields match `_on_manual_trigger()` response
- Store.ts `llm*` fields match ws.ts handler and analysis page bindings
