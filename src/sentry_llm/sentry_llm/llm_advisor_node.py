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

from sentry_interfaces.msg import LLMAnalysis


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
            self.api_key = os.environ.get('DEEPSEEK_API_KEY', '')
        if not self.api_key:
            self.get_logger().warn('No DeepSeek API key configured. Set DEEPSEEK_API_KEY env var or api_key parameter.')

        self._diagnosis = None
        self._env = None
        self._weather = None
        self._forecast = None
        self._fusion = None
        self._advisory = None

        self._setup_subscriptions()

        self.pub = self.create_publisher(LLMAnalysis, '/llm/analysis', 10)

        from sentry_interfaces.srv import LLMAnalyze
        self.srv = self.create_service(
            LLMAnalyze, '/llm/analyze', self._on_manual_trigger)

        period = self.get_parameter('auto_period_sec').value
        self.timer = self.create_timer(float(period), self._on_auto_tick)

        self._client = httpx.Client(timeout=TIMEOUT_STANDARD)
        self.get_logger().info('LLM advisor node ready')

    def _setup_subscriptions(self):
        from sentry_interfaces.msg import (
            Diagnosis, Environment, WeatherForecast,
            ForecastAlert, FusionResult, AdvisoryAction,
        )
        qos = rclpy.qos.QoSProfile(depth=10, reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(Diagnosis, '/vision/diagnosis', self._on_diagnosis, 10)
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
            d['weather_summary'] = w.days[0].weather_desc if w.days else '无'
            d['current_weather'] = f'{w.days[0].weather_desc} {w.days[0].temp_high:.0f}-{w.days[0].temp_low:.0f}°C' if w.days else '无'
            d['day7_summary'] = ', '.join(f'{day.weather_desc}{day.temp_high:.0f}°' for day in w.days[:7])
            d['hour24_summary'] = ', '.join(f'+{h.hour_offset}h {h.temp:.0f}°' for h in w.hours[:24])
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
                        'focus_areas': [], 'next_check': '', 'raw_text': resp.text[:500]}
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
