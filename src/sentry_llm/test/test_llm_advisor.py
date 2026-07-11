"""Tests for llm_advisor_node."""
import pytest
import sys
from unittest.mock import MagicMock

mock_rclpy = MagicMock()
mock_rclpy.node = MagicMock()
mock_rclpy.qos = MagicMock()
mock_rclpy.qos.QoSProfile = MagicMock(return_value=10)
mock_rclpy.qos.ReliabilityPolicy = MagicMock()
sys.modules['rclpy'] = mock_rclpy
sys.modules['rclpy.node'] = MagicMock()
sys.modules['rclpy.qos'] = MagicMock()
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
)


def test_build_compact_prompt():
    data = {
        'crop_type': 'tomato',
        'disease': '早疫病',
        'confidence': 0.942,
        'air_temp': '26.3',
        'air_humidity': '68.5',
        'co2': '412',
        'soil_temp': '22.1',
        'soil_humidity': '42.3',
        'soil_n': '45.0',
        'soil_p': '32.0',
        'soil_k': '58.0',
        'weather_summary': '晴 28°C',
        'risk_score': '0.70',
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
        'probabilities_list': '早疫病:94.2%, 晚疫病:3.1%',
        'air_temp': '26.3',
        'air_humidity': '68.5',
        'co2': '412',
        'soil_temp': '22.1',
        'soil_humidity': '42.3',
        'soil_n': '45.0',
        'soil_p': '32.0',
        'soil_k': '58.0',
        'leaf_wetness': '34.2',
        'current_weather': '晴 28°C',
        'day7_summary': '周一晴...',
        'hour24_summary': '14:00晴...',
        'disaster_alerts': '暴雨蓝色预警',
        'risk_score': '0.70',
        'trend_direction': '上升',
        'alert_type': 'RISING_RISK',
        'forecast_description': '24h风险0.7',
        'advisory_text': '喷施代森锰锌600倍液',
    }
    prompt = build_standard_prompt(data)
    assert 'tomato' in prompt
    assert '早疫病' in prompt
    assert '300字' in prompt or '深入分析' in prompt
    assert '代森锰锌' in prompt
    assert 'focus_areas' in prompt


def test_parse_llm_response_ok():
    raw = '{"summary": "早疫病风险较高", "suggestions": ["立即施药", "加强通风"], "risk_level": "high"}'
    result = parse_llm_response(raw)
    assert result['status'] == 'ok'
    assert result['summary'] == '早疫病风险较高'
    assert len(result['suggestions']) == 2
    assert result['risk_level'] == 'high'


def test_parse_llm_response_invalid_json():
    raw = 'not json at all'
    result = parse_llm_response(raw)
    assert result['status'] == 'parse_error'
    assert result['raw_text'] == 'not json at all'


def test_parse_llm_response_with_fences():
    raw = '```json\n{"summary": "test", "suggestions": [], "risk_level": "low"}\n```'
    result = parse_llm_response(raw)
    assert result['status'] == 'ok'
    assert result['summary'] == 'test'


def test_build_compact_prompt_missing_data():
    data = {'crop_type': 'wheat'}
    prompt = build_compact_prompt(data)
    assert 'wheat' in prompt
    assert '--' in prompt


def test_build_standard_prompt_minimal():
    data = {'crop_type': 'strawberry'}
    prompt = build_standard_prompt(data)
    assert 'strawberry' in prompt
    assert '无' in prompt
