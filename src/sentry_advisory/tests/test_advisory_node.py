import pytest
import rclpy
from sentry_advisory.rule_engine import RuleEngine, ALERT_LEVEL_MAP
from sentry_advisory.advisory_node import AdvisoryNode
from sentry_interfaces.msg import (
    FusionResult,
    ForecastAlert,
    Environment,
    AdvisoryAction,
)


@pytest.fixture(scope='module')
def ros_context():
    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.fixture
def engine():
    rules = [
        {
            'name': 'critical_spray',
            'conditions': {
                'crop_type': 'tomato',
                'alert_level': 'CRITICAL',
                'mode': 'VISION_DOMINANT',
            },
            'action': {
                'action_type': 'SPRAY',
                'priority': 'CRITICAL',
                'description': '晚疫病高风险，立即喷药',
                'steps': ['停车', '喷药', '记录'],
            },
        },
        {
            'name': 'latent_monitor',
            'conditions': {'alert_type': 'LATENT_OUTBREAK'},
            'action': {
                'action_type': 'MONITOR',
                'priority': 'HIGH',
                'description': '加强监测',
                'steps': ['增加巡检'],
            },
        },
    ]
    return RuleEngine(rules)


def test_match_critical(engine):
    fusion = FusionResult()
    fusion.risk_score = 0.9
    fusion.alert_level = ALERT_LEVEL_MAP['CRITICAL']
    fusion.mode = 'VISION_DOMINANT'

    forecast = ForecastAlert()
    forecast.active = False
    forecast.alert_type = 'NONE'

    env = Environment()

    action = engine.match(fusion, forecast, env, 'tomato')
    assert action['action_type'] == 'SPRAY'
    assert action['priority'] == 'CRITICAL'


def test_match_latent(engine):
    fusion = FusionResult()
    fusion.risk_score = 0.3
    fusion.alert_level = ALERT_LEVEL_MAP['NORMAL']
    fusion.mode = 'BALANCED'

    forecast = ForecastAlert()
    forecast.active = True
    forecast.alert_type = 'LATENT_OUTBREAK'

    env = Environment()

    action = engine.match(fusion, forecast, env, 'tomato')
    assert action['action_type'] == 'MONITOR'


def test_no_match_fallback(engine):
    fusion = FusionResult()
    fusion.risk_score = 0.1
    fusion.alert_level = ALERT_LEVEL_MAP['NORMAL']
    fusion.mode = 'BALANCED'

    forecast = ForecastAlert()
    forecast.active = False
    forecast.alert_type = 'NONE'

    env = Environment()

    action = engine.match(fusion, forecast, env, 'tomato')
    assert action['action_type'] == 'NONE'
    assert action['priority'] == 'LOW'


@pytest.fixture
def node(ros_context):
    n = AdvisoryNode()
    yield n
    n.destroy_node()


def test_evaluate_publishes_action(node):
    fusion = FusionResult()
    fusion.risk_score = 0.9
    fusion.alert_level = ALERT_LEVEL_MAP['CRITICAL']
    fusion.mode = 'VISION_DOMINANT'

    forecast = ForecastAlert()
    forecast.active = False
    forecast.alert_type = 'NONE'

    env = Environment()
    env.air_temp = 22.0
    env.air_humidity = 70.0

    action = node._evaluate(fusion, forecast, env)
    assert action.action_type == 'SPRAY'
    assert action.priority == 'CRITICAL'


def test_evaluate_uses_fallback(node):
    fusion = FusionResult()
    fusion.risk_score = 0.1
    fusion.alert_level = ALERT_LEVEL_MAP['NORMAL']
    fusion.mode = 'BALANCED'

    forecast = ForecastAlert()
    forecast.active = False
    forecast.alert_type = 'NONE'

    env = Environment()

    action = node._evaluate(fusion, forecast, env)
    assert action.action_type == 'NONE'
