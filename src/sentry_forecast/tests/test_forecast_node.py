import pytest
import rclpy
from sentry_forecast.forecast_node import TrendForecaster, ForecastNode
from sentry_interfaces.msg import FusionResult, Environment, ForecastAlert


@pytest.fixture(scope='module')
def ros_context():
    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.fixture
def node(ros_context):
    n = ForecastNode()
    yield n
    n.destroy_node()


def test_linear_trend_rising():
    samples = [
        {'timestamp': 0.0, 'risk_score': 0.2},
        {'timestamp': 3600.0, 'risk_score': 0.4},
        {'timestamp': 7200.0, 'risk_score': 0.6},
    ]
    slope = TrendForecaster.linear_trend(samples, 'risk_score')
    assert abs(slope - 0.2) < 0.01


def test_predict_risk():
    samples = [
        {'timestamp': 0.0, 'risk_score': 0.2},
        {'timestamp': 3600.0, 'risk_score': 0.4},
        {'timestamp': 7200.0, 'risk_score': 0.6},
    ]
    pred = TrendForecaster.predict(samples, 24, 'risk_score')
    assert abs(pred - 1.0) < 0.01


def test_predict_clips_to_one():
    samples = [
        {'timestamp': 0.0, 'risk_score': 0.5},
        {'timestamp': 3600.0, 'risk_score': 0.8},
    ]
    pred = TrendForecaster.predict(samples, 24, 'risk_score')
    assert pred == pytest.approx(1.0, abs=0.01)


def test_empty_predict_returns_zero():
    assert TrendForecaster.predict([], 24, 'risk_score') == 0.0


def test_insufficient_samples_returns_last():
    samples = [{'timestamp': 0.0, 'risk_score': 0.3}]
    pred = TrendForecaster.predict(samples, 24, 'risk_score')
    assert pred == pytest.approx(0.3, abs=0.01)


def test_predict_alert_rising_risk(node):
    now = node.get_clock().now().nanoseconds / 1e9
    fusion = FusionResult()
    fusion.header.stamp = node.get_clock().now().to_msg()
    fusion.risk_score = 0.65
    fusion.lwd_hours = 3.0
    node.on_fusion(fusion)

    env = Environment()
    env.header.stamp = node.get_clock().now().to_msg()
    env.air_temp = 22.0
    env.air_humidity = 70.0
    node.on_env(env)

    # Fill history with rising risk
    for i in range(10):
        t = now - (3600.0 * (9 - i))
        node.history.append({
            'timestamp': t,
            'risk_score': 0.2 + 0.05 * i,
            'humidity': 70.0,
            'lwd_hours': 3.0,
            'temperature': 22.0,
        })

    alert = node._predict_alert()
    assert alert.active is True
    assert alert.alert_type == 'RISING_RISK'
    assert alert.hours_ahead == 24


def test_predict_alert_inactive_when_fusion_stale(node):
    stale_fusion = FusionResult()
    stale_fusion.header.stamp = node.get_clock().now().to_msg()
    stale_fusion.risk_score = 0.9
    node.on_fusion(stale_fusion)

    # Move history far back to make fusion stale
    node.last_fusion_ts -= 60.0
    alert = node._predict_alert()
    assert alert.active is False
    assert alert.alert_type == 'NONE'


def test_latent_outbreak_detection(node):
    now = node.get_clock().now().nanoseconds / 1e9
    fusion = FusionResult()
    fusion.header.stamp = node.get_clock().now().to_msg()
    fusion.risk_score = 0.3
    fusion.lwd_hours = 4.5  # tomato threshold 6.0, margin 2.0
    node.on_fusion(fusion)

    env = Environment()
    env.header.stamp = node.get_clock().now().to_msg()
    env.air_temp = 20.0
    env.air_humidity = 88.0
    node.on_env(env)

    for i in range(8):
        t = now - (3600.0 * (7 - i))
        node.history.append({
            'timestamp': t,
            'risk_score': 0.3,
            'humidity': 75.0 + 2.0 * i,
            'lwd_hours': 4.5,
            'temperature': 20.0,
        })

    alert = node._predict_alert()
    assert alert.active is True
    assert alert.alert_type == 'LATENT_OUTBREAK'


def test_drought_stress_detection(node):
    now = node.get_clock().now().nanoseconds / 1e9
    fusion = FusionResult()
    fusion.header.stamp = node.get_clock().now().to_msg()
    fusion.risk_score = 0.2
    node.on_fusion(fusion)

    env = Environment()
    env.header.stamp = node.get_clock().now().to_msg()
    env.air_temp = 35.0
    env.air_humidity = 30.0
    node.on_env(env)

    node.history.append({
        'timestamp': now,
        'risk_score': 0.2,
        'humidity': 30.0,
        'lwd_hours': 0.0,
        'temperature': 35.0,
    })

    alert = node._predict_alert()
    assert alert.active is True
    assert alert.alert_type == 'DROUGHT_STRESS'


def test_weather_risk_model_high_risk():
    from sentry_forecast.forecast_node import weather_risk_model
    hours = []
    for i in range(12):
        hours.append({"hour_offset": i, "temp": 20.0, "humidity": 90.0,
                       "precipitation": 1.5, "wind_speed": 2.0})
    risk = weather_risk_model(hours, 24)
    assert risk > 0.3


def test_weather_risk_model_low_risk():
    from sentry_forecast.forecast_node import weather_risk_model
    hours = []
    for i in range(12):
        hours.append({"hour_offset": i, "temp": 30.0, "humidity": 40.0,
                       "precipitation": 0.0, "wind_speed": 5.0})
    risk = weather_risk_model(hours, 24)
    assert risk < 0.2


def test_disaster_factor_with_matching_alerts():
    from sentry_forecast.forecast_node import disaster_factor
    alerts = ["暴雨蓝色预警", "大风黄色预警"]
    assert disaster_factor(alerts) == 0.3
    assert disaster_factor(alerts, 0.5) == 0.5


def test_disaster_factor_empty():
    from sentry_forecast.forecast_node import disaster_factor
    assert disaster_factor([]) == 0.0
    assert disaster_factor([], 0.5) == 0.0


def test_storm_warning_from_disaster_alerts(node):
    now = node.get_clock().now().nanoseconds / 1e9

    fusion = FusionResult()
    fusion.header.stamp = node.get_clock().now().to_msg()
    fusion.risk_score = 0.2
    node.on_fusion(fusion)

    node.last_weather = {
        "hours": [{"hour_offset": i, "temp": 22.0, "humidity": 60.0,
                    "precipitation": 0.0, "wind_speed": 2.0} for i in range(24)],
        "disaster_alerts": ["暴雨蓝色预警"],
    }
    node.last_weather_ts = now

    node.history.append({
        'timestamp': now, 'risk_score': 0.2,
        'humidity': 60.0, 'lwd_hours': 1.0, 'temperature': 22.0,
    })

    alert = node._predict_alert()
    assert alert.active is True
    assert alert.alert_type == 'STORM_WARNING'
    assert alert.alert_source == 'WEATHER'
