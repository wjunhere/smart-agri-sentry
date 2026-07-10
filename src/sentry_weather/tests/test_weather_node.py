import pytest
import rclpy
from sentry_weather.weather_node import WeatherNode
from sentry_interfaces.msg import WeatherForecast


@pytest.fixture(scope='module')
def ros_context():
    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.fixture
def node(ros_context):
    n = WeatherNode()
    yield n
    n.destroy_node()


def test_node_starts_in_mock_mode(node):
    assert node.mock_mode is True


def test_weather_forecast_message_built_from_mock_data(node):
    node._fetch_and_publish()
    msg = node.last_published
    assert msg is not None
    assert len(msg.days) == 7
    assert len(msg.hours) > 0
    assert msg.stale is False
    assert msg.lat == pytest.approx(39.9)
    assert msg.lon == pytest.approx(116.4)


def test_cache_save_and_restore(node):
    node._fetch_and_publish()
    first_city = node.last_published.city
    cached = node._load_from_cache()
    assert cached is not None
    assert cached["city"] == first_city


def test_convert_raw_to_msg(node):
    raw = {
        "city": "TestCity", "lat": 35.0, "lon": 110.0,
        "days": [{"day_offset": 0, "temp_high": 30.0, "temp_low": 20.0,
                  "humidity": 60.0, "precipitation": 0.0, "wind_speed": 5.0,
                  "weather_desc": "晴"}],
        "hours": [{"hour_offset": 0, "temp": 25.0, "humidity": 55.0,
                   "precipitation": 0.0, "wind_speed": 3.0}],
    }
    msg = node._raw_to_msg(raw, False)
    assert msg.city == "TestCity"
    assert len(msg.days) == 1
    assert len(msg.hours) == 1
    assert msg.stale is False
