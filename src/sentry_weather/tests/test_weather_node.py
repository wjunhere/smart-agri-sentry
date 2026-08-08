import json

import pytest
import rclpy
from sensor_msgs.msg import NavSatFix
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
    assert len(msg.days) == 7  # fixed-size array
    assert msg.days[0].temp_high == 30.0
    assert msg.days[0].weather_desc == "晴"
    assert len(msg.hours) == 1
    assert msg.stale is False


def test_set_location_updates_persists_and_refetches(node, tmp_path):
    node.location_cache_path = str(tmp_path / 'loc.json')
    fix = NavSatFix()
    fix.latitude = 32.06
    fix.longitude = 118.79
    node._on_set_location(fix)

    assert node.lat == pytest.approx(32.06)
    assert node.lon == pytest.approx(118.79)
    assert node.city == 'MockCity'  # mock-mode reverse geocode
    saved = json.loads((tmp_path / 'loc.json').read_text())
    assert saved['lat'] == pytest.approx(32.06)
    assert saved['city'] == 'MockCity'
    # A fresh forecast was published for the new coordinates
    assert node.last_published.lat == pytest.approx(32.06)
    assert node.last_published.lon == pytest.approx(118.79)
    assert node.last_published.city == 'MockCity'


def test_set_location_rejects_out_of_range(node, tmp_path):
    node.location_cache_path = str(tmp_path / 'loc.json')
    fix = NavSatFix()
    fix.latitude = 123.0
    fix.longitude = 45.0
    node._on_set_location(fix)

    assert node.lat == pytest.approx(39.9)
    assert node.lon == pytest.approx(116.4)
    assert not (tmp_path / 'loc.json').exists()


def test_saved_location_restored_on_restart(ros_context, tmp_path):
    cache = tmp_path / 'loc.json'
    cache.write_text(json.dumps({'lat': 31.2, 'lon': 121.5, 'city': '上海'}))
    n = WeatherNode()
    try:
        n.location_cache_path = str(cache)
        n.lat, n.lon, n.city = 39.9, 116.4, ''
        n._load_saved_location()
        assert n.lat == pytest.approx(31.2)
        assert n.lon == pytest.approx(121.5)
        assert n.city == '上海'
    finally:
        n.destroy_node()
