import pytest
from sentry_weather.cma_client import CMAClient


def test_mock_mode_returns_synthetic_data():
    client = CMAClient(mock_mode=True)
    data = client.fetch_grid_forecast(39.9, 116.4)
    assert data is not None
    assert "days" in data
    assert len(data["days"]) == 7
    assert "hours" in data
    assert len(data["hours"]) > 0
    assert "city" in data
    for day in data["days"]:
        assert "day_offset" in day
        assert "temp_high" in day
        assert "temp_low" in day
        assert "humidity" in day
        assert "precipitation" in day


def test_mock_mode_disaster_warnings():
    client = CMAClient(mock_mode=True)
    alerts = client.fetch_disaster_warning(39.9, 116.4)
    assert alerts == []


def test_real_mode_returns_none_without_credentials():
    client = CMAClient(project_id="", credential_id="", private_key_path="", mock_mode=False)
    data = client.fetch_grid_forecast(39.9, 116.4)
    assert data is None


def test_lookup_city_mock_mode():
    client = CMAClient(mock_mode=True)
    assert client.lookup_city(32.06, 118.79) == "MockCity"


def test_lookup_city_returns_empty_without_credentials():
    client = CMAClient(project_id="", credential_id="", private_key_path="", mock_mode=False)
    assert client.lookup_city(32.06, 118.79) == ""
