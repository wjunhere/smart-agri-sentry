import json
import os
import time
import pytest
from sentry_weather.cache_manager import CacheManager


@pytest.fixture
def tmp_cache(tmp_path):
    return str(tmp_path / "weather_cache.json")


def test_save_and_load(tmp_cache):
    cm = CacheManager(tmp_cache)
    data = {"city": "Beijing", "temp": 25.0}
    cm.save(data)
    loaded = cm.load()
    assert loaded == data


def test_load_nonexistent_returns_none(tmp_cache):
    cm = CacheManager(tmp_cache)
    assert cm.load() is None


def test_is_valid_within_24h(tmp_cache):
    cm = CacheManager(tmp_cache)
    cm.save({"test": True})
    assert cm.is_valid() is True


def test_is_valid_after_24h(tmp_cache):
    cm = CacheManager(tmp_cache)
    cm.save({"test": True})
    old_time = time.time() - 25 * 3600
    os.utime(tmp_cache, (old_time, old_time))
    assert cm.is_valid() is False


def test_is_valid_nonexistent(tmp_cache):
    cm = CacheManager(tmp_cache)
    assert cm.is_valid() is False
