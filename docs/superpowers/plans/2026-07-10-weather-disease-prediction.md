# Weather Disease Prediction Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Integrate CMA weather API into the fusion→forecast→advisory pipeline, enabling hybrid prediction that blends local sensor trends with 7-day weather forecasts.

**Architecture:** New `sentry_weather` package fetches CMA grid forecasts every 3h, caches locally, and publishes on `/weather/forecast`. `sentry_forecast` is enhanced to blend local sensor history (40%) with weather-derived risk (60%). `sentry_advisory` rule engine gains weather-aware conditions. Frontend adds location config, weather panel, and advisory source labels.

**Tech Stack:** ROS2 Humble (Python), CMA Open API, JSON file cache, Vue 3 + ECharts frontend, roslibjs

---

## Phase 1: Messages + sentry_weather Package

### Task 1: Create WeatherDay.msg and WeatherHour.msg

**Files:**
- Create: `src/sentry_interfaces/msg/WeatherDay.msg`
- Create: `src/sentry_interfaces/msg/WeatherHour.msg`

- [ ] **Step 1: Create WeatherDay.msg**

```
uint8 day_offset
float32 temp_high
float32 temp_low
float32 humidity
float32 precipitation
float32 wind_speed
string weather_desc
```

- [ ] **Step 2: Create WeatherHour.msg**

```
uint8 hour_offset
float32 temperature
float32 humidity
float32 precipitation
float32 wind_speed
```

- [ ] **Step 3: Commit**

```bash
git add src/sentry_interfaces/msg/WeatherDay.msg src/sentry_interfaces/msg/WeatherHour.msg
git commit -m "feat: add WeatherDay and WeatherHour message definitions"
```

---

### Task 2: Create WeatherForecast.msg and extend ForecastAlert.msg

**Files:**
- Create: `src/sentry_interfaces/msg/WeatherForecast.msg`
- Modify: `src/sentry_interfaces/msg/ForecastAlert.msg`

- [ ] **Step 1: Create WeatherForecast.msg**

```
std_msgs/Header header
string city
float64 lat
float64 lon
WeatherDay[7] days
WeatherHour[] hours
string[] disaster_alerts
bool stale
```

- [ ] **Step 2: Add alert_source field to ForecastAlert.msg**

Add to the end of the existing file:
```
string alert_source
```

- [ ] **Step 3: Update CMakeLists.txt to register new messages**

Add to the `rosidl_generate_interfaces` call in `src/sentry_interfaces/CMakeLists.txt`, after the existing msg list:
```
  "msg/WeatherDay.msg"
  "msg/WeatherHour.msg"
  "msg/WeatherForecast.msg"
```

- [ ] **Step 4: Commit**

```bash
git add src/sentry_interfaces/msg/WeatherForecast.msg src/sentry_interfaces/msg/ForecastAlert.msg src/sentry_interfaces/CMakeLists.txt
git commit -m "feat: add WeatherForecast message and extend ForecastAlert with alert_source"
```

---

### Task 3: Create sentry_weather package skeleton

**Files:**
- Create: `src/sentry_weather/package.xml`
- Create: `src/sentry_weather/setup.py`
- Create: `src/sentry_weather/sentry_weather/__init__.py`
- Create: `src/sentry_weather/resource/sentry_weather` (empty marker)

- [ ] **Step 1: Create package.xml**

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>sentry_weather</name>
  <version>0.1.0</version>
  <description>Weather data ingestion node (CMA API) for Smart Agri Sentry</description>
  <maintainer email="team@example.com">team</maintainer>
  <license>MIT</license>

  <depend>rclpy</depend>
  <depend>sentry_interfaces</depend>
  <depend>std_msgs</depend>

  <test_depend>pytest</test_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
```

- [ ] **Step 2: Create setup.py**

```python
from setuptools import find_packages, setup

package_name = 'sentry_weather'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/weather_params.yaml']),
    ],
    install_requires=['setuptools'],
    extras_require={'test': ['pytest']},
    zip_safe=True,
    maintainer='team',
    maintainer_email='team@example.com',
    description='Weather data ingestion for Smart Agri Sentry',
    license='MIT',
    entry_points={
        'console_scripts': [
            'weather_node = sentry_weather.weather_node:main',
        ],
    },
)
```

- [ ] **Step 3: Create empty __init__.py and resource marker**

```bash
touch src/sentry_weather/sentry_weather/__init__.py
touch src/sentry_weather/resource/sentry_weather
```

- [ ] **Step 4: Commit**

```bash
git add src/sentry_weather/package.xml src/sentry_weather/setup.py src/sentry_weather/sentry_weather/__init__.py src/sentry_weather/resource/sentry_weather
git commit -m "feat: add sentry_weather package skeleton"
```

---

### Task 4: Create cache_manager.py with TDD

**Files:**
- Create: `src/sentry_weather/sentry_weather/cache_manager.py`
- Create: `src/sentry_weather/tests/test_cache_manager.py`
- Create: `src/sentry_weather/tests/__init__.py`

- [ ] **Step 1: Write the failing test**

```python
# tests/test_cache_manager.py
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
    # Manually set mtime to 25 hours ago
    old_time = time.time() - 25 * 3600
    os.utime(tmp_cache, (old_time, old_time))
    assert cm.is_valid() is False


def test_is_valid_nonexistent(tmp_cache):
    cm = CacheManager(tmp_cache)
    assert cm.is_valid() is False
```

- [ ] **Step 2: Run test to verify it fails**

Run: `python -m pytest src/sentry_weather/tests/test_cache_manager.py -v`
Expected: FAIL with "No module named 'sentry_weather.cache_manager'"

- [ ] **Step 3: Implement cache_manager.py**

```python
"""JSON file cache for weather forecast data."""
import json
import os
import time


class CacheManager:
    def __init__(self, cache_path: str):
        self.cache_path = cache_path

    def save(self, data: dict) -> None:
        os.makedirs(os.path.dirname(self.cache_path), exist_ok=True)
        with open(self.cache_path, 'w') as f:
            json.dump(data, f, ensure_ascii=False, indent=2)

    def load(self) -> dict | None:
        if not os.path.exists(self.cache_path):
            return None
        with open(self.cache_path, 'r') as f:
            return json.load(f)

    def is_valid(self) -> bool:
        if not os.path.exists(self.cache_path):
            return False
        mtime = os.path.getmtime(self.cache_path)
        age_sec = time.time() - mtime
        return age_sec < 24 * 3600
```

- [ ] **Step 4: Run test to verify it passes**

Run: `python -m pytest src/sentry_weather/tests/test_cache_manager.py -v`
Expected: 5 PASS

- [ ] **Step 5: Commit**

```bash
git add src/sentry_weather/sentry_weather/cache_manager.py src/sentry_weather/tests/
git commit -m "feat: add CacheManager with JSON file cache"
```

---

### Task 5: Create cma_client.py with TDD

**Files:**
- Create: `src/sentry_weather/sentry_weather/cma_client.py`
- Create: `src/sentry_weather/tests/test_cma_client.py`

- [ ] **Step 1: Write the failing test**

```python
# tests/test_cma_client.py
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
        assert "temp_high" in day
        assert "temp_low" in day
        assert "humidity" in day
        assert "precipitation" in day


def test_mock_mode_disaster_warnings():
    client = CMAClient(mock_mode=True)
    alerts = client.fetch_disaster_warning(39.9, 116.4)
    assert alerts == []


def test_real_mode_returns_none_on_network_error():
    client = CMAClient(api_base_url="http://localhost:99999/nonexistent", api_key="test")
    data = client.fetch_grid_forecast(39.9, 116.4)
    assert data is None
```

- [ ] **Step 2: Run test to verify it fails**

Run: `python -m pytest src/sentry_weather/tests/test_cma_client.py -v`
Expected: FAIL

- [ ] **Step 3: Implement cma_client.py**

```python
"""China Meteorological Administration API client."""
import random
import urllib.request
import urllib.error
import json


class CMAClient:
    def __init__(self, api_base_url="", api_key="", mock_mode=False):
        self.api_base_url = api_base_url
        self.api_key = api_key
        self.mock_mode = mock_mode

    def fetch_grid_forecast(self, lat, lon):
        if self.mock_mode:
            return self._mock_forecast(lat, lon)
        return self._http_get(self._build_url(lat, lon))

    def fetch_disaster_warning(self, lat, lon):
        if self.mock_mode:
            return []
        data = self._http_get(self._build_warning_url(lat, lon))
        if data is None:
            return []
        return data.get("alerts", [])

    def _build_url(self, lat, lon):
        return f"{self.api_base_url}?lat={lat}&lon={lon}&key={self.api_key}"

    def _build_warning_url(self, lat, lon):
        return f"{self.api_base_url}/warning?lat={lat}&lon={lon}&key={self.api_key}"

    def _http_get(self, url):
        try:
            req = urllib.request.Request(url)
            with urllib.request.urlopen(req, timeout=30) as resp:
                return json.loads(resp.read().decode())
        except (urllib.error.URLError, OSError, json.JSONDecodeError, ValueError):
            return None

    def _mock_forecast(self, lat, lon):
        weather_descs = ["晴", "多云", "阴", "小雨", "中雨", "晴", "多云"]
        days = []
        for offset in range(7):
            base_temp = 22.0 + random.uniform(-5, 5)
            days.append({
                "day_offset": offset,
                "temp_high": round(base_temp + random.uniform(2, 8), 1),
                "temp_low": round(base_temp - random.uniform(3, 10), 1),
                "humidity": round(random.uniform(40, 95), 1),
                "precipitation": round(random.uniform(0, 15), 1),
                "wind_speed": round(random.uniform(0, 12), 1),
                "weather_desc": weather_descs[offset % len(weather_descs)],
            })

        hours = []
        for offset in range(168):
            hour_temp = 22.0 + random.uniform(-8, 8)
            hours.append({
                "hour_offset": offset,
                "temp": round(hour_temp, 1),
                "humidity": round(random.uniform(40, 95), 1),
                "precipitation": round(random.uniform(0, 3), 1),
                "wind_speed": round(random.uniform(0, 8), 1),
            })

        return {"city": "MockCity", "lat": lat, "lon": lon, "days": days, "hours": hours}
```

- [ ] **Step 4: Run test to verify it passes**

Run: `python -m pytest src/sentry_weather/tests/test_cma_client.py -v`
Expected: 3 PASS

- [ ] **Step 5: Commit**

```bash
git add src/sentry_weather/sentry_weather/cma_client.py src/sentry_weather/tests/test_cma_client.py
git commit -m "feat: add CMAClient with mock mode"
```

---

### Task 6: Create weather_params.yaml

**Files:**
- Create: `src/sentry_weather/config/weather_params.yaml`

- [ ] **Step 1: Create the config file**

```yaml
weather_node:
  ros__parameters:
    lat: 39.9
    lon: 116.4
    city: ""
    fetch_interval_sec: 10800
    cache_path: "/tmp/sentry_weather_cache.json"
    api_key: ""
    api_base_url: ""
    mock_mode: true
```

- [ ] **Step 2: Commit**

```bash
git add src/sentry_weather/config/weather_params.yaml
git commit -m "feat: add weather node default parameters"
```

---

### Task 7: Create weather_node.py with TDD

**Files:**
- Create: `src/sentry_weather/sentry_weather/weather_node.py`
- Create: `src/sentry_weather/tests/test_weather_node.py`

- [ ] **Step 1: Write the failing tests**

```python
# tests/test_weather_node.py
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
    # Trigger load from cache
    cached = node._load_from_cache()
    assert cached is not None
    assert cached.city == first_city


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
```

- [ ] **Step 2: Run test to verify it fails**

Run: `python -m pytest src/sentry_weather/tests/test_weather_node.py -v`
Expected: FAIL

- [ ] **Step 3: Implement weather_node.py**

```python
"""Weather data ingestion node for Smart Agri Sentry."""
import rclpy
from rclpy.node import Node
from sentry_interfaces.msg import WeatherForecast, WeatherDay, WeatherHour
from .cma_client import CMAClient
from .cache_manager import CacheManager


class WeatherNode(Node):
    def __init__(self):
        super().__init__('weather_node')
        self.declare_parameter('lat', 39.9)
        self.declare_parameter('lon', 116.4)
        self.declare_parameter('city', '')
        self.declare_parameter('fetch_interval_sec', 10800)
        self.declare_parameter('cache_path', '/tmp/sentry_weather_cache.json')
        self.declare_parameter('api_key', '')
        self.declare_parameter('api_base_url', '')
        self.declare_parameter('mock_mode', True)

        self.lat = self.get_parameter('lat').value
        self.lon = self.get_parameter('lon').value
        self.city = self.get_parameter('city').value
        self.fetch_interval_sec = self.get_parameter('fetch_interval_sec').value
        self.mock_mode = self.get_parameter('mock_mode').value

        cache_path = self.get_parameter('cache_path').value
        api_key = self.get_parameter('api_key').value
        api_base_url = self.get_parameter('api_base_url').value

        self.cache = CacheManager(cache_path)
        self.client = CMAClient(api_base_url=api_base_url, api_key=api_key,
                                mock_mode=self.mock_mode)
        self.last_published = None

        self.pub = self.create_publisher(WeatherForecast, '/weather/forecast', 10)

        # Publish cached data on startup
        cached = self._load_from_cache()
        if cached is not None:
            msg = self._raw_to_msg(cached, True)
            self.pub.publish(msg)
            self.last_published = msg

        # Initial fetch
        self._fetch_and_publish()

        self.timer = self.create_timer(float(self.fetch_interval_sec), self._on_timer)
        self.get_logger().info('Weather node ready (mock=%s)', str(self.mock_mode))

    def _on_timer(self):
        self._fetch_and_publish()

    def _fetch_and_publish(self):
        raw = self.client.fetch_grid_forecast(self.lat, self.lon)
        if raw is None:
            self.get_logger().warn('API fetch failed, trying cache')
            cached = self._load_from_cache()
            if cached is not None:
                msg = self._raw_to_msg(cached, True)
                self.pub.publish(msg)
                self.last_published = msg
            return

        raw.setdefault("city", self.city)
        raw.setdefault("lat", self.lat)
        raw.setdefault("lon", self.lon)

        alerts = self.client.fetch_disaster_warning(self.lat, self.lon)
        raw["disaster_alerts"] = alerts if alerts else []

        self.cache.save(raw)
        msg = self._raw_to_msg(raw, False)
        self.pub.publish(msg)
        self.last_published = msg
        self.get_logger().info('Weather forecast published')

    def _load_from_cache(self):
        if self.cache.is_valid():
            return self.cache.load()
        return None

    def _raw_to_msg(self, raw, stale):
        msg = WeatherForecast()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'weather'
        msg.city = raw.get("city", "")
        msg.lat = float(raw.get("lat", self.lat))
        msg.lon = float(raw.get("lon", self.lon))
        msg.stale = stale

        for d in raw.get("days", []):
            day = WeatherDay()
            day.day_offset = d["day_offset"]
            day.temp_high = d["temp_high"]
            day.temp_low = d["temp_low"]
            day.humidity = d["humidity"]
            day.precipitation = d["precipitation"]
            day.wind_speed = d["wind_speed"]
            day.weather_desc = d.get("weather_desc", "")
            msg.days.append(day)

        for h in raw.get("hours", []):
            hour = WeatherHour()
            hour.hour_offset = h["hour_offset"]
            hour.temp = h["temp"]
            hour.humidity = h["humidity"]
            hour.precipitation = h["precipitation"]
            hour.wind_speed = h["wind_speed"]
            msg.hours.append(hour)

        msg.disaster_alerts = raw.get("disaster_alerts", [])
        return msg


def main(args=None):
    rclpy.init(args=args)
    node = WeatherNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
```

- [ ] **Step 4: Run test to verify it passes**

Run: `python -m pytest src/sentry_weather/tests/test_weather_node.py -v`
Expected: 4 PASS

- [ ] **Step 5: Commit**

```bash
git add src/sentry_weather/sentry_weather/weather_node.py src/sentry_weather/tests/test_weather_node.py
git commit -m "feat: add WeatherNode with cache, mock, and CMA client"
```

---

## Phase 2: sentry_forecast Enhancement

### Task 8: Update forecast_params.yaml

**Files:**
- Modify: `config/forecast_params.yaml`

- [ ] **Step 1: Add new parameters**

Append after existing params:
```yaml
  blend_weight_local: 0.4
  blend_weight_weather: 0.6
  disaster_boost_cap: 0.3
  weather_stale_sec: 21600
```

- [ ] **Step 2: Commit**

```bash
git add config/forecast_params.yaml
git commit -m "feat: add blend weights and weather params to forecast config"
```

---

### Task 9: Add weather risk model and new alert types to forecast_node.py

**Files:**
- Modify: `src/sentry_forecast/sentry_forecast/forecast_node.py`

- [ ] **Step 1: Write the failing tests first**

```python
# Add to existing tests/test_forecast_node.py
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
    factor = disaster_factor(alerts)
    assert factor == 0.3


def test_disaster_factor_empty():
    from sentry_forecast.forecast_node import disaster_factor
    assert disaster_factor([]) == 0.0
```

Run: `python -m pytest src/sentry_forecast/tests/test_forecast_node.py::test_weather_risk_model_high_risk -v`
Expected: FAIL

- [ ] **Step 2: Add new alert constants and helper functions**

Add after the existing `ALERT_*` constants in `forecast_node.py`:
```python
ALERT_STORM_WARNING = 'STORM_WARNING'
ALERT_FROST_WARNING = 'FROST_WARNING'
ALERT_HEAT_STRESS = 'HEAT_STRESS'


def weather_risk_model(hours, prediction_hours):
    """Compute disease risk (0-1) from hourly weather forecast data."""
    if not hours:
        return 0.0
    window = [h for h in hours if h["hour_offset"] <= prediction_hours]
    if not window:
        return 0.0

    risk = 0.0
    # Fungal window: RH > 85% and 15 < T < 25, sustained > 6h
    fungal_hours = sum(1 for h in window
                       if h["humidity"] > 85.0 and 15.0 < h["temp"] < 25.0)
    if fungal_hours > 6:
        risk += min(0.4, 0.2 + 0.033 * (fungal_hours - 6))

    # Continuous rain > 1mm/h for > 12h
    rain_hours = 0
    for h in window:
        if h["precipitation"] > 1.0:
            rain_hours += 1
        else:
            rain_hours = 0
    if rain_hours > 12:
        risk += min(0.3, 0.2 + 0.008 * (rain_hours - 12))

    # Consecutive rain days > 2
    rain_days = 0
    for day_offset in range(7):
        day_hours = [h for h in hours if day_offset * 24 <= h["hour_offset"] < (day_offset + 1) * 24]
        if day_hours and sum(h["precipitation"] for h in day_hours) > 1.0:
            rain_days += 1
        else:
            rain_days = 0
    if rain_days > 2:
        risk += min(0.2, 0.1 * (rain_days - 2))

    # Heat stress: T > 35 sustained > 6h
    heat_hours = sum(1 for h in window if h["temp"] > 35.0)
    if heat_hours > 6:
        risk += min(0.3, 0.2 + 0.016 * (heat_hours - 6))

    # Frost risk: T < 5 present
    frost_hours = sum(1 for h in window if h["temp"] < 5.0)
    if frost_hours > 0:
        risk += min(0.4, 0.2 + 0.05 * frost_hours)

    return min(1.0, risk)


def disaster_factor(disaster_alerts):
    """Boost risk based on disaster warnings (0 to disaster_boost_cap)."""
    if not disaster_alerts:
        return 0.0
    return 0.3
```

- [ ] **Step 3: Run weather model tests to verify they pass**

Run: `python -m pytest src/sentry_forecast/tests/test_forecast_node.py -k "weather_risk or disaster" -v`
Expected: 4 PASS

- [ ] **Step 4: Commit**

```bash
git add src/sentry_forecast/sentry_forecast/forecast_node.py src/sentry_forecast/tests/test_forecast_node.py
git commit -m "feat: add weather risk model and disaster factor functions"
```

---

### Task 10: Integrate weather subscription and hybrid prediction into forecast_node

**Files:**
- Modify: `src/sentry_forecast/sentry_forecast/forecast_node.py`

- [ ] **Step 1: Write the failing test**

```python
# Add to test_forecast_node.py
def test_hybrid_blend_with_weather(node):
    from sentry_forecast.forecast_node import weather_risk_model, disaster_factor
    now = node.get_clock().now().nanoseconds / 1e9

    fusion = FusionResult()
    fusion.header.stamp = node.get_clock().now().to_msg()
    fusion.risk_score = 0.3
    fusion.lwd_hours = 3.0
    node.on_fusion(fusion)

    # Set weather data on node
    node.last_weather = {
        "hours": [{"hour_offset": i, "temp": 20.0, "humidity": 90.0,
                    "precipitation": 1.5, "wind_speed": 2.0} for i in range(24)],
        "disaster_alerts": [],
    }
    node.last_weather_ts = now

    for i in range(8):
        t = now - (3600.0 * (7 - i))
        node.history.append({
            'timestamp': t, 'risk_score': 0.3,
            'humidity': 80.0, 'lwd_hours': 3.0, 'temperature': 22.0,
        })

    alert = node._predict_alert()
    # Hybrid prediction should incorporate weather data
    assert alert.alert_source in ('LOCAL', 'WEATHER', 'HYBRID')


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
```

Run: `python -m pytest src/sentry_forecast/tests/test_forecast_node.py::test_hybrid_blend_with_weather -v`
Expected: FAIL (AttributeError: 'ForecastNode' object has no attribute 'last_weather')

- [ ] **Step 2: Add weather subscription and hybrid prediction to forecast_node.py**

Add import at top:
```python
from sentry_interfaces.msg import WeatherForecast
```

Add to `__init__` after existing subscriptions:
```python
self.sub_weather = self.create_subscription(
    WeatherForecast, '/weather/forecast', self.on_weather, 10)

self.last_weather = None
self.last_weather_ts = 0.0
self.weather_stale_sec = self.params.get('weather_stale_sec', 21600)
```

Add callback:
```python
def on_weather(self, msg: WeatherForecast):
    now = self.get_clock().now().nanoseconds / 1e9
    self.last_weather = {
        "hours": [{"hour_offset": h.hour_offset, "temp": h.temp,
                    "humidity": h.humidity, "precipitation": h.precipitation,
                    "wind_speed": h.wind_speed} for h in msg.hours],
        "disaster_alerts": list(msg.disaster_alerts),
    }
    self.last_weather_ts = now
```

Replace the `_predict_alert` method body to blend weather data. Replace the lines from `prediction_hours = ...` to the final `return msg`:

```python
def _predict_alert(self) -> ForecastAlert:
    now = self.get_clock().now().nanoseconds / 1e9
    self._prune_history(now)

    msg = ForecastAlert()
    msg.header.stamp = self.get_clock().now().to_msg()
    msg.header.frame_id = 'forecast'
    msg.hours_ahead = 24

    if self.last_fusion is None:
        msg.active = False
        msg.alert_type = ALERT_NONE
        msg.probability = 0.0
        msg.description = 'No fusion data yet'
        msg.alert_source = 'LOCAL'
        return msg

    if (now - self.last_fusion_ts) > self.fusion_stale_sec:
        msg.active = False
        msg.alert_type = ALERT_NONE
        msg.probability = 0.0
        msg.description = 'Fusion data stale'
        msg.alert_source = 'LOCAL'
        return msg

    prediction_hours = self.params.get('prediction_hours', 24)
    risk_threshold = self.params.get('risk_threshold', 0.7)
    lwd_margin = self.params.get('lwd_margin_hours', 2.0)
    hum_trend_th = self.params.get('humidity_trend_threshold', 0.3)
    lwd_threshold = self.profile.get('lwd_threshold_hours', 6.0)
    w_local = self.params.get('blend_weight_local', 0.4)
    w_weather = self.params.get('blend_weight_weather', 0.6)
    boost_cap = self.params.get('disaster_boost_cap', 0.3)

    # Local trend
    local_risk = TrendForecaster.predict(self.history, prediction_hours, 'risk_score')
    humidity_slope = TrendForecaster.linear_trend(self.history, 'humidity')

    # Weather risk
    weather_risk = 0.0
    d_factor = 0.0
    weather_available = (self.last_weather is not None
                         and (now - self.last_weather_ts) <= self.weather_stale_sec)

    if weather_available:
        weather_risk = weather_risk_model(
            self.last_weather["hours"], prediction_hours)
        d_factor = disaster_factor(self.last_weather["disaster_alerts"])

    # Hybrid blend
    blended = w_local * local_risk + w_weather * weather_risk
    blended = max(0.0, min(1.0, blended + d_factor))

    alert_type = ALERT_NONE
    description = '风险平稳，无需预警'
    alert_source = 'LOCAL'

    risk_slope = TrendForecaster.linear_trend(self.history, 'risk_score')

    # Weather-driven alerts first (higher priority)
    if weather_available:
        if d_factor > 0:
            keywords = ["暴雨", "台风", "大风"]
            for kw in keywords:
                if any(kw in a for a in self.last_weather["disaster_alerts"]):
                    alert_type = ALERT_STORM_WARNING
                    description = f'灾害预警: {self.last_weather["disaster_alerts"][0]}'
                    alert_source = 'WEATHER'
                    break

        if alert_type == ALERT_NONE:
            frost_hours = sum(1 for h in self.last_weather["hours"][:72]
                              if h["temp"] < 5.0)
            if frost_hours > 0:
                alert_type = ALERT_FROST_WARNING
                description = f'未来3天有霜冻风险（{frost_hours}h < 5°C）'
                alert_source = 'WEATHER'

        if alert_type == ALERT_NONE:
            heat_hours = sum(1 for h in self.last_weather["hours"][:72]
                             if h["temp"] > 35.0)
            if heat_hours > 6:
                alert_type = ALERT_HEAT_STRESS
                description = f'未来3天持续高温 {heat_hours}h > 35°C'
                alert_source = 'WEATHER'

    # Local-driven alerts (fallback)
    if alert_type == ALERT_NONE:
        if blended >= risk_threshold and risk_slope > 0:
            alert_type = ALERT_RISING_RISK
            description = f'预测 24h 风险 {blended:.2f}，呈上升趋势'
            alert_source = 'HYBRID' if weather_available else 'LOCAL'
        elif (self.last_fusion.lwd_hours >= (lwd_threshold - lwd_margin)
              and humidity_slope >= hum_trend_th):
            alert_type = ALERT_LATENT_OUTBREAK
            description = (
                f'LWD 接近阈值 ({self.last_fusion.lwd_hours:.1f}h / '
                f'{lwd_threshold:.1f}h)，湿度持续上升')
            alert_source = 'LOCAL'
        elif (self.last_env is not None
              and (now - self.last_env_ts) <= self.mobile_stale_sec
              and self.last_env.air_humidity <= 40.0
              and self.last_env.air_temp >= 30.0):
            alert_type = ALERT_DROUGHT_STRESS
            description = (
                f'干旱胁迫：温度 {self.last_env.air_temp:.1f}C，'
                f'湿度 {self.last_env.air_humidity:.1f}%')
            alert_source = 'LOCAL'

    msg.active = alert_type != ALERT_NONE
    msg.alert_type = alert_type
    msg.probability = float(blended)
    msg.description = description
    msg.alert_source = alert_source
    return msg
```

- [ ] **Step 3: Run all forecast tests to verify they pass**

Run: `python -m pytest src/sentry_forecast/tests/test_forecast_node.py -v`
Expected: All PASS (existing + new)

- [ ] **Step 4: Commit**

```bash
git add src/sentry_forecast/sentry_forecast/forecast_node.py src/sentry_forecast/tests/test_forecast_node.py
git commit -m "feat: integrate weather subscription and hybrid prediction into forecast_node"
```

---

## Phase 3: sentry_advisory Enhancement

### Task 11: Extend rule_engine.py with weather conditions

**Files:**
- Modify: `src/sentry_advisory/sentry_advisory/rule_engine.py`

- [ ] **Step 1: Write the failing tests**

```python
# Add to test_advisory_node.py
def test_match_with_disaster_alert_contains(engine):
    fusion = FusionResult()
    fusion.risk_score = 0.2
    fusion.alert_level = ALERT_LEVEL_MAP['NORMAL']
    fusion.mode = 'BALANCED'

    forecast = ForecastAlert()
    forecast.active = True
    forecast.alert_type = 'STORM_WARNING'

    env = Environment()

    weather_hours = [{"hour_offset": i, "temp": 22.0, "humidity": 60.0,
                       "precipitation": 0.0, "wind_speed": 2.0} for i in range(24)]

    engine_with_storm = RuleEngine([{
        'name': 'storm_test',
        'conditions': {'alert_type': 'STORM_WARNING', 'disaster_alert_contains': '暴雨'},
        'action': {'action_type': 'SPRAY', 'priority': 'CRITICAL',
                   'description': '暴雨预警', 'steps': []},
    }])
    action = engine_with_storm.match(fusion, forecast, env, 'tomato', weather_hours, ["暴雨蓝色预警"])
    assert action['action_type'] == 'SPRAY'


def test_match_with_forecast_high_gt(engine):
    fusion = FusionResult()
    fusion.risk_score = 0.2
    fusion.alert_level = ALERT_LEVEL_MAP['NORMAL']
    fusion.mode = 'BALANCED'

    forecast = ForecastAlert()
    forecast.active = True
    forecast.alert_type = 'HEAT_STRESS'

    env = Environment()

    weather_hours = [{"hour_offset": i, "temp": 36.0, "humidity": 40.0,
                       "precipitation": 0.0, "wind_speed": 2.0} for i in range(72)]

    engine_with_heat = RuleEngine([{
        'name': 'heat_test',
        'conditions': {'alert_type': 'HEAT_STRESS', 'forecast_high_gt': 35, 'forecast_days': 3},
        'action': {'action_type': 'IRRIGATE', 'priority': 'HIGH',
                   'description': '高温灌溉', 'steps': []},
    }])
    action = engine_with_heat.match(fusion, forecast, env, 'tomato', weather_hours, [])
    assert action['action_type'] == 'IRRIGATE'


def test_match_rain_days_condition(engine):
    fusion = FusionResult()
    fusion.risk_score = 0.1
    fusion.alert_level = ALERT_LEVEL_MAP['NORMAL']
    fusion.mode = 'BALANCED'

    forecast = ForecastAlert()
    forecast.active = False
    forecast.alert_type = 'NONE'

    env = Environment()

    # 3 consecutive days with rain
    weather_hours = []
    for day in range(3):
        for h in range(24):
            weather_hours.append({"hour_offset": day * 24 + h, "temp": 20.0,
                                  "humidity": 90.0, "precipitation": 2.0, "wind_speed": 2.0})

    engine_with_rain = RuleEngine([{
        'name': 'rain_test',
        'conditions': {'forecast_rain_days': 2},
        'action': {'action_type': 'MONITOR', 'priority': 'MEDIUM',
                   'description': '连续降雨', 'steps': []},
    }])
    action = engine_with_rain.match(fusion, forecast, env, 'tomato', weather_hours, [])
    assert action['action_type'] == 'MONITOR'
```

- [ ] **Step 2: Run test to verify it fails**

Run: `python -m pytest src/sentry_advisory/tests/test_advisory_node.py::test_match_with_disaster_alert_contains -v`
Expected: FAIL (TypeError: match() takes N positional arguments)

- [ ] **Step 3: Extend match() and _match_conditions() signatures**

Change `match()`:
```python
def match(self, fusion, forecast, env, crop_type, weather_hours=None, disaster_alerts=None):
    for rule in self.rules:
        if self._match_conditions(
                rule.get('conditions', {}), fusion, forecast, env, crop_type,
                weather_hours, disaster_alerts):
            return rule.get('action', self.default_action())
    return self.default_action()
```

Change `_match_conditions()` signature and add weather checks at the end (before `return True`):
```python
def _match_conditions(self, cond, fusion, forecast, env, crop_type,
                      weather_hours=None, disaster_alerts=None):
    # ... existing condition checks unchanged ...

    # Weather conditions
    if 'disaster_alert_contains' in cond:
        if not disaster_alerts:
            return False
        keyword = cond['disaster_alert_contains']
        if not any(keyword in a for a in disaster_alerts):
            return False

    if 'forecast_high_gt' in cond:
        if not weather_hours:
            return False
        days = cond.get('forecast_days', 3)
        window = [h for h in weather_hours if h["hour_offset"] < days * 24]
        if not window:
            return False
        if max(h["temp"] for h in window) <= cond['forecast_high_gt']:
            return False

    if 'forecast_low_lt' in cond:
        if not weather_hours:
            return False
        days = cond.get('forecast_days', 3)
        window = [h for h in weather_hours if h["hour_offset"] < days * 24]
        if not window:
            return False
        if min(h["temp"] for h in window) >= cond['forecast_low_lt']:
            return False

    if 'forecast_rain_days' in cond:
        if not weather_hours:
            return False
        consecutive = 0
        max_consecutive = 0
        for day_offset in range(7):
            day_hours = [h for h in weather_hours
                         if day_offset * 24 <= h["hour_offset"] < (day_offset + 1) * 24]
            if day_hours and sum(h["precipitation"] for h in day_hours) > 1.0:
                consecutive += 1
                max_consecutive = max(max_consecutive, consecutive)
            else:
                consecutive = 0
        if max_consecutive < cond['forecast_rain_days']:
            return False

    return True
```

- [ ] **Step 4: Run all advisory tests to verify they pass**

Run: `python -m pytest src/sentry_advisory/tests/test_advisory_node.py -v`
Expected: All PASS

- [ ] **Step 5: Commit**

```bash
git add src/sentry_advisory/sentry_advisory/rule_engine.py src/sentry_advisory/tests/test_advisory_node.py
git commit -m "feat: extend rule engine with weather conditions"
```

---

### Task 12: Update advisory_node.py to subscribe and pass weather data

**Files:**
- Modify: `src/sentry_advisory/sentry_advisory/advisory_node.py`

- [ ] **Step 1: Update advisory_node.py**

Add import:
```python
from sentry_interfaces.msg import WeatherForecast
```

Add to `__init__` after existing subscriptions:
```python
self.last_weather = None

self.sub_weather = self.create_subscription(
    WeatherForecast, '/weather/forecast', self.on_weather, 10)
```

Add callback:
```python
def on_weather(self, msg: WeatherForecast):
    self.last_weather = {
        "hours": [{"hour_offset": h.hour_offset, "temp": h.temp,
                    "humidity": h.humidity, "precipitation": h.precipitation,
                    "wind_speed": h.wind_speed} for h in msg.hours],
        "disaster_alerts": list(msg.disaster_alerts),
    }
    self._maybe_publish()
```

Update `_evaluate` to extract weather data and pass to `match()`:
```python
def _evaluate(self, fusion, forecast, env):
    forecast = forecast or ForecastAlert()
    env = env or Environment()
    weather_hours = self.last_weather["hours"] if self.last_weather else None
    disaster_alerts = self.last_weather["disaster_alerts"] if self.last_weather else None
    matched = self.engine.match(fusion, forecast, env, self.crop_type,
                                weather_hours, disaster_alerts)

    msg = AdvisoryAction()
    msg.header.stamp = self.get_clock().now().to_msg()
    msg.header.frame_id = 'advisory'
    msg.action_type = matched.get('action_type', 'NONE')
    msg.description = matched.get('description', '')
    msg.priority = matched.get('priority', 'LOW')
    msg.steps = matched.get('steps', [])
    return msg
```

- [ ] **Step 2: Run tests to verify**

Run: `python -m pytest src/sentry_advisory/tests/test_advisory_node.py -v`
Expected: All PASS

- [ ] **Step 3: Commit**

```bash
git add src/sentry_advisory/sentry_advisory/advisory_node.py
git commit -m "feat: add weather subscription to advisory_node"
```

---

### Task 13: Add weather-aware rules to advisory_rules.yaml

**Files:**
- Modify: `config/advisory_rules.yaml`

- [ ] **Step 1: Prepend new rules before existing rules**

```yaml
rules:
  - name: pre_storm_spray
    conditions:
      alert_type: STORM_WARNING
      disaster_alert_contains: "暴雨"
    action:
      action_type: SPRAY
      priority: CRITICAL
      description: "未来有暴雨，建议雨前完成喷药，避免冲刷药效。"
      steps:
        - "确认药剂库存"
        - "优先喷洒高风险区"
        - "暴雨前6h完成"

  - name: storm_general
    conditions:
      alert_type: STORM_WARNING
    action:
      action_type: PROTECT
      priority: HIGH
      description: "灾害性天气预警，检查大棚与排水设施。"
      steps:
        - "检查大棚结构稳固性"
        - "清理排水沟渠"
        - "必要时加固"

  - name: heat_irrigation
    conditions:
      alert_type: HEAT_STRESS
      forecast_high_gt: 35
      forecast_days: 3
    action:
      action_type: IRRIGATE
      priority: HIGH
      description: "未来3天持续高温超过35°C，建议增加早晚灌溉频次。"
      steps:
        - "清晨或傍晚灌溉"
        - "检查土壤墒情"
        - "叶片喷水降温"

  - name: frost_protection
    conditions:
      crop_type: strawberry
      alert_type: FROST_WARNING
    action:
      action_type: PROTECT
      priority: CRITICAL
      description: "霜冻预警，草莓需覆盖保温，必要时大棚加温。"
      steps:
        - "覆盖无纺布或草帘"
        - "检查大棚密封"
        - "准备加温设备"

  - name: frost_general
    conditions:
      alert_type: FROST_WARNING
    action:
      action_type: PROTECT
      priority: HIGH
      description: "霜冻预警，注意作物保温防寒。"
      steps:
        - "检查覆盖物"
        - "减少灌溉"

  - name: continuous_rain_monitor
    conditions:
      forecast_rain_days: 3
    action:
      action_type: MONITOR
      priority: MEDIUM
      description: "未来连续3天以上降雨，注意病害监测，雨后检查。"
      steps:
        - "增加巡检频次"
        - "关注叶面湿润情况"
        - "雨后检查病害迹象"

  # --- existing rules below ---
  - name: critical_late_blight
    ...
```

- [ ] **Step 2: Commit**

```bash
git add config/advisory_rules.yaml
git commit -m "feat: add weather-aware advisory rules"
```

---

## Phase 4: Frontend Enhancement

### Task 14: Add weather mock data and subscriber to ros.js

**Files:**
- Modify: `src/sentry_mission/static_v2/ros.js`

- [ ] **Step 1: Add weather state to store**

Add inside `window.store = Vue.reactive({...})` after the existing `fusionResults: []`:
```javascript
  weatherDays: [],
  weatherHours: [],
  weatherDisasterAlerts: [],
  weatherStale: false,
  weatherCity: '',
  weatherLat: 39.9,
  weatherLon: 116.4,
```

- [ ] **Step 2: Add weather topic subscription**

Add to the `TOPICS` array, before `/fusion/diagnosis`:
```javascript
  ['/weather/forecast', 'sentry_interfaces/WeatherForecast',
   (msg) => {
     store.weatherDays = msg.days || [];
     store.weatherHours = msg.hours || [];
     store.weatherDisasterAlerts = msg.disaster_alerts || [];
     store.weatherStale = msg.stale;
     store.weatherCity = msg.city;
   }],
```

- [ ] **Step 3: Add weather mock data in injectMock()**

Add inside `injectMock()`, before the `// === MOCK START: forecast alerts` section:
```javascript
  // Weather mock
  store.weatherDays = [
    { day_offset: 0, temp_high: 28, temp_low: 20, humidity: 75, precipitation: 0, wind_speed: 3, weather_desc: '晴' },
    { day_offset: 1, temp_high: 30, temp_low: 22, humidity: 80, precipitation: 2, wind_speed: 5, weather_desc: '多云转小雨' },
    { day_offset: 2, temp_high: 26, temp_low: 19, humidity: 92, precipitation: 25, wind_speed: 12, weather_desc: '暴雨' },
    { day_offset: 3, temp_high: 24, temp_low: 17, humidity: 88, precipitation: 8, wind_speed: 8, weather_desc: '中雨转阴' },
    { day_offset: 4, temp_high: 27, temp_low: 18, humidity: 70, precipitation: 1, wind_speed: 4, weather_desc: '多云' },
    { day_offset: 5, temp_high: 29, temp_low: 20, humidity: 65, precipitation: 0, wind_speed: 3, weather_desc: '晴' },
    { day_offset: 6, temp_high: 31, temp_low: 21, humidity: 60, precipitation: 0, wind_speed: 2, weather_desc: '晴' },
  ];
  store.weatherDisasterAlerts = ['暴雨蓝色预警'];
  store.weatherStale = false;
  store.weatherCity = '北京';
```

- [ ] **Step 4: Commit**

```bash
git add src/sentry_mission/static_v2/ros.js
git commit -m "feat: add weather store state, subscriber, and mock data"
```

---

### Task 15: Create weather-panel component

**Files:**
- Create: `src/sentry_mission/static_v2/components/weather-panel.js`

- [ ] **Step 1: Create the component**

```javascript
const WeatherPanel = {
  template: `
  <div class="card">
    <h3>
      未来天气
      <span class="count-badge danger" v-if="store.weatherDisasterAlerts.length > 0">
        {{ store.weatherDisasterAlerts.length }}预警
      </span>
      <span class="stale-badge" v-if="store.weatherStale">缓存</span>
    </h3>
    <div ref="chart" class="forecast-chart" style="height:180px"></div>
    <div class="weather-days">
      <div v-for="d in store.weatherDays" :key="d.day_offset"
           class="weather-day-row">
        <span class="day-label">{{ dayLabel(d.day_offset) }}</span>
        <span class="day-icon">{{ weatherIcon(d.weather_desc) }}</span>
        <span class="day-desc">{{ d.weather_desc }}</span>
        <span class="day-temp">{{ d.temp_low.toFixed(0) }}° / {{ d.temp_high.toFixed(0) }}°</span>
        <span class="day-rain" v-if="d.precipitation > 0">{{ d.precipitation.toFixed(0) }}mm</span>
      </div>
    </div>
    <div v-if="store.weatherDisasterAlerts.length > 0" class="disaster-alerts">
      <div v-for="a in store.weatherDisasterAlerts" :key="a" class="disaster-tag">
        {{ a }}
      </div>
    </div>
  </div>`,
  methods: {
    dayLabel(offset) {
      if (offset === 0) return '今天';
      if (offset === 1) return '明天';
      const d = new Date();
      d.setDate(d.getDate() + offset);
      return ['周日', '周一', '周二', '周三', '周四', '周五', '周六'][d.getDay()];
    },
    weatherIcon(desc) {
      const map = { '晴': '☀️', '多云': '⛅', '阴': '☁️', '小雨': '🌧️',
                    '中雨': '🌧️', '暴雨': '⛈️', '雨': '🌧️', '雪': '❄️' };
      for (const [k, v] of Object.entries(map)) {
        if (desc.includes(k)) return v;
      }
      return '🌤️';
    },
    renderChart() {
      const dom = this.$refs.chart;
      if (!dom) return;
      if (this._chart) this._chart.dispose();

      const days = this.store.weatherDays;
      if (days.length === 0) { this._chart = null; return; }

      this._chart = echarts.init(dom, null, { devicePixelRatio: 2 });
      const labels = days.map(d => this.dayLabel(d.day_offset));
      const highs = days.map(d => d.temp_high);
      const lows = days.map(d => d.temp_low);
      const rain = days.map(d => d.precipitation);

      this._chart.setOption({
        grid: { top: 20, right: 50, bottom: 30, left: 40 },
        xAxis: {
          type: 'category', data: labels,
          axisLabel: { color: '#64748B', fontSize: 10, fontFamily: 'JetBrains Mono' },
          axisLine: { lineStyle: { color: '#1F2937' } },
        },
        yAxis: [
          {
            type: 'value', name: '°C',
            axisLabel: { color: '#64748B', fontSize: 10 },
            splitLine: { lineStyle: { color: '#1F2937', type: 'dashed' } },
          },
          {
            type: 'value', name: 'mm',
            axisLabel: { color: '#64748B', fontSize: 10 },
            splitLine: { show: false },
          },
        ],
        series: [
          {
            type: 'line', data: highs, name: '最高温',
            lineStyle: { color: '#EF4444', width: 2 },
            symbol: 'circle', symbolSize: 4,
            itemStyle: { color: '#EF4444' },
            smooth: true,
          },
          {
            type: 'line', data: lows, name: '最低温',
            lineStyle: { color: '#38BDF8', width: 2 },
            symbol: 'circle', symbolSize: 4,
            itemStyle: { color: '#38BDF8' },
            smooth: true,
          },
          {
            type: 'bar', data: rain, name: '降水', yAxisIndex: 1,
            itemStyle: { color: '#6366F1' },
            barWidth: 12,
          },
        ],
        tooltip: {
          trigger: 'axis',
          backgroundColor: '#0F172A',
          borderColor: '#1F2937',
          textStyle: { color: '#F8FAFC', fontSize: 11 },
        },
      });
    },
  },
  watch: {
    'store.weatherDays.length'() { this.$nextTick(() => this.renderChart()); },
  },
  mounted() { this.$nextTick(() => this.renderChart()); },
  beforeUnmount() { if (this._chart) this._chart.dispose(); },
};
```

- [ ] **Step 2: Commit**

```bash
git add src/sentry_mission/static_v2/components/weather-panel.js
git commit -m "feat: add weather forecast panel component"
```

---

### Task 16: Register WeatherPanel and add CSS

**Files:**
- Modify: `src/sentry_mission/static_v2/app.js`
- Modify: `src/sentry_mission/static_v2/index.html`
- Modify: `src/sentry_mission/static_v2/style.css`

- [ ] **Step 1: Register component in app.js**

Add after `app.component('ForecastPanel', ForecastPanel);`:
```javascript
app.component('WeatherPanel', WeatherPanel);
```

- [ ] **Step 2: Add script tag to index.html**

Add after the forecast-panel script tag:
```html
<script src="components/weather-panel.js"></script>
```

- [ ] **Step 3: Add WeatherPanel to index.html layout**

Add after `<forecast-panel></forecast-panel>`:
```html
          <weather-panel></weather-panel>
```

- [ ] **Step 4: Add CSS**

```css
.weather-days { margin-top: 8px; }
.weather-day-row { display: flex; align-items: center; gap: 8px; padding: 4px 0; border-bottom: 1px solid #1F2937; font-size: 12px; }
.weather-day-row:last-child { border-bottom: none; }
.day-label { width: 40px; color: #64748B; }
.day-icon { width: 24px; text-align: center; }
.day-desc { flex: 1; color: #94A3B8; }
.day-temp { color: #F8FAFC; min-width: 80px; text-align: right; }
.day-rain { color: #6366F1; min-width: 40px; text-align: right; }
.disaster-alerts { margin-top: 8px; display: flex; flex-wrap: wrap; gap: 6px; }
.disaster-tag { background: #7F1D1D; color: #FCA5A5; padding: 2px 8px; border-radius: 4px; font-size: 12px; font-weight: 600; }
.stale-badge { background: #78350F; color: #FBBF24; padding: 1px 6px; border-radius: 4px; font-size: 11px; margin-left: 8px; }
```

- [ ] **Step 5: Commit**

```bash
git add src/sentry_mission/static_v2/app.js src/sentry_mission/static_v2/index.html src/sentry_mission/static_v2/style.css
git commit -m "feat: register weather panel and add styles"
```

---

### Task 17: Add location config to frontend settings

**Files:**
- Modify: `src/sentry_mission/static_v2/components/crop-selector.js` (or a new settings component)

The frontend already has a `crop-selector` component. Add location fields there since it's the crop settings area. Alternatively, update `top-bar.js` or `status-bar.js`.

Since modifying the settings area is simpler, add location inputs to the `CropSelector` template.

- [ ] **Step 1: Add store fields for location config**

In ros.js store, already added `weatherLat` and `weatherLon` in Task 14. Add a method to update them:
```javascript
  setWeatherLocation(lat, lon) {
    store.weatherLat = lat;
    store.weatherLon = lon;
    // Call ROS2 parameter service if connected
    if (ros) {
      const topic = new ROSLIB.Topic({
        ros, name: '/weather/set_location', messageType: 'sentry_interfaces/WeatherForecast'
      });
      topic.publish(new ROSLIB.Message({
        lat: lat, lon: lon
      }));
    }
  },
```

- [ ] **Step 2: Minimal location inputs in index.html (added above weather-panel)**

Since the crop-selector is modal-based, keep it simple and add inline location inputs directly in index.html next to the weather panel:
```html
          <div class="card" style="margin-bottom:8px">
            <div style="display:flex; gap:8px; align-items:center;">
              <span style="color:#64748B; font-size:12px;">📍</span>
              <input type="number" :value="store.weatherLat" step="0.01"
                     @change="e => store.weatherLat = parseFloat(e.target.value)"
                     style="width:80px; background:#1E293B; border:1px solid #334155; color:#F8FAFC; padding:4px 8px; border-radius:4px; font-size:12px;" placeholder="纬度">
              <input type="number" :value="store.weatherLon" step="0.01"
                     @change="e => store.weatherLon = parseFloat(e.target.value)"
                     style="width:80px; background:#1E293B; border:1px solid #334155; color:#F8FAFC; padding:4px 8px; border-radius:4px; font-size:12px;" placeholder="经度">
              <span style="color:#64748B; font-size:11px;">{{ store.weatherCity }}</span>
            </div>
          </div>
```

- [ ] **Step 3: Commit**

```bash
git add src/sentry_mission/static_v2/ros.js src/sentry_mission/static_v2/index.html
git commit -m "feat: add weather location config inputs to frontend"
```

---

## Plan Summary

| Phase | Tasks | Files Created | Files Modified |
|---|---|---|---|
| Phase 1: Messages + Weather | 1-7 | 10 | 2 |
| Phase 2: Forecast | 8-10 | 0 | 2 |
| Phase 3: Advisory | 11-13 | 0 | 3 |
| Phase 4: Frontend | 14-17 | 1 | 5 |

**Total: 17 tasks, 11 new files, 12 modified files**
