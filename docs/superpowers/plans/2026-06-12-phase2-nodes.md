# Phase 2 节点实现计划

> **For agentic workers:** REQUIRED SUB-SKILL: Use `superpowers:subagent-driven-development` (recommended) or `superpowers:executing-plans` to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 实现 Phase 2 三个 ROS2 Python 节点：`forecast_node`、`advisory_node`、`data_logger_node`，并补齐对应配置与测试。

**Architecture:** 按节点拆包（`sentry_forecast` / `sentry_advisory` / `sentry_data_logger`），每个包内含节点实现、配置、单元测试；`data_logger_node` 使用 `rosbag2_py` 的 `SequentialWriter` 做 bag 录制，CRITICAL 事件触发快照复制；`advisory_node` 通过 YAML 规则引擎匹配 `FusionResult` / `ForecastAlert` / `Environment` 生成建议。

**Tech Stack:** ROS2 Humble + `rclpy` + `ament_python` + `sentry_interfaces` + `rosbag2_py` + `PyYAML` + `pytest`

---

## 文件结构总览

```text
src/
├── sentry_forecast/
│   ├── sentry_forecast/
│   │   ├── __init__.py
│   │   └── forecast_node.py
│   ├── config/
│   │   └── forecast_params.yaml
│   ├── tests/
│   │   └── test_forecast_node.py
│   ├── resource/
│   │   └── sentry_forecast
│   ├── setup.py
│   ├── setup.cfg
│   └── package.xml
├── sentry_advisory/
│   ├── sentry_advisory/
│   │   ├── __init__.py
│   │   ├── advisory_node.py
│   │   └── rule_engine.py
│   ├── config/
│   │   └── advisory_rules.yaml
│   ├── tests/
│   │   └── test_advisory_node.py
│   ├── resource/
│   │   └── sentry_advisory
│   ├── setup.py
│   ├── setup.cfg
│   └── package.xml
└── sentry_data_logger/
    ├── sentry_data_logger/
    │   ├── __init__.py
    │   ├── data_logger_node.py
    │   └── bag_writer.py
    ├── config/
    │   └── data_logger_params.yaml
    ├── tests/
    │   └── test_data_logger_node.py
    ├── resource/
    │   └── sentry_data_logger
    ├── setup.py
    ├── setup.cfg
    └── package.xml

config/
├── advisory_rules.yaml
├── forecast_params.yaml
└── data_logger_params.yaml

PLAN.md                          # 根目录任务追踪
src/sentry_bringup/launch/sentry_v2.launch.py   # 追加三个节点
```

---

## Task 0：创建/更新根目录 `PLAN.md`

**Files:**
- Create/Modify: `PLAN.md`

- [ ] **Step 1：写入任务追踪清单**

```markdown
# Phase 2 节点实现追踪

- [ ] Task 1：创建三个 ROS2 Python 包骨架
- [ ] Task 2：添加 Phase 2 配置文件
- [ ] Task 3：实现并测试 `forecast_node`
- [ ] Task 4：实现并测试 `advisory_node`
- [ ] Task 5：实现并测试 `data_logger_node`
- [ ] Task 6：在 `sentry_v2.launch.py` 中注册三个节点
- [ ] Task 7：编译并运行 colcon test
- [ ] Task 8：最终审查与提交
```

- [ ] **Step 2：提交 `PLAN.md`（仅当用户明确同意提交时）**

---

## Task 1a：创建 `sentry_forecast` 包骨架

**Files:**
- Create: `src/sentry_forecast/setup.py`
- Create: `src/sentry_forecast/setup.cfg`
- Create: `src/sentry_forecast/package.xml`
- Create: `src/sentry_forecast/resource/sentry_forecast`
- Create: `src/sentry_forecast/sentry_forecast/__init__.py`
- Create: `src/sentry_forecast/config/forecast_params.yaml`

- [ ] **Step 1：创建 `src/sentry_forecast/setup.py`**

```python
from setuptools import find_packages, setup

package_name = 'sentry_forecast'

setup(
    name=package_name,
    version='0.2.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/forecast_params.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='team',
    maintainer_email='team@example.com',
    description='Forecast node for Smart Agri Sentry v2.0',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'forecast_node = sentry_forecast.forecast_node:main',
        ],
    },
)
```

- [ ] **Step 2：创建 `src/sentry_forecast/setup.cfg`**

```ini
[develop]
script_dir=$base/lib/sentry_forecast
[install]
install_scripts=$base/lib/sentry_forecast
```

- [ ] **Step 3：创建 `src/sentry_forecast/package.xml`**

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>sentry_forecast</name>
  <version>0.2.0</version>
  <description>Forecast node for Smart Agri Sentry v2.0</description>
  <maintainer email="team@example.com">team</maintainer>
  <license>MIT</license>

  <depend>rclpy</depend>
  <depend>std_msgs</depend>
  <depend>sentry_interfaces</depend>

  <test_depend>ament_copyright</test_depend>
  <test_depend>ament_flake8</test_depend>
  <test_depend>ament_pep257</test_depend>
  <test_depend>python3-pytest</test_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
```

- [ ] **Step 4：创建空资源文件 `src/sentry_forecast/resource/sentry_forecast`**

内容为空文件即可。

- [ ] **Step 5：创建 `src/sentry_forecast/sentry_forecast/__init__.py`**

```python
# sentry_forecast package
```

- [ ] **Step 6：创建 `src/sentry_forecast/config/forecast_params.yaml`**

```yaml
forecast_node:
  timer_period_sec: 600
  history_hours: 6
  prediction_hours: 24
  risk_threshold: 0.7
  lwd_margin_hours: 2.0
  humidity_trend_threshold: 0.3
```

---

## Task 1b：创建 `sentry_advisory` 包骨架

**Files:**
- Create: `src/sentry_advisory/setup.py`
- Create: `src/sentry_advisory/setup.cfg`
- Create: `src/sentry_advisory/package.xml`
- Create: `src/sentry_advisory/resource/sentry_advisory`
- Create: `src/sentry_advisory/sentry_advisory/__init__.py`
- Create: `src/sentry_advisory/sentry_advisory/rule_engine.py`
- Create: `src/sentry_advisory/config/advisory_rules.yaml`

- [ ] **Step 1：创建 `src/sentry_advisory/setup.py`**

```python
from setuptools import find_packages, setup

package_name = 'sentry_advisory'

setup(
    name=package_name,
    version='0.2.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/advisory_rules.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='team',
    maintainer_email='team@example.com',
    description='Advisory node for Smart Agri Sentry v2.0',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'advisory_node = sentry_advisory.advisory_node:main',
        ],
    },
)
```

- [ ] **Step 2：创建 `src/sentry_advisory/setup.cfg`**

```ini
[develop]
script_dir=$base/lib/sentry_advisory
[install]
install_scripts=$base/lib/sentry_advisory
```

- [ ] **Step 3：创建 `src/sentry_advisory/package.xml`**

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>sentry_advisory</name>
  <version>0.2.0</version>
  <description>Advisory node for Smart Agri Sentry v2.0</description>
  <maintainer email="team@example.com">team</maintainer>
  <license>MIT</license>

  <depend>rclpy</depend>
  <depend>std_msgs</depend>
  <depend>sentry_interfaces</depend>

  <test_depend>ament_copyright</test_depend>
  <test_depend>ament_flake8</test_depend>
  <test_depend>ament_pep257</test_depend>
  <test_depend>python3-pytest</test_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
```

- [ ] **Step 4：创建空资源文件 `src/sentry_advisory/resource/sentry_advisory`**

- [ ] **Step 5：创建 `src/sentry_advisory/sentry_advisory/__init__.py`**

```python
# sentry_advisory package
```

- [ ] **Step 6：创建 `src/sentry_advisory/config/advisory_rules.yaml`**

```yaml
rules:
  - name: critical_late_blight
    conditions:
      crop_type: tomato
      alert_level: CRITICAL
      mode: VISION_DOMINANT
    action:
      action_type: SPRAY
      priority: CRITICAL
      description: "检测到晚疫病高风险，建议立即喷洒杀菌剂。"
      steps:
        - "停车并确认植株编号"
        - "使用对应杀菌剂喷洒"
        - "记录处理位置"

  - name: latent_outbreak
    conditions:
      crop_type: tomato
      alert_type: LATENT_OUTBREAK
    action:
      action_type: MONITOR
      priority: HIGH
      description: "环境条件利于病害爆发，建议增加巡检频次。"

  - name: drought_stress
    conditions:
      humidity_max: 40
      temperature_min: 30
    action:
      action_type: IRRIGATE
      priority: MEDIUM
      description: "干旱胁迫风险，建议适时灌溉。"

  - name: rising_risk
    conditions:
      alert_type: RISING_RISK
      risk_min: 0.6
    action:
      action_type: MONITOR
      priority: HIGH
      description: "风险呈上升趋势，建议密切观察。"
```

- [ ] **Step 7：创建占位 `src/sentry_advisory/sentry_advisory/rule_engine.py`**

```python
class RuleEngine:
    pass
```

---

## Task 1c：创建 `sentry_data_logger` 包骨架

**Files:**
- Create: `src/sentry_data_logger/setup.py`
- Create: `src/sentry_data_logger/setup.cfg`
- Create: `src/sentry_data_logger/package.xml`
- Create: `src/sentry_data_logger/resource/sentry_data_logger`
- Create: `src/sentry_data_logger/sentry_data_logger/__init__.py`
- Create: `src/sentry_data_logger/sentry_data_logger/bag_writer.py`
- Create: `src/sentry_data_logger/config/data_logger_params.yaml`

- [ ] **Step 1：创建 `src/sentry_data_logger/setup.py`**

```python
from setuptools import find_packages, setup

package_name = 'sentry_data_logger'

setup(
    name=package_name,
    version='0.2.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/data_logger_params.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='team',
    maintainer_email='team@example.com',
    description='Data logger node for Smart Agri Sentry v2.0',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'data_logger_node = sentry_data_logger.data_logger_node:main',
        ],
    },
)
```

- [ ] **Step 2：创建 `src/sentry_data_logger/setup.cfg`**

```ini
[develop]
script_dir=$base/lib/sentry_data_logger
[install]
install_scripts=$base/lib/sentry_data_logger
```

- [ ] **Step 3：创建 `src/sentry_data_logger/package.xml`**

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>sentry_data_logger</name>
  <version>0.2.0</version>
  <description>Data logger node for Smart Agri Sentry v2.0</description>
  <maintainer email="team@example.com">team</maintainer>
  <license>MIT</license>

  <depend>rclpy</depend>
  <depend>rosbag2_py</depend>
  <depend>std_msgs</depend>
  <depend>sensor_msgs</depend>
  <depend>sentry_interfaces</depend>

  <test_depend>ament_copyright</test_depend>
  <test_depend>ament_flake8</test_depend>
  <test_depend>ament_pep257</test_depend>
  <test_depend>python3-pytest</test_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
```

- [ ] **Step 4：创建空资源文件 `src/sentry_data_logger/resource/sentry_data_logger`**

- [ ] **Step 5：创建 `src/sentry_data_logger/sentry_data_logger/__init__.py`**

```python
# sentry_data_logger package
```

- [ ] **Step 6：创建 `src/sentry_data_logger/config/data_logger_params.yaml`**

```yaml
data_logger_node:
  topics:
    - /fusion/diagnosis
    - /mission/status
    - /forecast/alert
    - /advisory/action
    - /sensor/environment_mobile
    - /vision/diagnosis
  bag_base_dir: bags
  split_duration_sec: 900
  split_max_size_mb: 1024
  retention_days: 7
  critical_retention_sec: 300
  record_metadata: true
```

- [ ] **Step 7：创建占位 `src/sentry_data_logger/sentry_data_logger/bag_writer.py`**

```python
class BagWriter:
    pass
```

---

## Task 2：添加项目级配置文件

**Files:**
- Create: `config/forecast_params.yaml`
- Create: `config/advisory_rules.yaml`
- Create: `config/data_logger_params.yaml`

- [ ] **Step 1：复制各包 `config/*.yaml` 到项目根目录 `config/`**

```bash
cp src/sentry_forecast/config/forecast_params.yaml config/forecast_params.yaml
cp src/sentry_advisory/config/advisory_rules.yaml config/advisory_rules.yaml
cp src/sentry_data_logger/config/data_logger_params.yaml config/data_logger_params.yaml
```

- [ ] **Step 2：确认 `config/` 目录包含以下文件**

```text
config/
├── crop_profiles.yaml
├── advisory_rules.yaml
├── forecast_params.yaml
└── data_logger_params.yaml
```

---

## Task 3：实现并测试 `forecast_node`

**Files:**
- Create: `src/sentry_forecast/sentry_forecast/forecast_node.py`
- Create: `src/sentry_forecast/tests/test_forecast_node.py`

### 3.1 纯趋势预测逻辑（先测试后实现）

- [ ] **Step 1：写失败测试 `test_forecast_node.py` 中的 `TrendForecaster` 测试**

```python
import pytest
import rclpy
from sentry_forecast.forecast_node import TrendForecaster, ForecastNode


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
```

Run:

```bash
cd src/sentry_forecast
python -m pytest tests/test_forecast_node.py -v
```

Expected: FAIL（`TrendForecaster` 未定义）

- [ ] **Step 2：实现 `TrendForecaster` 类**

在 `src/sentry_forecast/sentry_forecast/forecast_node.py` 顶部加入：

```python
class TrendForecaster:
    """Simple linear trend extrapolation helper."""

    @staticmethod
    def linear_trend(samples, key='risk_score'):
        if len(samples) < 2:
            return 0.0
        x = [(s['timestamp'] - samples[0]['timestamp']) / 3600.0
             for s in samples]
        y = [s[key] for s in samples]
        mean_x = sum(x) / len(x)
        mean_y = sum(y) / len(y)
        num = sum((xi - mean_x) * (yi - mean_y) for xi, yi in zip(x, y))
        den = sum((xi - mean_x) ** 2 for xi in x)
        if den == 0.0:
            return 0.0
        return num / den

    @staticmethod
    def predict(samples, prediction_hours, key='risk_score'):
        if not samples:
            return 0.0
        if len(samples) < 2:
            return float(samples[-1][key])
        slope = TrendForecaster.linear_trend(samples, key)
        last = samples[-1][key]
        return max(0.0, min(1.0, last + slope * prediction_hours))
```

Run:

```bash
python -m pytest tests/test_forecast_node.py -v
```

Expected: PASS

### 3.2 `ForecastNode` 实现

- [ ] **Step 3：写失败测试 `ForecastNode._predict_alert()`**

在 `test_forecast_node.py` 中追加：

```python
from sentry_interfaces.msg import FusionResult, Environment, ForecastAlert


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
```

Run:

```bash
python -m pytest tests/test_forecast_node.py -v
```

Expected: FAIL（`_predict_alert` 未实现）

- [ ] **Step 4：实现完整 `forecast_node.py`**

```python
import os
import time
import yaml

import rclpy
from rclpy.node import Node

from sentry_interfaces.msg import (
    Environment,
    ForecastAlert,
    FusionResult,
)


class TrendForecaster:
    """Simple linear trend extrapolation helper."""

    @staticmethod
    def linear_trend(samples, key='risk_score'):
        if len(samples) < 2:
            return 0.0
        x = [(s['timestamp'] - samples[0]['timestamp']) / 3600.0
             for s in samples]
        y = [s[key] for s in samples]
        mean_x = sum(x) / len(x)
        mean_y = sum(y) / len(y)
        num = sum((xi - mean_x) * (yi - mean_y) for xi, yi in zip(x, y))
        den = sum((xi - mean_x) ** 2 for xi in x)
        if den == 0.0:
            return 0.0
        return num / den

    @staticmethod
    def predict(samples, prediction_hours, key='risk_score'):
        if not samples:
            return 0.0
        if len(samples) < 2:
            return float(samples[-1][key])
        slope = TrendForecaster.linear_trend(samples, key)
        last = samples[-1][key]
        return max(0.0, min(1.0, last + slope * prediction_hours))


ALERT_NONE = 'NONE'
ALERT_RISING_RISK = 'RISING_RISK'
ALERT_LATENT_OUTBREAK = 'LATENT_OUTBREAK'
ALERT_DROUGHT_STRESS = 'DROUGHT_STRESS'


class ForecastNode(Node):
    def __init__(self):
        super().__init__('forecast_node')
        self.declare_parameter('crop_type', 'tomato')
        self.declare_parameter(
            'crop_profiles_path', 'config/crop_profiles.yaml')
        self.declare_parameter(
            'forecast_params_path', 'config/forecast_params.yaml')
        self.declare_parameter('mobile_stale_sec', 2.0)
        self.declare_parameter('fusion_stale_sec', 30.0)

        self.crop_type = self.get_parameter('crop_type').value
        self.mobile_stale_sec = self.get_parameter('mobile_stale_sec').value
        self.fusion_stale_sec = self.get_parameter('fusion_stale_sec').value

        self.profiles = self._load_profiles(
            self.get_parameter('crop_profiles_path').value)
        self.profile = self.profiles.get(self.crop_type, {})
        self.params = self._load_params(
            self.get_parameter('forecast_params_path').value)

        self.history = []
        self.last_fusion = None
        self.last_fusion_ts = 0.0
        self.last_env = None
        self.last_env_ts = 0.0

        self.sub_fusion = self.create_subscription(
            FusionResult, '/fusion/diagnosis', self.on_fusion, 10)
        self.sub_env = self.create_subscription(
            Environment, '/sensor/environment_mobile', self.on_env, 10)

        period = self.params.get('timer_period_sec', 600)
        self.timer = self.create_timer(float(period), self.tick)
        self.pub = self.create_publisher(
            ForecastAlert, '/forecast/alert', 10)

        self.get_logger().info(
            f'Forecast node ready (crop={self.crop_type})')

    def _load_profiles(self, path):
        if not os.path.isabs(path):
            ws = os.environ.get('COLCON_PREFIX_PATH', os.getcwd())
            candidates = [
                os.path.join(ws, '..', '..', path),
                os.path.join(ws, path),
                path,
            ]
            for c in candidates:
                if os.path.exists(c):
                    path = c
                    break
        if os.path.exists(path):
            with open(path, 'r') as f:
                return yaml.safe_load(f) or {}
        self.get_logger().warn(f'Crop profile not found: {path}, using defaults')
        return {}

    def _load_params(self, path):
        if not os.path.isabs(path):
            ws = os.environ.get('COLCON_PREFIX_PATH', os.getcwd())
            candidates = [
                os.path.join(ws, '..', '..', path),
                os.path.join(ws, path),
                path,
            ]
            for c in candidates:
                if os.path.exists(c):
                    path = c
                    break
        if os.path.exists(path):
            with open(path, 'r') as f:
                data = yaml.safe_load(f) or {}
            return data.get('forecast_node', data)
        self.get_logger().warn(f'Forecast params not found: {path}, using defaults')
        return {}

    def on_fusion(self, msg: FusionResult):
        now = self.get_clock().now().nanoseconds / 1e9
        self.last_fusion = msg
        self.last_fusion_ts = now

    def on_env(self, msg: Environment):
        now = self.get_clock().now().nanoseconds / 1e9
        self.last_env = msg
        self.last_env_ts = now

        env_ok = self.last_env is not None and (
            now - self.last_env_ts) <= self.mobile_stale_sec
        sample = {
            'timestamp': now,
            'risk_score': (self.last_fusion.risk_score
                           if self.last_fusion is not None else 0.0),
            'humidity': msg.air_humidity,
            'lwd_hours': (self.last_fusion.lwd_hours
                          if self.last_fusion is not None else 0.0),
            'temperature': msg.air_temp,
        }
        if env_ok:
            self.history.append(sample)

    def tick(self):
        alert = self._predict_alert()
        self.pub.publish(alert)

    def _prune_history(self, now):
        window = self.params.get('history_hours', 6) * 3600.0
        cutoff = now - window
        self.history = [h for h in self.history if h['timestamp'] > cutoff]

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
            return msg

        if (now - self.last_fusion_ts) > self.fusion_stale_sec:
            msg.active = False
            msg.alert_type = ALERT_NONE
            msg.probability = 0.0
            msg.description = 'Fusion data stale'
            return msg

        prediction_hours = self.params.get('prediction_hours', 24)
        risk_threshold = self.params.get('risk_threshold', 0.7)
        lwd_margin = self.params.get('lwd_margin_hours', 2.0)
        hum_trend_th = self.params.get('humidity_trend_threshold', 0.3)
        lwd_threshold = self.profile.get('lwd_threshold_hours', 6.0)

        predicted_risk = TrendForecaster.predict(
            self.history, prediction_hours, 'risk_score')
        predicted_humidity = TrendForecaster.predict(
            self.history, prediction_hours, 'humidity')
        humidity_slope = TrendForecaster.linear_trend(self.history, 'humidity')

        alert_type = ALERT_NONE
        description = '风险平稳，无需预警'

        # Rising risk: predicted risk high and trend increasing
        risk_slope = TrendForecaster.linear_trend(self.history, 'risk_score')
        if predicted_risk >= risk_threshold and risk_slope > 0:
            alert_type = ALERT_RISING_RISK
            description = (
                f'预测 24h 风险 {predicted_risk:.2f}，呈上升趋势')
        # Latent outbreak: LWD close to threshold and humidity rising
        elif (self.last_fusion.lwd_hours >= (lwd_threshold - lwd_margin)
              and humidity_slope >= hum_trend_th):
            alert_type = ALERT_LATENT_OUTBREAK
            description = (
                f'LWD 接近阈值 ({self.last_fusion.lwd_hours:.1f}h / '
                f'{lwd_threshold:.1f}h)，湿度持续上升')
        # Drought stress
        elif (self.last_env is not None
              and (now - self.last_env_ts) <= self.mobile_stale_sec
              and self.last_env.air_humidity <= 40.0
              and self.last_env.air_temp >= 30.0):
            alert_type = ALERT_DROUGHT_STRESS
            description = (
                f'干旱胁迫：温度 {self.last_env.air_temp:.1f}C，'
                f'湿度 {self.last_env.air_humidity:.1f}%')

        msg.active = alert_type != ALERT_NONE
        msg.alert_type = alert_type
        msg.probability = float(predicted_risk)
        msg.description = description
        return msg


def main(args=None):
    rclpy.init(args=args)
    node = ForecastNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
```

- [ ] **Step 5：运行 forecast 测试**

```bash
cd src/sentry_forecast
python -m pytest tests/test_forecast_node.py -v
```

Expected: PASS

---

## Task 4：实现并测试 `advisory_node`

**Files:**
- Modify: `src/sentry_advisory/sentry_advisory/rule_engine.py`
- Create: `src/sentry_advisory/sentry_advisory/advisory_node.py`
- Create: `src/sentry_advisory/tests/test_advisory_node.py`

### 4.1 规则引擎（先测试后实现）

- [ ] **Step 1：写失败测试 `test_advisory_node.py`**

```python
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
```

Run:

```bash
cd src/sentry_advisory
python -m pytest tests/test_advisory_node.py -v
```

Expected: FAIL（`RuleEngine` 未实现）

- [ ] **Step 2：实现 `rule_engine.py`**

```python
import os
import yaml


ALERT_LEVEL_MAP = {
    'NORMAL': 0,
    'SUSPICION': 1,
    'WARNING': 2,
    'CRITICAL': 3,
}

_PRIORITY_ORDER = {
    'CRITICAL': 0,
    'HIGH': 1,
    'MEDIUM': 2,
    'LOW': 3,
}


class RuleEngine:
    """YAML-based rule engine for advisory generation."""

    def __init__(self, rules):
        self.rules = rules

    @classmethod
    def from_yaml(cls, path):
        if not os.path.isabs(path):
            ws = os.environ.get('COLCON_PREFIX_PATH', os.getcwd())
            candidates = [
                os.path.join(ws, '..', '..', path),
                os.path.join(ws, path),
                path,
            ]
            for c in candidates:
                if os.path.exists(c):
                    path = c
                    break
        if os.path.exists(path):
            with open(path, 'r') as f:
                data = yaml.safe_load(f) or {}
            return cls(data.get('rules', []))
        return cls([])

    @staticmethod
    def default_action():
        return {
            'action_type': 'NONE',
            'priority': 'LOW',
            'description': '暂无明确建议，继续监测',
            'steps': [],
        }

    def match(self, fusion, forecast, env, crop_type):
        for rule in self.rules:
            if self._match_conditions(
                    rule.get('conditions', {}), fusion, forecast, env, crop_type):
                return rule.get('action', self.default_action())
        return self.default_action()

    def _match_conditions(self, cond, fusion, forecast, env, crop_type):
        if 'crop_type' in cond and cond['crop_type'] != crop_type:
            return False
        if 'alert_level' in cond:
            level_value = ALERT_LEVEL_MAP.get(cond['alert_level'])
            if level_value is None or fusion.alert_level != level_value:
                return False
        if 'mode' in cond and fusion.mode != cond['mode']:
            return False
        if 'alert_type' in cond and forecast.alert_type != cond['alert_type']:
            return False
        if 'risk_min' in cond and fusion.risk_score < cond['risk_min']:
            return False
        if 'risk_max' in cond and fusion.risk_score > cond['risk_max']:
            return False
        if env is not None:
            if ('humidity_max' in cond
                    and env.air_humidity > cond['humidity_max']):
                return False
            if ('temperature_min' in cond
                    and env.air_temp < cond['temperature_min']):
                return False
        else:
            if 'humidity_max' in cond or 'temperature_min' in cond:
                return False
        return True

    def highest_priority_action(self, actions):
        if not actions:
            return self.default_action()
        return min(actions, key=lambda a: _PRIORITY_ORDER.get(a.get('priority', 'LOW'), 99))
```

Run:

```bash
python -m pytest tests/test_advisory_node.py -v
```

Expected: PASS（仅 RuleEngine 测试通过）

### 4.2 `AdvisoryNode` 实现

- [ ] **Step 3：写失败测试 `AdvisoryNode._evaluate()`**

在 `test_advisory_node.py` 中追加：

```python
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
```

Run:

```bash
python -m pytest tests/test_advisory_node.py -v
```

Expected: FAIL（`AdvisoryNode` 未实现）

- [ ] **Step 4：实现 `advisory_node.py`**

```python
import os

import rclpy
from rclpy.node import Node

from sentry_interfaces.msg import (
    AdvisoryAction,
    Environment,
    ForecastAlert,
    FusionResult,
)
from .rule_engine import RuleEngine


class AdvisoryNode(Node):
    def __init__(self):
        super().__init__('advisory_node')
        self.declare_parameter('crop_type', 'tomato')
        self.declare_parameter(
            'advisory_rules_path', 'config/advisory_rules.yaml')
        self.declare_parameter('fusion_stale_sec', 30.0)

        self.crop_type = self.get_parameter('crop_type').value
        self.fusion_stale_sec = self.get_parameter('fusion_stale_sec').value

        rules_path = self.get_parameter('advisory_rules_path').value
        self.engine = RuleEngine.from_yaml(rules_path)
        if not self.engine.rules:
            self.get_logger().warn(
                f'No advisory rules loaded from {rules_path}, using empty set')

        self.last_fusion = None
        self.last_fusion_ts = 0.0
        self.last_forecast = None
        self.last_env = None

        self.sub_fusion = self.create_subscription(
            FusionResult, '/fusion/diagnosis', self.on_fusion, 10)
        self.sub_forecast = self.create_subscription(
            ForecastAlert, '/forecast/alert', self.on_forecast, 10)
        self.sub_env = self.create_subscription(
            Environment, '/sensor/environment_mobile', self.on_env, 10)

        self.pub = self.create_publisher(
            AdvisoryAction, '/advisory/action', 10)

        self.get_logger().info(
            f'Advisory node ready (crop={self.crop_type})')

    def on_fusion(self, msg: FusionResult):
        self.last_fusion = msg
        self.last_fusion_ts = self.get_clock().now().nanoseconds / 1e9
        self._maybe_publish()

    def on_forecast(self, msg: ForecastAlert):
        self.last_forecast = msg
        self._maybe_publish()

    def on_env(self, msg: Environment):
        self.last_env = msg
        self._maybe_publish()

    def _maybe_publish(self):
        if self.last_fusion is None:
            return
        now = self.get_clock().now().nanoseconds / 1e9
        if (now - self.last_fusion_ts) > self.fusion_stale_sec:
            return
        action = self._evaluate(
            self.last_fusion,
            self.last_forecast,
            self.last_env)
        self.pub.publish(action)

    def _evaluate(self, fusion, forecast, env):
        forecast = forecast or ForecastAlert()
        env = env or Environment()
        matched = self.engine.match(fusion, forecast, env, self.crop_type)

        msg = AdvisoryAction()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'advisory'
        msg.action_type = matched.get('action_type', 'NONE')
        msg.description = matched.get('description', '')
        msg.priority = matched.get('priority', 'LOW')
        msg.steps = matched.get('steps', [])
        return msg


def main(args=None):
    rclpy.init(args=args)
    node = AdvisoryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
```

- [ ] **Step 5：运行 advisory 测试**

```bash
cd src/sentry_advisory
python -m pytest tests/test_advisory_node.py -v
```

Expected: PASS

---

## Task 5：实现并测试 `data_logger_node`

**Files:**
- Modify: `src/sentry_data_logger/sentry_data_logger/bag_writer.py`
- Create: `src/sentry_data_logger/sentry_data_logger/data_logger_node.py`
- Create: `src/sentry_data_logger/tests/test_data_logger_node.py`

### 5.1 `BagWriter`（先测试后实现）

- [ ] **Step 1：写失败测试 `test_data_logger_node.py`**

```python
import os
import shutil
import tempfile
import time
import pytest
import rclpy
from unittest.mock import MagicMock

from sentry_data_logger.bag_writer import BagWriter
from sentry_data_logger.data_logger_node import DataLoggerNode, ALERT_CRITICAL
from sentry_interfaces.msg import FusionResult, ForecastAlert


@pytest.fixture(scope='module')
def ros_context():
    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.fixture
def tmp_bag_dir():
    path = tempfile.mkdtemp()
    yield path
    shutil.rmtree(path, ignore_errors=True)


def test_bag_writer_opens_and_closes(tmp_bag_dir):
    writer = BagWriter(tmp_bag_dir, split_duration_sec=1)
    writer.open()
    assert writer._current_dir is not None
    writer.close()


def test_bag_writer_json_fallback_on_missing_rosbag(tmp_bag_dir, monkeypatch):
    import sys
    monkeypatch.setitem(sys.modules, 'rosbag2_py', None)
    writer = BagWriter(tmp_bag_dir)
    writer.open()
    assert writer._json_fallback is True
    writer.close()


@pytest.fixture
def node(ros_context, tmp_bag_dir):
    n = DataLoggerNode()
    n.writer = MagicMock()
    n.writer._current_dir = tmp_bag_dir
    yield n
    n.destroy_node()


def test_normal_fusion_writes_only(node):
    msg = FusionResult()
    msg.header.stamp = node.get_clock().now().to_msg()
    msg.alert_level = 1  # SUSPICION
    node._on_msg('/fusion/diagnosis', msg)
    assert node.writer.write.called
    assert not node.writer.snapshot_critical.called


def test_critical_fusion_triggers_snapshot(node):
    msg = FusionResult()
    msg.header.stamp = node.get_clock().now().to_msg()
    msg.alert_level = ALERT_CRITICAL
    node._on_msg('/fusion/diagnosis', msg)
    assert node.writer.snapshot_critical.called


def test_duplicate_critical_not_double_snapshot(node):
    msg = FusionResult()
    msg.header.stamp = node.get_clock().now().to_msg()
    msg.alert_level = ALERT_CRITICAL
    node._on_msg('/fusion/diagnosis', msg)
    node._on_msg('/fusion/diagnosis', msg)
    assert node.writer.snapshot_critical.call_count == 1
```

Run:

```bash
cd src/sentry_data_logger
python -m pytest tests/test_data_logger_node.py -v
```

Expected: FAIL（`BagWriter` / `DataLoggerNode` 未实现）

- [ ] **Step 2：实现 `bag_writer.py`**

```python
import json
import os
import shutil
import threading
import time


def _topic_type_str(msg):
    cls = msg.__class__
    module = cls.__module__
    # Convert 'sentry_interfaces.msg.FusionResult' -> 'sentry_interfaces/msg/FusionResult'
    parts = module.split('.')
    if 'msg' in parts:
        parts[parts.index('msg')] = 'msg'
    return f"{'/'.join(parts)}/{cls.__name__}"


class BagWriter:
    """Wrapper around rosbag2_py.SequentialWriter with JSON fallback."""

    def __init__(self, base_dir, split_duration_sec=900, split_max_size_mb=1024):
        self.base_dir = base_dir
        self.split_duration_sec = split_duration_sec
        self.split_max_size_mb = split_max_size_mb
        self.split_max_size_bytes = split_max_size_mb * 1024 * 1024
        self._writer = None
        self._current_dir = None
        self._start_time = 0.0
        self._topics = set()
        self._lock = threading.Lock()
        self._json_fallback = False
        self._json_file = None
        self._json_records = []

    def open(self):
        try:
            import rosbag2_py
            from rclpy.serialization import serialize_message
            self._rosbag2_py = rosbag2_py
            self._serialize_message = serialize_message
            self._new_bag()
        except Exception as e:
            self._json_fallback = True
            self._new_json()

    def _new_bag(self):
        self.close()
        timestamp = time.strftime('%Y%m%d_%H%M%S')
        self._current_dir = os.path.join(self.base_dir, timestamp)
        os.makedirs(self._current_dir, exist_ok=True)
        storage_options = self._rosbag2_py.StorageOptions(
            uri=self._current_dir,
            storage_id='sqlite3',
        )
        converter_options = self._rosbag2_py.ConverterOptions(
            input_serialization_format='cdr',
            output_serialization_format='cdr',
        )
        self._writer = self._rosbag2_py.SequentialWriter()
        self._writer.open(storage_options, converter_options)
        self._topics = set()
        self._start_time = time.time()

    def _new_json(self):
        self.close()
        timestamp = time.strftime('%Y%m%d_%H%M%S')
        self._current_dir = os.path.join(self.base_dir, timestamp)
        os.makedirs(self._current_dir, exist_ok=True)
        json_path = os.path.join(self._current_dir, 'events.jsonl')
        self._json_file = open(json_path, 'a')
        self._start_time = time.time()

    def _should_split(self):
        elapsed = time.time() - self._start_time
        if elapsed >= self.split_duration_sec:
            return True
        if self._current_dir and os.path.exists(self._current_dir):
            size = sum(
                os.path.getsize(os.path.join(dp, f))
                for dp, dn, filenames in os.walk(self._current_dir)
                for f in filenames
            )
            if size >= self.split_max_size_bytes:
                return True
        return False

    def write(self, topic, msg, timestamp_nanoseconds):
        with self._lock:
            if self._should_split():
                if self._json_fallback:
                    self._new_json()
                else:
                    self._new_bag()

            if self._json_fallback:
                record = {
                    'topic': topic,
                    'timestamp_ns': int(timestamp_nanoseconds),
                    'type': _topic_type_str(msg),
                }
                self._json_file.write(json.dumps(record) + '\n')
                self._json_file.flush()
                return

            topic_type = _topic_type_str(msg)
            if topic not in self._topics:
                self._writer.create_topic(
                    self._rosbag2_py.TopicMetadata(
                        name=topic,
                        type=topic_type,
                        serialization_format='cdr',
                    ))
                self._topics.add(topic)
            self._writer.write(
                topic,
                self._serialize_message(msg),
                int(timestamp_nanoseconds),
            )

    def snapshot_critical(self, target_dir, metadata=None):
        with self._lock:
            if not self._current_dir or not os.path.exists(self._current_dir):
                return
            os.makedirs(target_dir, exist_ok=True)
            for item in os.listdir(self._current_dir):
                src = os.path.join(self._current_dir, item)
                dst = os.path.join(target_dir, item)
                if os.path.isdir(src):
                    shutil.copytree(src, dst, dirs_exist_ok=True)
                else:
                    shutil.copy2(src, dst)
            if metadata:
                meta_path = os.path.join(target_dir, 'metadata.json')
                with open(meta_path, 'w') as f:
                    json.dump(metadata, f, indent=2)

    def close(self):
        with self._lock:
            if self._writer is not None:
                try:
                    self._writer.close()
                except Exception:
                    pass
                self._writer = None
            if self._json_file is not None:
                try:
                    self._json_file.close()
                except Exception:
                    pass
                self._json_file = None
```

- [ ] **Step 3：实现 `data_logger_node.py`**

```python
import json
import os
import shutil
import time

import rclpy
from rclpy.node import Node

from sentry_interfaces.msg import (
    AdvisoryAction,
    Diagnosis,
    Environment,
    ForecastAlert,
    FusionResult,
    MissionStatus,
)
from .bag_writer import BagWriter


ALERT_CRITICAL = 3

_TOPIC_TYPES = {
    '/fusion/diagnosis': FusionResult,
    '/mission/status': MissionStatus,
    '/forecast/alert': ForecastAlert,
    '/advisory/action': AdvisoryAction,
    '/sensor/environment_mobile': Environment,
    '/vision/diagnosis': Diagnosis,
}


class DataLoggerNode(Node):
    def __init__(self):
        super().__init__('data_logger_node')
        self.declare_parameter('topics', [
            '/fusion/diagnosis',
            '/mission/status',
            '/forecast/alert',
            '/advisory/action',
            '/sensor/environment_mobile',
            '/vision/diagnosis',
        ])
        self.declare_parameter('bag_base_dir', 'bags')
        self.declare_parameter('split_duration_sec', 900)
        self.declare_parameter('split_max_size_mb', 1024)
        self.declare_parameter('retention_days', 7)
        self.declare_parameter('critical_retention_sec', 300)
        self.declare_parameter('record_metadata', True)

        topics = self.get_parameter('topics').value
        base_dir = self.get_parameter('bag_base_dir').value
        split_duration = self.get_parameter('split_duration_sec').value
        split_size = self.get_parameter('split_max_size_mb').value
        self.retention_days = self.get_parameter('retention_days').value
        self.critical_retention_sec = self.get_parameter(
            'critical_retention_sec').value
        self.record_metadata = self.get_parameter('record_metadata').value

        self.writer = BagWriter(
            base_dir=base_dir,
            split_duration_sec=split_duration,
            split_max_size_mb=split_size,
        )
        self.writer.open()

        self._latest = {}
        self._critical_keys = set()
        self._subscriptions = []
        for topic in topics:
            msg_type = _TOPIC_TYPES.get(topic)
            if msg_type is None:
                self.get_logger().warn(f'Unknown topic type for {topic}, skipping')
                continue
            sub = self.create_subscription(
                msg_type,
                topic,
                lambda msg, t=topic: self._on_msg(t, msg),
                10,
            )
            self._subscriptions.append(sub)

        self._cleanup_timer = self.create_timer(3600.0, self._cleanup_old_bags)
        self._cleanup_old_bags()

        self.get_logger().info(f'Data logger ready (base_dir={base_dir})')

    def _on_msg(self, topic, msg):
        now_ns = self.get_clock().now().nanoseconds
        self.writer.write(topic, msg, now_ns)
        self._latest[topic] = msg

        if topic == '/fusion/diagnosis':
            self._handle_fusion(msg)

    def _handle_fusion(self, msg: FusionResult):
        if msg.alert_level != ALERT_CRITICAL:
            return
        key = f'{msg.header.stamp.sec}_{msg.header.stamp.nanosec}'
        if key in self._critical_keys:
            return
        self._critical_keys.add(key)

        ts = time.strftime('%Y%m%d_%H%M%S')
        target_dir = os.path.join('records', 'critical', ts)
        metadata = {}
        if self.record_metadata:
            metadata = self._build_metadata(msg)
        self.writer.snapshot_critical(target_dir, metadata)
        self.get_logger().info(
            f'CRITICAL snapshot saved to {target_dir}')

    def _build_metadata(self, fusion_msg):
        now = self.get_clock().now().to_msg()
        return {
            'saved_at': {
                'sec': now.sec,
                'nanosec': now.nanosec,
            },
            'trigger': {
                'topic': '/fusion/diagnosis',
                'risk_score': float(fusion_msg.risk_score),
                'alert_level': int(fusion_msg.alert_level),
                'mode': str(fusion_msg.mode),
            },
            'context': self._latest_context(),
        }

    def _latest_context(self):
        ctx = {}
        env = self._latest.get('/sensor/environment_mobile')
        if env is not None:
            ctx['environment'] = {
                'air_temp': float(env.air_temp),
                'air_humidity': float(env.air_humidity),
                'air_co2': float(env.air_co2),
                'soil_temp': float(env.soil_temp),
                'soil_humidity': float(env.soil_humidity),
                'leaf_wetness': float(env.leaf_wetness),
                'data_source': str(env.data_source),
            }
        advisory = self._latest.get('/advisory/action')
        if advisory is not None:
            ctx['advisory'] = {
                'action_type': str(advisory.action_type),
                'priority': str(advisory.priority),
                'description': str(advisory.description),
            }
        forecast = self._latest.get('/forecast/alert')
        if forecast is not None:
            ctx['forecast'] = {
                'active': bool(forecast.active),
                'alert_type': str(forecast.alert_type),
                'probability': float(forecast.probability),
            }
        return ctx

    def _cleanup_old_bags(self):
        if not os.path.exists(self.writer.base_dir):
            return
        cutoff = time.time() - (self.retention_days * 86400)
        for name in os.listdir(self.writer.base_dir):
            path = os.path.join(self.writer.base_dir, name)
            if not os.path.isdir(path):
                continue
            try:
                mtime = os.path.getmtime(path)
            except OSError:
                continue
            if mtime < cutoff:
                try:
                    shutil.rmtree(path)
                    self.get_logger().info(f'Removed old bag dir: {path}')
                except Exception as e:
                    self.get_logger().warn(f'Failed to remove {path}: {e}')

    def destroy_node(self):
        self.writer.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = DataLoggerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
```

注意：文件顶部需要导入 `shutil`。

- [ ] **Step 4：运行 data_logger 测试**

```bash
cd src/sentry_data_logger
python -m pytest tests/test_data_logger_node.py -v
```

Expected: PASS

---

## Task 6：在 `sentry_v2.launch.py` 中注册三个节点

**Files:**
- Modify: `src/sentry_bringup/launch/sentry_v2.launch.py`

- [ ] **Step 1：在 launch 文件末尾追加三个 Node 声明**

在 `sentry_v2.launch.py` 的 `return LaunchDescription([...])` 内部、`web_remote_node` 之后添加：

```python
        # Phase 2: forecast + advisory + data logger
        Node(
            package='sentry_forecast',
            executable='forecast_node',
            name='forecast_node',
            parameters=[{
                'crop_type': LaunchConfiguration('crop_type'),
                'crop_profiles_path': crop_profiles_path,
                'forecast_params_path': os.path.join(config_dir, 'forecast_params.yaml'),
                'mobile_stale_sec': 2.0,
                'fusion_stale_sec': 30.0,
            }],
            output='screen',
        ),
        Node(
            package='sentry_advisory',
            executable='advisory_node',
            name='advisory_node',
            parameters=[{
                'crop_type': LaunchConfiguration('crop_type'),
                'advisory_rules_path': os.path.join(config_dir, 'advisory_rules.yaml'),
                'fusion_stale_sec': 30.0,
            }],
            output='screen',
        ),
        Node(
            package='sentry_data_logger',
            executable='data_logger_node',
            name='data_logger_node',
            parameters=[os.path.join(config_dir, 'data_logger_params.yaml')],
            output='screen',
        ),
```

- [ ] **Step 2：检查 launch 文件语法**

```bash
python -m py_compile src/sentry_bringup/launch/sentry_v2.launch.py
```

Expected: 无输出（编译成功）

---

## Task 7：编译并运行 colcon test

- [ ] **Step 1：安装新增依赖（如有需要）**

```bash
rosdep install --from-paths src --ignore-src -r -y
```

- [ ] **Step 2：编译三个新包**

```bash
cd E:/smart_agri_sentry
colcon build --packages-select sentry_forecast sentry_advisory sentry_data_logger --symlink-install
```

Expected: 三个包均 `Finished` 成功。

- [ ] **Step 3：运行测试**

```bash
colcon test --packages-select sentry_forecast sentry_advisory sentry_data_logger
```

Expected: 所有测试 `passed`。

- [ ] **Step 4：查看测试结果**

```bash
colcon test-result --verbose
```

Expected: 无失败用例。

---

## Task 8：最终审查与提交

- [ ] **Step 1：运行 lint 检查**

```bash
cd src/sentry_forecast && python -m flake8 sentry_forecast tests --max-line-length=100
cd src/sentry_advisory && python -m flake8 sentry_advisory tests --max-line-length=100
cd src/sentry_data_logger && python -m flake8 sentry_data_logger tests --max-line-length=100
```

Expected: 无严重风格错误。

- [ ] **Step 2：更新根目录 `PLAN.md` 为全部完成状态**

```markdown
# Phase 2 节点实现追踪

- [x] Task 1：创建三个 ROS2 Python 包骨架
- [x] Task 2：添加 Phase 2 配置文件
- [x] Task 3：实现并测试 `forecast_node`
- [x] Task 4：实现并测试 `advisory_node`
- [x] Task 5：实现并测试 `data_logger_node`
- [x] Task 6：在 `sentry_v2.launch.py` 中注册三个节点
- [x] Task 7：编译并运行 colcon test
- [x] Task 8：最终审查与提交
```

- [ ] **Step 3：提交（仅当用户明确同意提交时）**

```bash
git add src/sentry_forecast src/sentry_advisory src/sentry_data_logger config docs/superpowers/specs/2026-06-12-phase2-nodes-design.md docs/superpowers/plans/2026-06-12-phase2-nodes.md src/sentry_bringup/launch/sentry_v2.launch.py PLAN.md
git status
git commit -m "feat(phase2): add forecast, advisory and data_logger nodes

- Add sentry_forecast with linear trend extrapolation
- Add sentry_advisory with YAML rule engine
- Add sentry_data_logger with rosbag2_py recording and CRITICAL snapshots
- Register nodes in sentry_v2.launch.py"
```

---

## Self-Review Checklist

- [x] **Spec coverage**：设计文档中 forecast / advisory / data_logger / 配置 / 测试 / launch 均有对应任务。
- [x] **Placeholder scan**：无 TBD、TODO、"implement later" 等占位。
- [x] **Type consistency**：`FusionResult.alert_level` 使用整型（0-3），与设计文档一致；`ForecastAlert.hours_ahead` 为整型；`AdvisoryAction.priority` 为字符串。
- [x] **文件路径**：所有 create/modify 路径均为相对仓库根目录的精确路径。
- [x] **测试命令**：每个任务都给出明确的 `pytest` / `colcon` 命令与期望输出。

---

## 执行方式

Plan complete and saved to `docs/superpowers/plans/2026-06-12-phase2-nodes.md`.

两个执行选项：

1. **Subagent-Driven（推荐）**：每个 Task 派一个独立 subagent 执行，我在每轮后 review 结果，适合复杂/多文件改动。
2. **Inline Execution**：在当前会话中使用 `superpowers:executing-plans` 批量执行，中间设 checkpoint 供你确认。

你想用哪种？
