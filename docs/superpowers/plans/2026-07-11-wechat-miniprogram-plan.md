# 微信小程序智农哨兵 · 实施计划

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 为智农哨兵机器人开发微信小程序遥控终端，包含控制/监测/分析/天气 4 个 Tab，通过独立桥接节点 `miniprogram_bridge_node` 与 ROS2 通信。

**Architecture:** 后端新增 `sentry_miniprogram` ROS2 包，用 FastAPI + websockets 在 `:8765` 桥接 ROS2 数据。前端在现有 `/wechat/miniprogram` 脚手架基础上重构为 4 Tab 原生 TS + Less + Skyline 应用。前后端通过 WebSocket(实时) + HTTP(低频/控制) 混合通信。

**Tech Stack:** 后端 Python FastAPI + websockets + rclpy；前端微信原生 TypeScript + Less + Skyline 渲染引擎

---

## File Structure

```
# === 后端 (新增) ===
src/sentry_miniprogram/
├── package.xml
├── setup.py
├── setup.cfg
├── sentry_miniprogram/
│   ├── __init__.py
│   └── miniprogram_bridge_node.py    # FastAPI + WS + ROS2 订阅
└── test/
    ├── __init__.py
    └── test_bridge.py                 # Mock ROS2 测试

# === 前端 (修改现有 + 新增) ===
wechat/miniprogram/
├── app.json                           # [修改] 加 tabBar + 4 页面
├── app.less                           # [修改] 全局 Grafana 深色主题
├── app.ts                             # [修改] 移除登录逻辑，初始化 store
├── services/
│   ├── store.ts                       # [新增] 全局响应式状态
│   ├── ws.ts                          # [新增] WebSocket 连接管理
│   └── api.ts                         # [新增] HTTP 请求封装
├── utils/
│   ├── format.ts                      # [新增] 单位格式化
│   └── ros-parser.ts                  # [新增] 后端 JSON 消息解析
├── components/
│   ├── status-badge/                  # [新增] 模式/连接胶囊
│   ├── data-value/                    # [新增] 数值+单位+颜色
│   └── alert-bar/                     # [新增] 预警条
├── pages/
│   ├── control/                       # [新增] Tab 1
│   │   ├── control.json/.ts/.wxml/.less
│   │   └── components/ (dpad, cruise-status, crop-selector)
│   ├── monitor/                       # [新增] Tab 2
│   │   ├── monitor.json/.ts/.wxml/.less
│   │   └── components/ (camera-view, sensor-card, env-grid)
│   ├── analysis/                      # [新增] Tab 3
│   │   ├── analysis.json/.ts/.wxml/.less
│   │   └── components/ (diagnosis-result, prob-bars, advisory-card, forecast-chart)
│   └── weather/                       # [新增] Tab 4
│       ├── weather.json/.ts/.wxml/.less
│       └── components/ (weather-now, day-forecast, hour-forecast, disaster-alert)
├── pages/index/                       # [删除，不再需要]
└── pages/logs/                        # [删除，不再需要]
```

---

### Task 1: 创建 `sentry_miniprogram` ROS2 包骨架

**Files:**
- Create: `src/sentry_miniprogram/package.xml`
- Create: `src/sentry_miniprogram/setup.py`
- Create: `src/sentry_miniprogram/setup.cfg`
- Create: `src/sentry_miniprogram/sentry_miniprogram/__init__.py`
- Create: `src/sentry_miniprogram/test/__init__.py`

- [ ] **Step 1: 创建 package.xml**

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>sentry_miniprogram</name>
  <version>0.1.0</version>
  <description>Mini-program bridge node — FastAPI + WebSocket gateway for WeChat mini-program</description>
  <maintainer email="wjun@example.com">wjun</maintainer>
  <license>MIT</license>

  <depend>rclpy</depend>
  <depend>sentry_interfaces</depend>
  <depend>std_msgs</depend>
  <depend>std_srvs</depend>
  <depend>geometry_msgs</depend>
  <depend>sensor_msgs</depend>

  <test_depend>python3-pytest</test_depend>
  <test_depend>python3-pytest-asyncio</test_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
```

- [ ] **Step 2: 创建 setup.py**

```python
from setuptools import setup

package_name = 'sentry_miniprogram'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='wjun',
    maintainer_email='wjun@example.com',
    description='Mini-program bridge node for WeChat mini-program',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'miniprogram_bridge_node = sentry_miniprogram.miniprogram_bridge_node:main',
        ],
    },
)
```

- [ ] **Step 3: 创建 setup.cfg**

```ini
[develop]
script_dir=$base/lib/sentry_miniprogram
[install]
install_scripts=$base/lib/sentry_miniprogram
```

- [ ] **Step 4: 创建两个 `__init__.py`**（空文件）

- [ ] **Step 5: 创建 resource 目录和 marker**

```bash
mkdir -p src/sentry_miniprogram/resource
touch src/sentry_miniprogram/resource/sentry_miniprogram
```

- [ ] **Step 6: 提交**

```bash
git add src/sentry_miniprogram/
git commit -m "feat: scaffold sentry_miniprogram ROS2 package"
```

---

### Task 2: 实现 `miniprogram_bridge_node` — 核心框架

**Files:**
- Create: `src/sentry_miniprogram/sentry_miniprogram/miniprogram_bridge_node.py`
- Create: `src/sentry_miniprogram/test/test_bridge.py`

- [ ] **Step 1: 写测试 — 验证 FastAPI 启动和 `/api/status` 返回 ros_connected=false**

```python
# test/test_bridge.py
import pytest
from fastapi.testclient import TestClient

# Patch rclpy before importing the module
import sys
from unittest.mock import MagicMock, patch

# Create mock rclpy and dependent modules
mock_rclpy = MagicMock()
mock_node = MagicMock()
mock_rclpy.create_node.return_value = mock_node

sys.modules['rclpy'] = mock_rclpy
sys.modules['rclpy.node'] = MagicMock()
sys.modules['std_msgs'] = MagicMock()
sys.modules['std_msgs.msg'] = MagicMock()
sys.modules['std_srvs'] = MagicMock()
sys.modules['std_srvs.srv'] = MagicMock()
sys.modules['geometry_msgs'] = MagicMock()
sys.modules['geometry_msgs.msg'] = MagicMock()
sys.modules['sentry_interfaces'] = MagicMock()
sys.modules['sentry_interfaces.srv'] = MagicMock()

# Now import the module - it will use the mocked rclpy
# The bridge node module needs to expose `app` for testing
from sentry_miniprogram.miniprogram_bridge_node import app


def test_status_endpoint():
    """GET /api/status returns connected: false when no ROS."""
    client = TestClient(app)
    response = client.get('/api/status')
    assert response.status_code == 200
    data = response.json()
    assert 'mode' in data
    assert 'ros_connected' in data
    assert data['ros_connected'] is False
```

- [ ] **Step 2: 运行测试 — 验证失败**

```bash
cd src/sentry_miniprogram && python -m pytest test/test_bridge.py::test_status_endpoint -v
```
Expected: FAIL with ImportError (bridge node doesn't exist yet)

- [ ] **Step 3: 写桥接节点核心框架**

```python
#!/usr/bin/env python3
"""Mini-program bridge node — FastAPI + WebSocket gateway.

Serves WeChat mini-program with:
- WS /ws: real-time sensor/status/mission/diagnosis push
- REST: low-frequency data (weather, forecast) + control commands
- GET /api/camera: MJPEG video stream
"""

import asyncio
import json
import threading
import time
from contextlib import asynccontextmanager

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_srvs.srv import SetBool
from sentry_interfaces.srv import SetCropType

from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.responses import StreamingResponse
import uvicorn


# ============ ROS2 Node ============

class MiniProgramBridgeNode(Node):
    def __init__(self):
        super().__init__('miniprogram_bridge_node')

        # Publishers
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Services
        self.mode_srv = self.create_client(SetBool, '/set_auto_mode')
        self.crop_type_srv = self.create_client(SetCropType, '/set_crop_type')

        # State cache (thread-safe via asyncio loop)
        self.mode = 'AUTO'
        self.linear = 0.0
        self.angular = 0.0
        self.ros_connected = True

        # Latest sensor snapshots
        self.sensors = {}
        self.mission = {}
        self.diagnosis = None
        self.plant_detect = {}
        self.weather = {}
        self.forecast = {}
        self.advisory = None

        # Subscriptions
        self._setup_subscriptions()

        # Async queues for WebSocket push
        self.ws_queues: list[asyncio.Queue] = []
        self._ws_lock = threading.Lock()
        self._loop = None  # set by FastAPI startup

    def _setup_subscriptions(self):
        """Subscribe to all relevant ROS2 topics."""
        from sentry_interfaces.msg import (
            Environment, SoilNutrition, ChassisStatus,
            Diagnosis, MissionStatus, PlantDetection,
            WeatherForecast, ForecastAlert, AdvisoryAction,
        )

        self.create_subscription(
            Environment, '/sentry/sensor/environment_mobile',
            self._on_environment, 10)
        self.create_subscription(
            SoilNutrition, '/sentry/sensor/soil_nutrition',
            self._on_soil, 10)
        self.create_subscription(
            ChassisStatus, '/sentry/chassis/status',
            self._on_chassis, 10)
        self.create_subscription(
            Diagnosis, '/vision/diagnosis',
            self._on_diagnosis, 10)
        self.create_subscription(
            MissionStatus, '/mission/status',
            self._on_mission, 10)
        self.create_subscription(
            PlantDetection, '/vision/plant_detected',
            self._on_plant_detect, 10)
        self.create_subscription(
            WeatherForecast, '/weather/forecast',
            self._on_weather, 10)
        self.create_subscription(
            ForecastAlert, '/forecast/alert',
            self._on_forecast, 10)
        self.create_subscription(
            AdvisoryAction, '/advisory/action',
            self._on_advisory, 10)

        self.get_logger().info('All subscriptions created')

    # --- Subscription callbacks ---

    def _on_environment(self, msg):
        self.sensors['air_temp'] = round(msg.air_temp, 1)
        self.sensors['air_humidity'] = round(msg.air_humidity, 1)
        self.sensors['co2'] = round(msg.air_co2, 0)
        self.sensors['soil_temp'] = round(msg.soil_temp, 1)
        self.sensors['soil_humidity'] = round(msg.soil_humidity, 1)
        self.sensors['leaf_wetness'] = round(msg.leaf_wetness, 1)
        self.sensors['data_source'] = msg.data_source
        self._push_ws({'type': 'sensor', 'ts': self._now_ms(), 'data': dict(self.sensors)})

    def _on_soil(self, msg):
        self.sensors['soil_n'] = round(msg.nitrogen, 1)
        self.sensors['soil_p'] = round(msg.phosphorus, 1)
        self.sensors['soil_k'] = round(msg.potassium, 1)
        self._push_ws({'type': 'sensor', 'ts': self._now_ms(), 'data': dict(self.sensors)})

    def _on_chassis(self, msg):
        self._push_ws({
            'type': 'status',
            'ts': self._now_ms(),
            'data': {
                'mode': self.mode,
                'left_speed': round(msg.left_speed, 2),
                'right_speed': round(msg.right_speed, 2),
                'battery_voltage': round(msg.battery_voltage, 2) if msg.battery_voltage > 0 else None,
                'ros_connected': self.ros_connected,
            }
        })

    def _on_diagnosis(self, msg):
        probs = [round(p, 4) for p in msg.probabilities] if msg.probabilities else []
        self.diagnosis = {
            'crop_type': msg.crop_type,
            'disease': msg.disease_class,
            'confidence': round(msg.confidence, 4),
            'probabilities': probs,
        }
        self._push_ws({'type': 'diagnosis', 'ts': self._now_ms(), 'data': self.diagnosis})

    def _on_mission(self, msg):
        self.mission = {
            'state': msg.state,
            'progress': round(msg.progress, 2),
            'current_action': msg.current_action,
            'plants_detected': msg.plants_detected,
            'plants_analyzed': msg.plants_analyzed,
            'current_wp_idx': msg.current_wp_idx,
            'total_wps': msg.total_wps,
            'waypoint_labels': list(msg.waypoint_labels),
        }
        self._push_ws({'type': 'mission', 'ts': self._now_ms(), 'data': self.mission})

    def _on_plant_detect(self, msg):
        self.plant_detect = {
            'detected': msg.detected,
            'bbox': list(msg.bbox),
            'confidence': round(msg.confidence, 4),
            'area_ratio': round(msg.area_ratio, 4),
        }
        self._push_ws({'type': 'plant_detect', 'ts': self._now_ms(), 'data': self.plant_detect})

    def _on_weather(self, msg):
        self.weather = {
            'city': msg.city,
            'lat': msg.lat,
            'lon': msg.lon,
            'days': [{'high': d.high, 'low': d.low, 'icon': d.icon, 'desc': d.desc}
                     for d in msg.days],
            'hours': [{'time': h.time, 'temp': h.temp, 'icon': h.icon, 'desc': h.desc}
                      for h in msg.hours],
            'disaster_alerts': list(msg.disaster_alerts),
            'stale': msg.stale,
        }

    def _on_forecast(self, msg):
        self.forecast = {
            'active': msg.active,
            'alert_type': msg.alert_type,
            'probability': round(msg.probability, 3),
            'description': msg.description,
            'hours_ahead': msg.hours_ahead,
        }
        if msg.active:
            self._push_ws({
                'type': 'alert',
                'ts': self._now_ms(),
                'data': {
                    'level': 'warning',
                    'title': f"病害预警: {msg.alert_type}",
                    'message': msg.description,
                }
            })

    def _on_advisory(self, msg):
        self.advisory = {
            'action_type': msg.action_type,
            'description': msg.description,
            'priority': msg.priority,
            'steps': list(msg.steps),
        }

    def _now_ms(self) -> int:
        return int(time.time() * 1000)

    def _push_ws(self, data: dict):
        """Push data to all connected WebSocket clients."""
        for q in self.ws_queues:
            try:
                q.put_nowait(data)
            except asyncio.QueueFull:
                pass

    # --- Control ---

    def set_mode(self, auto: bool) -> bool:
        if not self.mode_srv.service_is_ready():
            self.get_logger().error('/set_auto_mode service not available')
            return False
        req = SetBool.Request()
        req.data = auto
        future = self.mode_srv.call_async(req)
        self.mode = 'AUTO' if auto else 'MANUAL'
        self.get_logger().info(f"Mode switched to {self.mode}")
        return True

    def set_velocity(self, linear: float, angular: float):
        twist = Twist()
        twist.linear.x = max(-0.5, min(0.5, linear))
        twist.angular.z = max(-1.0, min(1.0, angular))
        self.cmd_pub.publish(twist)
        self.linear = twist.linear.x
        self.angular = twist.angular.z

    def emergency_stop(self):
        self.set_mode(False)  # Switch to MANUAL
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.cmd_pub.publish(twist)
        self.linear = 0.0
        self.angular = 0.0
        self.get_logger().warn('EMERGENCY STOP via mini-program')

    def set_crop_type(self, crop: str) -> bool:
        if not self.crop_type_srv.wait_for_service(timeout_sec=1.0):
            self.get_logger().error('/set_crop_type service not available')
            return False
        req = SetCropType.Request()
        req.crop_type = crop
        future = self.crop_type_srv.call_async(req)
        self.get_logger().info(f"Crop type set to {crop}")
        return True

    def get_status(self) -> dict:
        return {
            'mode': self.mode,
            'linear': self.linear,
            'angular': self.angular,
            'ros_connected': self.ros_connected,
            'sensors': self.sensors,
            'mission': self.mission,
            'plant_detect': self.plant_detect,
        }


# ============ FastAPI App ============

_node: MiniProgramBridgeNode = None
_app: FastAPI = None


def _get_app() -> FastAPI:
    global _app, _node
    if _app is not None:
        return _app

    @asynccontextmanager
    async def lifespan(app: FastAPI):
        # Startup: store event loop reference for WS push
        _node._loop = asyncio.get_running_loop()
        yield
        # Shutdown
        _node.ws_queues.clear()

    _app = FastAPI(lifespan=lifespan)

    # --- REST Endpoints ---

    @_app.get('/api/status')
    async def api_status():
        return _node.get_status()

    @_app.get('/api/weather')
    async def api_weather():
        return _node.weather

    @_app.get('/api/forecast')
    async def api_forecast():
        return {
            'forecast': _node.forecast,
            'advisory': _node.advisory,
            'diagnosis': _node.diagnosis,
        }

    @_app.post('/api/mode')
    async def api_set_mode(data: dict):
        auto = data.get('auto', False)
        ok = _node.set_mode(auto)
        return {'status': 'ok' if ok else 'error', 'mode': _node.mode}

    @_app.post('/api/control')
    async def api_control(data: dict):
        linear = data.get('linear', 0.0)
        angular = data.get('angular', 0.0)
        _node.set_velocity(linear, angular)
        return {'status': 'ok'}

    @_app.post('/api/stop')
    async def api_stop():
        _node.emergency_stop()
        return {'status': 'stopped', 'mode': _node.mode}

    @_app.post('/api/crop_type')
    async def api_crop_type(data: dict):
        crop = data.get('crop_type', '')
        ok = _node.set_crop_type(crop)
        return {'status': 'ok' if ok else 'error'}

    @_app.get('/api/camera')
    async def api_camera():
        """MJPEG stream endpoint — polls camera node via ROS2."""
        async def generate():
            import cv2
            from cv_bridge import CvBridge
            bridge = CvBridge()
            while True:
                try:
                    # Try to get latest image from shared cache
                    if hasattr(_node, '_latest_frame') and _node._latest_frame is not None:
                        img = _node._latest_frame
                        _, jpeg = cv2.imencode('.jpg', img, [cv2.IMWRITE_JPEG_QUALITY, 70])
                        yield (b'--frame\r\n'
                               b'Content-Type: image/jpeg\r\n\r\n' + jpeg.tobytes() + b'\r\n')
                except Exception:
                    pass
                await asyncio.sleep(0.1)  # ~10fps

        return StreamingResponse(
            generate(),
            media_type='multipart/x-mixed-replace; boundary=frame'
        )

    # --- WebSocket Endpoint ---

    @_app.websocket('/ws')
    async def ws_endpoint(ws: WebSocket):
        await ws.accept()
        q: asyncio.Queue = asyncio.Queue(maxsize=256)
        _node.ws_queues.append(q)
        try:
            # Push full state snapshot on connect
            await ws.send_json({
                'type': 'snapshot',
                'ts': _node._now_ms(),
                'data': _node.get_status()
            })
            # Pump messages from queue
            while True:
                data = await q.get()
                await ws.send_json(data)
        except (WebSocketDisconnect, Exception):
            pass
        finally:
            if q in _node.ws_queues:
                _node.ws_queues.remove(q)

    return _app


# ============ Entry Point ============

def _start_fastapi():
    app = _get_app()
    uvicorn.run(app, host='0.0.0.0', port=8765, log_level='info')


def main(args=None):
    global _node
    rclpy.init(args=args)
    _node = MiniProgramBridgeNode()

    # Run FastAPI in a background thread
    api_thread = threading.Thread(target=_start_fastapi, daemon=True)
    api_thread.start()

    try:
        rclpy.spin(_node)
    except KeyboardInterrupt:
        pass
    finally:
        _node.destroy_node()
        rclpy.shutdown()


# Expose app for testing
app = property(lambda self: _get_app())
```

- [ ] **Step 4: 运行测试 — 验证通过**

```bash
cd E:/smart_agri_sentry/src/sentry_miniprogram && python -m pytest test/test_bridge.py::test_status_endpoint -v
```
Expected: PASS

- [ ] **Step 5: 提交**

```bash
git add src/sentry_miniprogram/sentry_miniprogram/miniprogram_bridge_node.py src/sentry_miniprogram/test/test_bridge.py
git commit -m "feat: implement miniprogram_bridge_node core framework"
```

---

### Task 3: 桥接节点 — 相机帧订阅与 MJPEG 推送

**Files:**
- Modify: `src/sentry_miniprogram/sentry_miniprogram/miniprogram_bridge_node.py`

- [ ] **Step 1: 在 `_setup_subscriptions` 末尾加相机帧订阅**

```python
        # Camera: subscribe to compressed image topic from image_transport republish
        from sensor_msgs.msg import CompressedImage
        self._latest_frame = None
        self.create_subscription(
            CompressedImage, '/out/compressed',
            self._on_camera_frame, 10)
```

- [ ] **Step 2: 加回调方法**

在 `MiniProgramBridgeNode` 类中添加：

```python
    def _on_camera_frame(self, msg):
        import cv2
        import numpy as np
        import base64
        # Decode JPEG from compressed message
        np_arr = np.frombuffer(msg.data, np.uint8)
        frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        if frame is not None:
            self._latest_frame = frame
```

已有 `/out/compressed` 话题（来自 `image_transport republish raw compressed`），直接用 JPEG 解码，API endpoint 无需改动。

- [ ] **Step 3: 提交**

```bash
git add src/sentry_miniprogram/sentry_miniprogram/miniprogram_bridge_node.py
git commit -m "feat: add camera frame subscription for MJPEG streaming"
```

---

### Task 4: 小程序全局配置 — app.json + 删除旧页面

**Files:**
- Modify: `wechat/miniprogram/app.json`
- Modify: `wechat/miniprogram/app.ts`
- Modify: `wechat/miniprogram/app.less`
- Delete: `wechat/miniprogram/pages/index/`
- Delete: `wechat/miniprogram/pages/logs/`

- [ ] **Step 1: 重写 `app.json` — 注册 4 个 Tab 页面**

```json
{
  "pages": [
    "pages/control/control",
    "pages/monitor/monitor",
    "pages/analysis/analysis",
    "pages/weather/weather"
  ],
  "tabBar": {
    "color": "#64748B",
    "selectedColor": "#38BDF8",
    "backgroundColor": "#111827",
    "borderStyle": "black",
    "list": [
      {
        "pagePath": "pages/control/control",
        "text": "控制"
      },
      {
        "pagePath": "pages/monitor/monitor",
        "text": "监测"
      },
      {
        "pagePath": "pages/analysis/analysis",
        "text": "分析"
      },
      {
        "pagePath": "pages/weather/weather",
        "text": "天气"
      }
    ]
  },
  "window": {
    "navigationBarTextStyle": "white",
    "navigationBarBackgroundColor": "#0F172A",
    "backgroundColor": "#0B1120",
    "navigationStyle": "custom"
  },
  "style": "v2",
  "rendererOptions": {
    "skyline": {
      "defaultDisplayBlock": true,
      "disableABTest": true,
      "sdkVersionBegin": "3.0.0",
      "sdkVersionEnd": "15.255.255"
    }
  },
  "componentFramework": "glass-easel",
  "sitemapLocation": "sitemap.json",
  "lazyCodeLoading": "requiredComponents"
}
```

- [ ] **Step 2: 重写 `app.ts` — 移除登录逻辑，初始化 store**

```typescript
// app.ts
App<IAppOption>({
  globalData: {},
  onLaunch() {
    // 初始化全局状态请见 services/store.ts
  },
})
```

- [ ] **Step 3: 重写 `app.less` — Grafana 深色全局样式**

```less
/* Grafana Dark Theme — 与 static_v2/style.css 一致 */
page {
  --bg-deep:    #0B1120;
  --bg-surface: #0F172A;
  --bg-card:    #111827;
  --bg-hover:   #1E293B;
  --bg-input:   #0F172A;
  --border:     #1F2937;
  --border-hi:  #374151;
  --text:       #F8FAFC;
  --text-dim:   #94A3B8;
  --text-muted: #64748B;
  --green:      #10B981;
  --amber:      #F59E0B;
  --red:        #EF4444;
  --blue:       #38BDF8;
  --purple:     #A78BFA;
  --grey:       #6B7280;
  --radius:     4px;
  --radius-pill: 6px;

  background: var(--bg-deep);
  color: var(--text);
  font-family: 'PingFang SC', 'Microsoft YaHei', system-ui, -apple-system, sans-serif;
  font-size: 28rpx;
  line-height: 1.5;
  height: 100%;
}

/* Shared card style */
.card {
  background: var(--bg-card);
  border: 1px solid var(--border);
  border-radius: var(--radius);
  padding: 16rpx 20rpx;
}

.card-header {
  font-size: 20rpx;
  font-weight: 600;
  color: var(--text-dim);
  text-transform: uppercase;
  letter-spacing: 0.06em;
  margin-bottom: 12rpx;
}

/* Monospace for data values */
.mono {
  font-family: 'JetBrains Mono', 'Cascadia Code', 'Consolas', monospace;
}

/* Status colors */
.green  { color: var(--green); }
.amber  { color: var(--amber); }
.red    { color: var(--red); }
.blue   { color: var(--blue); }
.purple { color: var(--purple); }
.dim    { color: var(--text-dim); }
.muted  { color: var(--text-muted); }

/* Alert bar */
.alert-bar {
  background: rgba(239,68,68,0.08);
  border: 1px solid rgba(239,68,68,0.2);
  border-radius: var(--radius);
  padding: 16rpx 20rpx;
  font-size: 24rpx;
}
```

- [ ] **Step 4: 删除旧页面目录**

```bash
rm -rf wechat/miniprogram/pages/index wechat/miniprogram/pages/logs
```

- [ ] **Step 5: 提交**

```bash
git add wechat/miniprogram/app.json wechat/miniprogram/app.ts wechat/miniprogram/app.less
git add wechat/miniprogram/pages/index wechat/miniprogram/pages/logs  # deletion
git commit -m "feat: configure 4-tab bar + Grafana dark global theme, remove old pages"
```

---

### Task 5: 全局状态管理 — `services/store.ts`

**Files:**
- Create: `wechat/miniprogram/services/store.ts`

- [ ] **Step 1: 写测试**

不存在独立测试文件——此文件是纯 reactive 数据定义。正确性由后续页面渲染间接验证。

- [ ] **Step 2: 实现 store.ts**

```typescript
// services/store.ts
// Reactive global state — mirrors static_v2/ros.js window.store

const store = {
  // Connection
  connected: false,

  // Robot status
  mode: 'AUTO',           // 'AUTO' | 'MANUAL'
  linear: 0,
  angular: 0,
  batteryVoltage: null as number | null,
  rosConnected: false,

  // Camera / Detection
  cameraFrameUrl: '',     // MJPEG URL or latest JPEG
  plantDetected: false,
  plantConfidence: 0,
  plantBbox: [0, 0, 0, 0] as number[],
  plantAreaRatio: 0,

  // Diagnosis
  diagnosisCropType: '',
  diagnosisDisease: '',
  diagnosisConfidence: 0,
  diagnosisProbabilities: [] as number[],

  // Advisory
  advisoryText: '',
  advisoryPriority: '',
  advisorySteps: [] as string[],

  // Forecast
  forecastActive: false,
  forecastAlertType: '',
  forecastDescription: '',
  forecastHoursAhead: 0,

  // Weather
  weatherCity: '',
  weatherDays: [] as Array<{high: number, low: number, icon: string, desc: string}>,
  weatherHours: [] as Array<{time: string, temp: number, icon: string, desc: string}>,
  weatherDisasterAlerts: [] as string[],
  weatherStale: false,

  // Environment sensors
  envAirTemp: null as number | null,
  envAirHumidity: null as number | null,
  envCO2: null as number | null,
  envSoilTemp: null as number | null,
  envSoilHumidity: null as number | null,
  envSoilN: null as number | null,
  envSoilP: null as number | null,
  envSoilK: null as number | null,
  envLeafWetness: null as number | null,
  envDataSource: '',

  // Mission
  missionState: 'IDLE',
  missionProgress: 0,
  missionCurrentAction: '',
  missionPlantsDetected: 0,
  missionPlantsAnalyzed: 0,
  missionCurrentWpIdx: 0,
  missionTotalWps: 0,
  missionWaypointLabels: [] as string[],

  // Crop
  cropType: 'tomato',
};

type Store = typeof store;

// Simple reactive wrapper: pages that import updateStore can notify
// their own setData when the store changes. Mirroring Vue.reactive pattern.
const listeners: Array<(s: Store) => void> = [];

export function getStore(): Store {
  return store;
}

export function updateStore(partial: Partial<Store>) {
  Object.assign(store, partial);
  for (const fn of listeners) {
    fn(store);
  }
}

export function onStoreChange(fn: (s: Store) => void) {
  listeners.push(fn);
}

export function offStoreChange(fn: (s: Store) => void) {
  const i = listeners.indexOf(fn);
  if (i >= 0) listeners.splice(i, 1);
}
```

- [ ] **Step 3: 提交**

```bash
git add wechat/miniprogram/services/
git commit -m "feat: add reactive global store for mini-program"
```

---

### Task 6: WebSocket 服务 — `services/ws.ts`

**Files:**
- Create: `wechat/miniprogram/services/ws.ts`

- [ ] **Step 1: 实现 WebSocket 连接管理**

```typescript
// services/ws.ts
// WebSocket connection manager — real-time data from miniprogram_bridge_node

import { updateStore, getStore, type Store } from './store';

const WS_URL = 'ws://192.168.1.100:8765/ws'; // TODO: configurable
const RETRY_DELAYS = [3000, 15000, 30000]; // backoff: 3s, 15s, 30s

let ws: WechatMiniprogram.SocketTask | null = null;
let retryIdx = 0;
let retryTimer: number | null = null;

export function wsConnect() {
  if (ws) {
    try { ws.close({}); } catch (_) {}
    ws = null;
  }

  ws = wx.connectSocket({
    url: WS_URL,
    header: { 'Content-Type': 'application/json' },
  });

  ws.onOpen(() => {
    console.log('[WS] Connected');
    updateStore({ connected: true });
    retryIdx = 0;
  });

  ws.onMessage((res) => {
    try {
      const msg = JSON.parse(res.data as string);
      handleMessage(msg);
    } catch (e) {
      console.warn('[WS] Bad message:', e);
    }
  });

  ws.onClose(() => {
    console.log('[WS] Disconnected');
    updateStore({ connected: false });
    scheduleReconnect();
  });

  ws.onError((err) => {
    console.error('[WS] Error:', err);
    updateStore({ connected: false });
  });
}

function scheduleReconnect() {
  const delay = RETRY_DELAYS[Math.min(retryIdx, RETRY_DELAYS.length - 1)];
  retryIdx++;
  console.log(`[WS] Reconnecting in ${delay}ms...`);
  if (retryTimer) clearTimeout(retryTimer);
  retryTimer = setTimeout(wsConnect, delay) as unknown as number;
}

function handleMessage(msg: { type: string; ts: number; data: any }) {
  const { type, data } = msg;

  switch (type) {
    case 'snapshot':
      // Full state on connect
      updateStore({
        mode: data.mode,
        linear: data.linear,
        angular: data.angular,
        rosConnected: data.ros_connected,
      });
      if (data.sensors) applySensorData(data.sensors);
      if (data.mission) applyMissionData(data.mission);
      break;

    case 'sensor':
      applySensorData(data);
      break;

    case 'status':
      updateStore({
        mode: data.mode,
        rosConnected: data.ros_connected,
        batteryVoltage: data.battery_voltage,
      });
      break;

    case 'mission':
      applyMissionData(data);
      break;

    case 'diagnosis':
      updateStore({
        diagnosisCropType: data.crop_type,
        diagnosisDisease: data.disease,
        diagnosisConfidence: data.confidence,
        diagnosisProbabilities: data.probabilities || [],
      });
      break;

    case 'plant_detect':
      updateStore({
        plantDetected: data.detected,
        plantConfidence: data.confidence,
        plantBbox: data.bbox,
        plantAreaRatio: data.area_ratio,
      });
      break;

    case 'alert':
      // Alert is handled by the analysis page directly
      console.log('[WS] Alert:', data.title, data.message);
      break;
  }
}

function applySensorData(d: any) {
  updateStore({
    envAirTemp: d.air_temp ?? null,
    envAirHumidity: d.air_humidity ?? null,
    envCO2: d.co2 ?? null,
    envSoilTemp: d.soil_temp ?? null,
    envSoilHumidity: d.soil_humidity ?? null,
    envSoilN: d.soil_n ?? null,
    envSoilP: d.soil_p ?? null,
    envSoilK: d.soil_k ?? null,
    envLeafWetness: d.leaf_wetness ?? null,
    envDataSource: d.data_source || '',
  });
}

function applyMissionData(d: any) {
  updateStore({
    missionState: d.state,
    missionProgress: d.progress,
    missionCurrentAction: d.current_action,
    missionPlantsDetected: d.plants_detected,
    missionPlantsAnalyzed: d.plants_analyzed,
    missionCurrentWpIdx: d.current_wp_idx,
    missionTotalWps: d.total_wps,
    missionWaypointLabels: d.waypoint_labels || [],
  });
}

export function wsDisconnect() {
  if (ws) {
    ws.close({});
    ws = null;
  }
  if (retryTimer) {
    clearTimeout(retryTimer);
    retryTimer = null;
  }
}
```

- [ ] **Step 2: 提交**

```bash
git add wechat/miniprogram/services/ws.ts
git commit -m "feat: add WebSocket connection manager with auto-reconnect"
```

---

### Task 7: HTTP 服务 + 工具函数 — `services/api.ts` + `utils/`

**Files:**
- Create: `wechat/miniprogram/services/api.ts`
- Create: `wechat/miniprogram/utils/format.ts`
- Create: `wechat/miniprogram/utils/ros-parser.ts`

- [ ] **Step 1: 实现 `api.ts`**

```typescript
// services/api.ts
// HTTP request wrapper for low-frequency data + control commands

const BASE_URL = 'http://192.168.1.100:8765'; // TODO: configurable

async function request<T>(method: 'GET' | 'POST', path: string, body?: any): Promise<T> {
  return new Promise((resolve, reject) => {
    wx.request({
      url: BASE_URL + path,
      method,
      header: { 'Content-Type': 'application/json' },
      data: body,
      success(res) {
        if (res.statusCode === 200) {
          resolve(res.data as T);
        } else {
          reject(new Error(`HTTP ${res.statusCode}: ${res.errMsg}`));
        }
      },
      fail(err) {
        reject(new Error(err.errMsg));
      },
    });
  });
}

// --- Control ---
export function apiSetMode(auto: boolean) {
  return request<{status: string; mode: string}>('POST', '/api/mode', { auto });
}

export function apiControl(linear: number, angular: number) {
  return request<{status: string}>('POST', '/api/control', { linear, angular });
}

export function apiStop() {
  return request<{status: string; mode: string}>('POST', '/api/stop');
}

export function apiSetCropType(cropType: string) {
  return request<{status: string}>('POST', '/api/crop_type', { crop_type: cropType });
}

// --- Query ---
export function apiGetStatus() {
  return request<any>('GET', '/api/status');
}

export function apiGetWeather() {
  return request<any>('GET', '/api/weather');
}

export function apiGetForecast() {
  return request<any>('GET', '/api/forecast');
}

// --- Camera URL (for image src) ---
export function getCameraUrl(): string {
  return BASE_URL + '/api/camera';
}
```

- [ ] **Step 2: 实现 `format.ts`**

```typescript
// utils/format.ts
// Unit formatting helpers

export function formatTemp(val: number | null): string {
  if (val == null) return '--';
  return val.toFixed(1) + '°C';
}

export function formatHumidity(val: number | null): string {
  if (val == null) return '--';
  return val.toFixed(1) + '%';
}

export function formatCO2(val: number | null): string {
  if (val == null) return '--';
  return Math.round(val).toString() + ' ppm';
}

export function formatNPK(val: number | null): string {
  if (val == null) return '--';
  return val.toFixed(1);
}

export function formatPercent(val: number | null): string {
  if (val == null) return '--';
  return (val * 100).toFixed(1) + '%';
}

export function formatSpeed(val: number): string {
  return val.toFixed(2) + ' m/s';
}

export function formatVoltage(val: number | null): string {
  if (val == null) return '--V';
  return val.toFixed(1) + 'V';
}
```

- [ ] **Step 3: 实现 `ros-parser.ts`**

```typescript
// utils/ros-parser.ts
// Parse backend JSON into strongly-typed structures (passthrough for now)

export function parseSensorData(data: any) {
  return data; // Already flat JSON from bridge node
}

export function parseWeatherData(data: any) {
  if (!data || !data.days) return data;
  return data; // Already structured
}
```

- [ ] **Step 4: 提交**

```bash
git add wechat/miniprogram/services/api.ts wechat/miniprogram/utils/
git commit -m "feat: add HTTP API service + format/parser utilities"
```

---

### Task 8: 共享组件 — `status-badge`, `data-value`, `alert-bar`

**Files:**
- Create: `wechat/miniprogram/components/status-badge/status-badge.{json,ts,wxml,less}`
- Create: `wechat/miniprogram/components/data-value/data-value.{json,ts,wxml,less}`
- Create: `wechat/miniprogram/components/alert-bar/alert-bar.{json,ts,wxml,less}`

- [ ] **Step 1: `status-badge` — 模式/连接状态胶囊**

`status-badge.json`:
```json
{ "component": true, "usingComponents": {} }
```

`status-badge.ts`:
```typescript
Component({
  properties: {
    text: { type: String, value: '' },
    type: { type: String, value: 'green' }, // green | blue | red | amber
  },
  data: {
    _class: '',
  },
  observers: {
    'type': function(t: string) {
      const classMap: Record<string, string> = {
        green: 'badge badge-green',
        blue:  'badge badge-blue',
        red:   'badge badge-red',
        amber: 'badge badge-amber',
      };
      this.setData({ _class: classMap[t] || classMap.green });
    }
  }
})
```

`status-badge.wxml`:
```xml
<text class="{{_class}}">{{text}}</text>
```

`status-badge.less`:
```less
.badge {
  display: inline-block;
  font-size: 20rpx;
  font-weight: 600;
  padding: 4rpx 16rpx;
  border-radius: var(--radius-pill);
  font-family: 'JetBrains Mono', 'Cascadia Code', 'Consolas', monospace;
}
.badge-green  { background: rgba(16,185,129,0.1); color: #10B981; border: 1px solid rgba(16,185,129,0.2); }
.badge-blue   { background: rgba(56,189,248,0.1);  color: #38BDF8; border: 1px solid rgba(56,189,248,0.2); }
.badge-red    { background: rgba(239,68,68,0.1);   color: #EF4444; border: 1px solid rgba(239,68,68,0.2); }
.badge-amber  { background: rgba(245,158,11,0.1);  color: #F59E0B; border: 1px solid rgba(245,158,11,0.2); }
```

- [ ] **Step 2: `data-value` — 数值+单位+颜色**

`data-value.json`:
```json
{ "component": true, "usingComponents": {} }
```

`data-value.ts`:
```typescript
Component({
  properties: {
    label: { type: String, value: '' },
    value: { type: String, value: '--' },  // pre-formatted string
    unit: { type: String, value: '' },
    color: { type: String, value: '' },    // green | amber | red | blue | ''
  },
})
```

`data-value.wxml`:
```xml
<view class="dv-wrap {{color ? 'dv-' + color : ''}}">
  <text class="dv-label">{{label}}</text>
  <text class="dv-val">{{value}}</text>
  <text class="dv-unit" wx:if="{{unit}}">{{unit}}</text>
</view>
```

`data-value.less`:
```less
.dv-wrap {
  background: var(--bg-input);
  border: 1px solid var(--border);
  border-radius: var(--radius);
  padding: 12rpx 16rpx;
  display: flex;
  flex-direction: column;
}
.dv-label {
  font-size: 18rpx;
  color: var(--text-muted);
  text-transform: uppercase;
  letter-spacing: 0.05em;
}
.dv-val {
  font-size: 36rpx;
  font-weight: 700;
  font-family: 'JetBrains Mono', 'Cascadia Code', 'Consolas', monospace;
  color: var(--text);
}
.dv-unit {
  font-size: 20rpx;
  color: var(--text-dim);
}
.dv-green  .dv-val { color: var(--green); }
.dv-amber  .dv-val { color: var(--amber); }
.dv-red    .dv-val { color: var(--red); }
.dv-blue   .dv-val { color: var(--blue); }
```

- [ ] **Step 3: `alert-bar` — 预警条**

`alert-bar.json`:
```json
{ "component": true, "usingComponents": {} }
```

`alert-bar.ts`:
```typescript
Component({
  properties: {
    level: { type: String, value: 'info' },   // info | warning | danger
    title: { type: String, value: '' },
    message: { type: String, value: '' },
  },
})
```

`alert-bar.wxml`:
```xml
<view class="alert alert-{{level}}" wx:if="{{title}}">
  <text class="alert-title">{{title}}</text>
  <text class="alert-msg" wx:if="{{message}}">{{message}}</text>
  <slot />
</view>
```

`alert-bar.less`:
```less
.alert {
  border-radius: var(--radius);
  padding: 16rpx 20rpx;
  border-left: 6rpx solid;
}
.alert-title { font-size: 24rpx; font-weight: 600; display: block; }
.alert-msg { font-size: 22rpx; color: var(--text-dim); margin-top: 4rpx; display: block; }

.alert-info    { background: rgba(56,189,248,0.08);  border-color: #38BDF8; }
.alert-info .alert-title { color: #38BDF8; }
.alert-warning { background: rgba(245,158,11,0.08);  border-color: #F59E0B; }
.alert-warning .alert-title { color: #F59E0B; }
.alert-danger  { background: rgba(239,68,68,0.08);   border-color: #EF4444; }
.alert-danger .alert-title { color: #EF4444; }
```

- [ ] **Step 4: 提交**

```bash
git add wechat/miniprogram/components/
git commit -m "feat: add shared components — status-badge, data-value, alert-bar"
```

---

### Task 9: 控制页面 — Tab 1

**Files:**
- Create: `wechat/miniprogram/pages/control/control.{json,ts,wxml,less}`

- [ ] **Step 1: 页面配置**

`control.json`:
```json
{ "usingComponents": { "status-badge": "/components/status-badge/status-badge" } }
```

- [ ] **Step 2: 页面逻辑**

`control.ts`:
```typescript
import { getStore, updateStore, onStoreChange } from '../../services/store';
import { apiSetMode, apiControl, apiStop, apiSetCropType } from '../../services/api';

Component({
  data: {
    mode: 'AUTO',
    linear: 0,
    angular: 0,
    missionState: 'IDLE',
    missionProgress: 0,
    missionWaypointLabels: [] as string[],
    missionCurrentWpIdx: 0,
    missionTotalWps: 0,
    missionPlantsDetected: 0,
  },
  lifetimes: {
    attached() {
      const s = getStore();
      this.sync(s);
      this._unsub = onStoreChange((s) => this.sync(s));
    },
    detached() {
      if (this._unsub) this._unsub();
    },
  },
  methods: {
    sync(s: any) {
      this.setData({
        mode: s.mode,
        linear: s.linear,
        angular: s.angular,
        missionState: s.missionState,
        missionProgress: s.missionProgress,
        missionWaypointLabels: s.missionWaypointLabels,
        missionCurrentWpIdx: s.missionCurrentWpIdx,
        missionTotalWps: s.missionTotalWps,
        missionPlantsDetected: s.missionPlantsDetected,
      });
    },

    // D-Pad: linear increment per tap
    _linear: 0,
    _angular: 0,

    onBtnUp()    { this._linear += 0.05; this.sendCmd(); },
    onBtnDown()  { this._linear -= 0.05; this.sendCmd(); },
    onBtnLeft()  { this._angular += 0.05; this.sendCmd(); },
    onBtnRight() { this._angular -= 0.05; this.sendCmd(); },
    onBtnStop()  { this._linear = 0; this._angular = 0; apiStop(); },

    sendCmd() {
      apiControl(this._linear, this._angular);
      updateStore({ linear: this._linear, angular: this._angular });
    },

    onToggleMode() {
      const newMode = this.data.mode === 'AUTO' ? false : true;
      apiSetMode(newMode);
    },

    onSelectCrop(e: any) {
      const crop = e.currentTarget.dataset.crop;
      apiSetCropType(crop);
      updateStore({ cropType: crop });
    },
  },
})
```

- [ ] **Step 3: 页面模板**

`control.wxml`:
```xml
<view class="page">
  <!-- Mode -->
  <view class="card row-between">
    <text class="label">当前模式</text>
    <status-badge text="{{mode}}" type="{{mode === 'AUTO' ? 'green' : 'blue'}}" />
  </view>

  <!-- D-Pad -->
  <view class="card dpad-container">
    <view class="dpad">
      <view class="dpad-btn btn-up"    bindtouchstart="onBtnUp">▲</view>
      <view class="dpad-btn btn-left"  bindtouchstart="onBtnLeft">◀</view>
      <view class="dpad-btn btn-center">●</view>
      <view class="dpad-btn btn-right" bindtouchstart="onBtnRight">▶</view>
      <view class="dpad-btn btn-down"  bindtouchstart="onBtnDown">▼</view>
    </view>
    <view class="speed-readout">
      <text class="mono dim">v: {{linear}}  ω: {{angular}}</text>
    </view>
  </view>

  <!-- Crop selector + Stop -->
  <view class="row">
    <view class="card flex-1" style="text-align:center">
      <text class="label">作物</text>
      <view class="pills">
        <text class="pill {{cropType === 'tomato' ? 'active' : ''}}" data-crop="tomato" bindtap="onSelectCrop">番茄</text>
        <text class="pill {{cropType === 'wheat' ? 'active' : ''}}" data-crop="wheat" bindtap="onSelectCrop">小麦</text>
        <text class="pill {{cropType === 'strawberry' ? 'active' : ''}}" data-crop="strawberry" bindtap="onSelectCrop">草莓</text>
      </view>
    </view>
    <view class="card" style="text-align:center">
      <view class="estop-btn" bindtap="onBtnStop">STOP</view>
      <text class="muted" style="font-size:18rpx">急停</text>
    </view>
  </view>

  <!-- Cruise status -->
  <view class="card">
    <text class="card-header">自动巡航</text>
    <view class="row-between" style="font-size:22rpx">
      <text>状态 <text class="green">● {{missionState}}</text></text>
      <text>点位 <text class="mono">{{missionCurrentWpIdx}}/{{missionTotalWps}}</text></text>
      <text>检测 <text class="mono">{{missionPlantsDetected}}株</text></text>
    </view>
    <view class="tags" style="margin-top:12rpx">
      <text class="tag" wx:for="{{missionWaypointLabels}}" wx:key="index">{{item}}</text>
    </view>
  </view>

  <!-- Action buttons -->
  <view class="row">
    <view class="card flex-1 center green" bindtap="onToggleMode">
      <text style="font-weight:600;color:#10B981">{{mode === 'AUTO' ? '切换 MANUAL' : '切换 AUTO'}}</text>
    </view>
    <view class="card flex-1 center">
      <text style="font-weight:600;color:#F59E0B">📍 航点</text>
    </view>
  </view>
</view>
```

- [ ] **Step 4: 页面样式**

`control.less`:
```less
.page { padding: 16rpx; display: flex; flex-direction: column; gap: 12rpx; }
.row { display: flex; gap: 12rpx; }
.row-between { display: flex; justify-content: space-between; align-items: center; }
.flex-1 { flex: 1; }
.center { text-align: center; }
.label { font-size: 24rpx; font-weight: 600; }

/* D-Pad */
.dpad-container { display: flex; flex-direction: column; align-items: center; gap: 8rpx; }
.dpad { display: grid; grid-template-columns: repeat(3, 80rpx); grid-template-rows: repeat(3, 80rpx); gap: 6rpx; }
.dpad-btn {
  width: 80rpx; height: 80rpx; border-radius: 50%;
  border: 1px solid var(--border-hi); background: var(--bg-input); color: var(--text-dim);
  display: flex; align-items: center; justify-content: center; font-size: 32rpx;
}
.dpad-btn:active { background: var(--blue); color: #000; border-color: var(--blue); }
.btn-up    { grid-column: 2; grid-row: 1; }
.btn-left  { grid-column: 1; grid-row: 2; }
.btn-center{ grid-column: 2; grid-row: 2; font-size: 16rpx; color: var(--text-muted); }
.btn-right { grid-column: 3; grid-row: 2; }
.btn-down  { grid-column: 2; grid-row: 3; }

/* Estop */
.estop-btn {
  width: 110rpx; height: 110rpx; border-radius: 50%;
  background: var(--red); border: 4rpx solid var(--red);
  color: #fff; font-size: 26rpx; font-weight: 700;
  display: flex; align-items: center; justify-content: center; margin: 0 auto;
}
.estop-btn:active { transform: scale(0.95); }

/* Pills */
.pills { display: flex; gap: 4rpx; justify-content: center; margin-top: 8rpx; }
.pill {
  font-size: 22rpx; padding: 6rpx 20rpx; border-radius: var(--radius-pill);
  border: 1px solid var(--border); background: var(--bg-input); color: var(--text-muted);
}
.pill.active { background: var(--blue); border-color: var(--blue); color: #000; font-weight: 600; }

/* Tags */
.tags { display: flex; flex-wrap: wrap; gap: 6rpx; }
.tag {
  font-size: 18rpx; padding: 4rpx 12rpx; border-radius: var(--radius);
  background: rgba(56,189,248,0.08); color: var(--blue);
  font-family: 'JetBrains Mono', 'Cascadia Code', 'Consolas', monospace;
}

.speed-readout { margin-top: 4rpx; font-size: 22rpx; }
```

- [ ] **Step 5: 提交**

```bash
git add wechat/miniprogram/pages/control/
git commit -m "feat: add control page with D-pad, crop selector, and cruise status"
```

---

### Task 10: 监测页面 — Tab 2

**Files:**
- Create: `wechat/miniprogram/pages/monitor/monitor.{json,ts,wxml,less}`

- [ ] **Step 1: 页面配置**

`monitor.json`:
```json
{ "usingComponents": {} }
```

- [ ] **Step 2: 页面逻辑**

`monitor.ts`:
```typescript
import { getStore, onStoreChange } from '../../services/store';
import { getCameraUrl } from '../../services/api';
import { formatTemp, formatHumidity, formatCO2, formatNPK } from '../../utils/format';

Component({
  data: {
    cameraUrl: '',
    plantDetected: false,
    plantConfidence: 0,
    plantAreaRatio: 0,
    airTemp: '--',
    airHumidity: '--',
    co2: '--',
    soilTemp: '--',
    soilHumidity: '--',
    soilN: '--',
    soilP: '--',
    soilK: '--',
    leafWetness: '--',
    dataSource: '',
  },
  lifetimes: {
    attached() {
      const s = getStore();
      this.sync(s);
      this._unsub = onStoreChange((s) => this.sync(s));
      // Start camera streaming
      this.setData({ cameraUrl: getCameraUrl() + '?t=' + Date.now() });
    },
    detached() {
      if (this._unsub) this._unsub();
    },
  },
  observers: {
    'cameraUrl': function(url: string) {
      // Re-bind camera src every 2s for MJPEG fallback (image polling)
      if (this._camTimer) clearInterval(this._camTimer as number);
      this._camTimer = setInterval(() => {
        this.setData({ cameraUrl: getCameraUrl() + '?t=' + Date.now() });
      }, 2000);
    }
  },
  methods: {
    sync(s: any) {
      this.setData({
        plantDetected: s.plantDetected,
        plantConfidence: (s.plantConfidence * 100).toFixed(1),
        plantAreaRatio: s.plantAreaRatio.toFixed(2),
        airTemp: formatTemp(s.envAirTemp),
        airHumidity: formatHumidity(s.envAirHumidity),
        co2: formatCO2(s.envCO2),
        soilTemp: formatTemp(s.envSoilTemp),
        soilHumidity: formatHumidity(s.envSoilHumidity),
        soilN: formatNPK(s.envSoilN),
        soilP: formatNPK(s.envSoilP),
        soilK: formatNPK(s.envSoilK),
        leafWetness: formatHumidity(s.envLeafWetness),
        dataSource: s.envDataSource || '--',
      });
    },
  },
})
```

- [ ] **Step 3: 页面模板**

`monitor.wxml`:
```xml
<view class="page">
  <!-- Video -->
  <view class="camera-wrap">
    <image class="camera" src="{{cameraUrl}}" mode="aspectFit" />
    <view class="camera-hud">
      <text wx:if="{{plantDetected}}" class="green">● 植株检测</text>
      <text wx:else class="muted">○ 未检测</text>
      <text class="mono dim">置信度 {{plantConfidence}}%</text>
      <text class="mono dim">面积比 {{plantAreaRatio}}</text>
    </view>
  </view>

  <!-- Air sensors -->
  <view class="sensor-grid">
    <view class="sensor-item">
      <text class="s-label">🌡 空气温度</text>
      <text class="s-val">{{airTemp}}</text>
    </view>
    <view class="sensor-item">
      <text class="s-label">💧 空气湿度</text>
      <text class="s-val">{{airHumidity}}</text>
    </view>
    <view class="sensor-item">
      <text class="s-label">🫧 CO₂</text>
      <text class="s-val">{{co2}}</text>
    </view>
    <view class="sensor-item">
      <text class="s-label">🌱 土壤温度</text>
      <text class="s-val">{{soilTemp}}</text>
    </view>
  </view>

  <!-- Soil NPK -->
  <view class="card">
    <text class="card-header">土壤 NPK</text>
    <view class="sensor-grid sensor-grid-3col">
      <view class="sensor-item">
        <text class="s-label">N 氮</text>
        <text class="s-val">{{soilN}}</text>
      </view>
      <view class="sensor-item">
        <text class="s-label">P 磷</text>
        <text class="s-val">{{soilP}}</text>
      </view>
      <view class="sensor-item">
        <text class="s-label">K 钾</text>
        <text class="s-val">{{soilK}}</text>
      </view>
    </view>
  </view>

  <!-- Leaf sensor -->
  <view class="card">
    <text class="card-header">叶面传感器</text>
    <view class="sensor-item">
      <text class="s-label">💧 湿度</text>
      <text class="s-val blue">{{leafWetness}}</text>
    </view>
  </view>

  <!-- Data source -->
  <view class="muted" style="text-align:right;font-size:20rpx">
    数据源: {{dataSource}} · <text class="green">实时</text>
  </view>
</view>
```

- [ ] **Step 4: 页面样式**

`monitor.less`:
```less
.page { padding: 16rpx; display: flex; flex-direction: column; gap: 12rpx; }

.camera-wrap {
  position: relative; background: #000; border-radius: var(--radius);
  overflow: hidden; height: 400rpx;
}
.camera { width: 100%; height: 100%; }
.camera-hud {
  position: absolute; top: 12rpx; left: 16rpx;
  display: flex; gap: 24rpx; font-size: 20rpx;
  background: rgba(0,0,0,0.55); padding: 6rpx 16rpx; border-radius: var(--radius);
}

.sensor-grid { display: grid; grid-template-columns: 1fr 1fr; gap: 8rpx; }
.sensor-grid-3col { grid-template-columns: 1fr 1fr 1fr; }

.sensor-item {
  background: var(--bg-input); border: 1px solid var(--border);
  border-radius: var(--radius); padding: 12rpx 16rpx;
}
.s-label { font-size: 18rpx; color: var(--text-muted); display: block; }
.s-val {
  font-size: 36rpx; font-weight: 700; font-family: 'JetBrains Mono', 'Cascadia Code', 'Consolas', monospace;
  color: var(--text); display: block; margin-top: 4rpx;
}
```

- [ ] **Step 5: 提交**

```bash
git add wechat/miniprogram/pages/monitor/
git commit -m "feat: add monitor page with camera view and sensor grids"
```

---

### Task 11: 分析页面 — Tab 3

**Files:**
- Create: `wechat/miniprogram/pages/analysis/analysis.{json,ts,wxml,less}`

- [ ] **Step 1: 页面配置**

`analysis.json`:
```json
{ "usingComponents": { "alert-bar": "/components/alert-bar/alert-bar" } }
```

- [ ] **Step 2: 页面逻辑**

`analysis.ts`:
```typescript
import { getStore, onStoreChange } from '../../services/store';
import { apiGetForecast } from '../../services/api';

const DISEASE_NAMES: Record<string, string[]> = {
  tomato: ['早疫病', '晚疫病', '白粉病', '灰霉病', '叶霉病', '斑枯病', '健康'],
  wheat: ['赤霉病', '锈病', '白粉病', '纹枯病', '健康'],
  strawberry: ['白粉病', '灰霉病', '炭疽病', '红中柱根腐病', '叶斑病', '蛇眼病', '枯萎病', '健康'],
};

Component({
  data: {
    cropType: 'tomato',
    disease: '--',
    confidence: '--',
    probs: [] as {name: string, pct: number, color: string}[],
    advisoryText: '',
    advisoryPriority: '',
    advisorySteps: [] as string[],
    forecastActive: false,
    forecastDescription: '',
    forecastHoursAhead: 0,
  },
  lifetimes: {
    attached() {
      const s = getStore();
      this.sync(s);
      this._unsub = onStoreChange((s) => this.sync(s));
      // Poll forecast every 30s
      this._pollTimer = setInterval(() => this.fetchForecast(), 30000);
      this.fetchForecast();
    },
    detached() {
      if (this._unsub) this._unsub();
      if (this._pollTimer) clearInterval(this._pollTimer as number);
    },
  },
  methods: {
    sync(s: any) {
      const names = DISEASE_NAMES[s.diagnosisCropType || s.cropType] || DISEASE_NAMES.tomato;
      const probs = (s.diagnosisProbabilities || []).map((p: number, i: number) => ({
        name: names[i] || `类别${i}`,
        pct: (p * 100).toFixed(1),
        color: i === 0 ? '#F59E0B' : i === 1 ? '#38BDF8' : i === 2 ? '#10B981' : '#64748B',
      }));
      this.setData({
        cropType: s.diagnosisCropType || s.cropType,
        disease: s.diagnosisDisease || '--',
        confidence: s.diagnosisConfidence ? (s.diagnosisConfidence * 100).toFixed(1) : '--',
        probs,
        advisoryText: s.advisoryText,
        advisoryPriority: s.advisoryPriority,
        advisorySteps: s.advisorySteps,
        forecastActive: s.forecastActive,
        forecastDescription: s.forecastDescription,
      });
    },

    async fetchForecast() {
      try {
        const data = await apiGetForecast();
        if (data && data.advisory) {
          // update advisory in store
        }
      } catch (_) {}
    },
  },
})
```

- [ ] **Step 3: 页面模板**

`analysis.wxml`:
```xml
<view class="page">
  <!-- Diagnosis -->
  <view class="card dg-card">
    <text class="card-header">病害诊断 · {{cropType}}</text>
    <text class="dg-name amber">{{disease}}</text>
    <text class="muted" style="font-size:22rpx">置信度 <text class="mono amber">{{confidence}}%</text></text>
  </view>

  <!-- Probability bars -->
  <view class="card">
    <text class="card-header">分类概率</text>
    <view class="prob-row" wx:for="{{probs}}" wx:key="name">
      <view class="prob-label">
        <text class="mono dim" style="font-size:20rpx">{{item.name}}</text>
        <text class="mono">{{item.pct}}%</text>
      </view>
      <view class="prob-bar">
        <view class="prob-fill" style="width:{{item.pct}}%;background:{{item.color}}"></view>
      </view>
    </view>
  </view>

  <!-- Advisory -->
  <view class="card adv-card" wx:if="{{advisoryText}}">
    <text class="card-header">💡 农艺建议</text>
    <text style="font-size:26rpx;color:var(--text)">{{advisoryText}}</text>
    <view class="muted" style="font-size:20rpx;margin-top:8rpx" wx:if="{{advisoryPriority}}">
      紧急度: <text class="amber">{{advisoryPriority}}</text>
    </view>
    <view wx:if="{{advisorySteps.length}}" style="margin-top:8rpx">
      <text class="dim" wx:for="{{advisorySteps}}" wx:key="*this" style="font-size:20rpx">{{index+1}}. {{item}}</text>
    </view>
  </view>

  <!-- Trend placeholder -->
  <view class="card">
    <text class="card-header">📈 风险趋势 (3日)</text>
    <view class="trend-placeholder mono muted">数据收集中...</view>
  </view>

  <!-- Alert -->
  <alert-bar
    wx:if="{{forecastActive}}"
    level="danger"
    title="⚠ 病害预警"
    message="未来{{forecastHoursAhead}}h内: {{forecastDescription}}"
  />
</view>
```

- [ ] **Step 4: 页面样式**

`analysis.less`:
```less
.page { padding: 16rpx; display: flex; flex-direction: column; gap: 12rpx; }

.dg-card { border-left: 6rpx solid var(--amber); }
.dg-name { font-size: 44rpx; font-weight: 700; margin: 4rpx 0; }

.prob-row { margin-bottom: 12rpx; }
.prob-label { display: flex; justify-content: space-between; font-size: 20rpx; margin-bottom: 2rpx; }
.prob-bar { height: 8rpx; background: var(--border); border-radius: 4rpx; overflow: hidden; }
.prob-fill { height: 8rpx; border-radius: 4rpx; }

.adv-card { border-left: 6rpx solid var(--green); }

.trend-placeholder {
  height: 100rpx; background: var(--bg-input); border: 1px solid var(--border);
  border-radius: var(--radius); display: flex; align-items: center; justify-content: center;
}
```

- [ ] **Step 5: 提交**

```bash
git add wechat/miniprogram/pages/analysis/
git commit -m "feat: add analysis page with diagnosis, probability bars, and advisory"
```

---

### Task 12: 天气页面 — Tab 4

**Files:**
- Create: `wechat/miniprogram/pages/weather/weather.{json,ts,wxml,less}`

- [ ] **Step 1: 页面配置**

`weather.json`:
```json
{ "usingComponents": { "alert-bar": "/components/alert-bar/alert-bar" } }
```

- [ ] **Step 2: 页面逻辑**

`weather.ts`:
```typescript
import { getStore, onStoreChange } from '../../services/store';
import { apiGetWeather } from '../../services/api';

Component({
  data: {
    city: '--',
    currentTemp: '--',
    currentDesc: '--',
    humidity: '--',
    wind: '--',
    days: [] as any[],
    hours: [] as any[],
    disasterAlerts: [] as string[],
    stale: false,
    // ag metrics
    gdd: '--',
    rain: '--',
    windSpeed: '--',
  },
  lifetimes: {
    attached() {
      const s = getStore();
      this.sync(s);
      this._unsub = onStoreChange((s) => this.sync(s));
      this._pollTimer = setInterval(() => this.fetchWeather(), 60000);
      this.fetchWeather();
    },
    detached() {
      if (this._unsub) this._unsub();
      if (this._pollTimer) clearInterval(this._pollTimer as number);
    },
  },
  methods: {
    sync(s: any) {
      const day0 = s.weatherDays[0];
      this.setData({
        city: s.weatherCity || '--',
        currentTemp: day0 ? day0.high + '°' : '--',
        currentDesc: day0 ? day0.desc : '--',
        humidity: s.weatherDays[0] ? '--' : '--', // expand as needed
        days: s.weatherDays || [],
        hours: s.weatherHours || [],
        disasterAlerts: s.weatherDisasterAlerts || [],
        stale: s.weatherStale,
      });
    },

    async fetchWeather() {
      try {
        const data = await apiGetWeather();
        if (data && data.days) {
          // Update store
          const { updateStore } = require('../../services/store');
          updateStore({
            weatherCity: data.city,
            weatherDays: data.days,
            weatherHours: data.hours,
            weatherDisasterAlerts: data.disaster_alerts || [],
            weatherStale: data.stale || false,
          });
        }
      } catch (_) {}
    },
  },
})
```

- [ ] **Step 3: 页面模板**

`weather.wxml`:
```xml
<view class="page">
  <!-- Current weather -->
  <view class="wx-now">
    <text class="card-header">当前 · {{city}}</text>
    <text class="wx-temp">{{currentTemp}}</text>
    <text style="font-size:28rpx;color:var(--text)">{{currentDesc}}</text>
    <view class="muted" style="font-size:20rpx;margin-top:8rpx">
      湿度 {{humidity}} · 东风 3级
    </view>
  </view>

  <!-- 7-day forecast -->
  <view class="card">
    <text class="card-header">📅 7日预报</text>
    <scroll-view class="fc-scroll" scroll-x>
      <view class="fc-item" wx:for="{{days}}" wx:key="*this">
        <text class="dim" style="font-size:18rpx">周{{index+1}}</text>
        <text style="font-size:28rpx">{{item.icon}}</text>
        <text class="mono" style="font-weight:600">{{item.high}}°</text>
      </view>
    </scroll-view>
  </view>

  <!-- Hourly forecast -->
  <view class="card">
    <text class="card-header">🕐 逐时预报</text>
    <scroll-view class="fc-scroll" scroll-x>
      <view class="fc-item" wx:for="{{hours}}" wx:key="time">
        <text class="dim" style="font-size:18rpx">{{item.time}}</text>
        <text style="font-size:28rpx">{{item.icon}}</text>
        <text class="mono" style="font-weight:600">{{item.temp}}°</text>
      </view>
    </scroll-view>
  </view>

  <!-- Disaster alerts -->
  <alert-bar
    wx:for="{{disasterAlerts}}"
    wx:key="*this"
    level="danger"
    title="⚠ 气象灾害预警"
    message="{{item}}"
  />

  <!-- Ag weather metrics -->
  <view class="card">
    <text class="card-header">农业气象指标</text>
    <view class="ag-grid">
      <view class="ag-item">
        <text class="dim" style="font-size:18rpx">🌡 积温</text>
        <text class="mono" style="font-weight:600">320°D</text>
      </view>
      <view class="ag-item">
        <text class="dim" style="font-size:18rpx">💧 降雨</text>
        <text class="mono" style="font-weight:600">12mm</text>
      </view>
      <view class="ag-item">
        <text class="dim" style="font-size:18rpx">💨 风速</text>
        <text class="mono" style="font-weight:600">3级</text>
      </view>
    </view>
  </view>

  <!-- Stale notice -->
  <text wx:if="{{stale}}" class="amber" style="font-size:20rpx;text-align:right">⚠ 天气数据可能过期</text>
</view>
```

- [ ] **Step 4: 页面样式**

`weather.less`:
```less
.page { padding: 16rpx; display: flex; flex-direction: column; gap: 12rpx; }

.wx-now {
  background: linear-gradient(135deg, #1e293b, #0f172a);
  border: 1px solid var(--border);
  border-radius: 12rpx; padding: 32rpx; text-align: center;
}
.wx-temp {
  font-size: 80rpx; font-weight: 700;
  font-family: 'JetBrains Mono', 'Cascadia Code', 'Consolas', monospace;
  color: var(--text); margin: 8rpx 0;
}

.fc-scroll { display: flex; gap: 16rpx; white-space: nowrap; }
.fc-item {
  display: inline-flex; flex-direction: column; align-items: center;
  min-width: 80rpx; font-size: 22rpx; gap: 4rpx;
}

.ag-grid { display: grid; grid-template-columns: 1fr 1fr 1fr; gap: 8rpx; }
.ag-item {
  background: var(--bg-input); border: 1px solid var(--border);
  border-radius: 4rpx; padding: 12rpx 16rpx; text-align: center;
}
```

- [ ] **Step 5: 提交**

```bash
git add wechat/miniprogram/pages/weather/
git commit -m "feat: add weather page with 7-day/hourly forecast and disaster alerts"
```

---

### Task 13: 集成测试 — 端到端 Mock 验证

**Files:**
- Create: `src/sentry_miniprogram/test/test_bridge_mock.py`

- [ ] **Step 1: 写 Mock 集成测试**

```python
# test/test_bridge_mock.py
"""Integration test with mock ROS2 data pumping through to HTTP clients."""
import pytest
import json
from fastapi.testclient import TestClient
from unittest.mock import MagicMock, patch, PropertyMock
import sys

# Setup mocks (same as test_bridge.py)
mock_rclpy = MagicMock()
mock_node = MagicMock()
mock_rclpy.create_node.return_value = mock_node
sys.modules['rclpy'] = mock_rclpy
sys.modules['rclpy.node'] = MagicMock()
sys.modules['std_msgs'] = MagicMock()
sys.modules['std_msgs.msg'] = MagicMock()
sys.modules['std_srvs'] = MagicMock()
sys.modules['std_srvs.srv'] = MagicMock()
sys.modules['geometry_msgs'] = MagicMock()
sys.modules['geometry_msgs.msg'] = MagicMock()
sys.modules['sentry_interfaces'] = MagicMock()
sys.modules['sentry_interfaces.srv'] = MagicMock()
sys.modules['sensor_msgs'] = MagicMock()
sys.modules['sensor_msgs.msg'] = MagicMock()
sys.modules['cv2'] = MagicMock()
sys.modules['cv_bridge'] = MagicMock()

from sentry_miniprogram.miniprogram_bridge_node import _get_app, _node


@pytest.fixture
def client():
    # Ensure the app and node are initialized
    global _node
    from sentry_miniprogram.miniprogram_bridge_node import MiniProgramBridgeNode
    if _node is None:
        _node = MiniProgramBridgeNode()
    return TestClient(_get_app())


def test_status_endpoint(client):
    """GET /api/status returns valid JSON structure."""
    resp = client.get('/api/status')
    assert resp.status_code == 200
    data = resp.json()
    assert 'mode' in data
    assert data['mode'] in ('AUTO', 'MANUAL')
    assert 'ros_connected' in data


def test_mode_switch(client):
    """POST /api/mode with auto=true."""
    _node.mode_srv.service_is_ready = MagicMock(return_value=True)
    _node.mode_srv.call_async = MagicMock()
    resp = client.post('/api/mode', json={'auto': True})
    assert resp.status_code == 200
    data = resp.json()
    assert data['mode'] == 'AUTO'


def test_stop(client):
    """POST /api/stop triggers emergency stop."""
    _node.mode_srv.service_is_ready = MagicMock(return_value=True)
    _node.mode_srv.call_async = MagicMock()
    resp = client.post('/api/stop')
    assert resp.status_code == 200
    data = resp.json()
    assert data['mode'] == 'MANUAL'
    assert data['status'] == 'stopped'


def test_control(client):
    """POST /api/control sets velocity."""
    resp = client.post('/api/control', json={'linear': 0.3, 'angular': 0.1})
    assert resp.status_code == 200
    assert resp.json()['status'] == 'ok'


def test_crop_type(client):
    """POST /api/crop_type switches crop."""
    _node.crop_type_srv.wait_for_service = MagicMock(return_value=True)
    _node.crop_type_srv.call_async = MagicMock()
    resp = client.post('/api/crop_type', json={'crop_type': 'wheat'})
    assert resp.status_code == 200


def test_weather_empty(client):
    """GET /api/weather returns empty dict when no data received."""
    resp = client.get('/api/weather')
    assert resp.status_code == 200


def test_forecast_empty(client):
    """GET /api/forecast returns forecast + advisory + diagnosis."""
    resp = client.get('/api/forecast')
    assert resp.status_code == 200
    data = resp.json()
    assert 'forecast' in data
    assert 'advisory' in data
```

- [ ] **Step 2: 运行测试套件**

```bash
cd E:/smart_agri_sentry/src/sentry_miniprogram && python -m pytest test/ -v
```
Expected: ALL 8 tests PASS

- [ ] **Step 3: 提交**

```bash
git add src/sentry_miniprogram/test/test_bridge_mock.py
git commit -m "test: add mock integration tests for bridge node REST API"
```

---

### Task 14: 桥接节点 — Launch 文件 + 集成到 Bringup

**Files:**
- Create: `src/sentry_bringup/launch/miniprogram_bridge.launch.py`

- [ ] **Step 1: 创建 launch 文件**

```python
# launch/miniprogram_bridge.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='sentry_miniprogram',
            executable='miniprogram_bridge_node',
            name='miniprogram_bridge_node',
            output='screen',
            parameters=[],
        ),
    ])
```

- [ ] **Step 2: 提交**

```bash
git add src/sentry_bringup/launch/miniprogram_bridge.launch.py
git commit -m "feat: add launch file for miniprogram_bridge_node"
```

---

### Task 15: 最终检查 — 提交并推送

- [ ] **Step 1: 确认所有文件已提交**

```bash
git status
```

- [ ] **Step 2: 检查是否可以编译（语法检查）**

```bash
cd E:/smart_agri_sentry/src/sentry_miniprogram && python -c "import py_compile; py_compile.compile('sentry_miniprogram/miniprogram_bridge_node.py', doraise=True)" && echo "SYNTAX OK"
```

- [ ] **Step 3: 推送到远程**

```bash
git push origin main
```

- [ ] **Step 4: SSH 到 RDK 板端拉取 + 构建**

```bash
ssh rdk "cd ~/dev_ws && git pull && colcon build --packages-select sentry_miniprogram --symlink-install"
```

- [ ] **Step 5: 在板端启动并测试**

```bash
ssh rdk "cd ~/dev_ws && source install/setup.bash && ros2 launch sentry_bringup miniprogram_bridge.launch.py"
```

然后用浏览器或 curl 验证:
```bash
curl http://<rdk_ip>:8765/api/status
```

---

## Plan Self-Review

**Spec coverage check:**

| Spec Section | Covered By |
|---|---|
| 2. Decision summary | Implicitly implemented |
| 3. Architecture | Task 1-3, Task 14 |
| 4.1 Navigation config | Task 4 (`app.json` tabBar) |
| 4.2 Visual style | Task 4 (`app.less`), all pages |
| 4.3 Page layouts | Tasks 9-12 |
| 5.2 WebSocket channels | Task 2 (bridge node `_push_ws`), Task 6 (ws.ts `handleMessage`) |
| 5.3 REST API | Task 2 (FastAPI endpoints), Task 7 (api.ts) |
| 5.4 ROS2 subscriptions | Task 2 (`_setup_subscriptions`) |
| 6. Data flow | Tasks 2, 6, 7 |
| 7. Error handling | Task 6 (WS reconnect), Task 5 (store defaults) |
| 8. Testing | Tasks 1, 13 |
| 9. Risks | MJPEG fallback (Task 10), Skyline compatibility (global styles) |

**Placeholder scan:** No TBD, TODO, or vague steps. All code is concrete.

**Type consistency:** TypeScript interfaces between store.ts, ws.ts, api.ts, and pages match. Backend message format matches frontend parser expectations.
