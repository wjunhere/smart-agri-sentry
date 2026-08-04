#!/usr/bin/env python3
"""Mini-program bridge node — FastAPI + WebSocket gateway.

Serves WeChat mini-program with:
- WS /ws: real-time sensor/status/mission/diagnosis push
- REST: low-frequency data (weather, forecast) + control commands
- GET /api/camera: MJPEG video stream
"""

import asyncio
import json
import os
import subprocess
import threading
import time
from contextlib import asynccontextmanager
from pathlib import Path

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_srvs.srv import SetBool
from sentry_interfaces.srv import SetCropType

from fastapi import FastAPI, WebSocket, WebSocketDisconnect, Response
from fastapi.responses import JSONResponse, StreamingResponse
import uvicorn


def _stack_script_env():
    """Environment for frontend-triggered stack scripts.

    Mirrors web_remote_node: preserve the web control plane (web_remote +
    rosbridge + this bridge) when the full stack starts, otherwise the
    frontend's own button click would kill the server handling it.
    """
    env = dict(os.environ)
    env['SENTRY_PRESERVE_WEB'] = '1'
    env['ENABLE_WEB'] = 'false'
    env['ENABLE_VISION'] = 'true'
    env['ENABLE_ADVISORY'] = 'true'
    env['CAMERA_BACKEND'] = 'mipi'
    return env


# ============ ROS2 Node ============

class MiniProgramBridgeNode(Node):
    def __init__(self):
        super().__init__('miniprogram_bridge_node')

        # Publishers
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Services
        self.mode_srv = self.create_client(SetBool, '/set_auto_mode')
        self.crop_type_srv = self.create_client(SetCropType, '/set_crop_type')
        self.vision_pause_srv = self.create_client(
            SetBool, '/vision/plant_detector/pause')

        # State cache
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

        # Camera frame
        self._latest_frame = None
        self._latest_jpeg = None

        # Subscriptions
        self._setup_subscriptions()

        # Async queues for WebSocket push
        self.ws_queues: list[asyncio.Queue] = []
        self._loop = None

        # Stack orchestration (mirrors web_remote_node semantics)
        self.declare_parameter(
            'stack_start_script',
            '/home/sunrise/dev_ws/scripts/rdk/start_robot_stack.sh')
        self.declare_parameter(
            'stack_stop_script',
            '/home/sunrise/dev_ws/scripts/rdk/stop_robot_stack.sh')
        self.declare_parameter('stack_script_timeout_sec', 180.0)
        self.stack_start_script = self.get_parameter('stack_start_script').value
        self.stack_stop_script = self.get_parameter('stack_stop_script').value
        self.stack_script_timeout = float(
            self.get_parameter('stack_script_timeout_sec').value)
        self.stack_lock = threading.Lock()
        self.stack_state = 'idle'  # idle|preheating|starting|cruising|stopping|error
        self.last_stack_output = ''

        self.get_logger().info('miniprogram_bridge_node started')

    def _setup_subscriptions(self):
        from sentry_interfaces.msg import (
            Environment, ChassisStatus,
            Diagnosis, MissionStatus, PlantDetection,
            WeatherForecast, ForecastAlert, AdvisoryAction,
        )
        from sensor_msgs.msg import CompressedImage

        self.create_subscription(
            Environment, '/sensor/environment_fixed',
            self._on_environment, 10)
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
        self.create_subscription(
            CompressedImage, '/out/compressed',
            self._on_camera_frame, 10)
        from sentry_interfaces.msg import LLMAnalysis as LLMAnalysisMsg
        self.create_subscription(
            LLMAnalysisMsg, '/llm/analysis',
            self._on_llm_analysis, 10)

    # --- Subscription callbacks ---

    def _on_environment(self, msg):
        self.sensors['air_temp'] = round(msg.air_temp, 1)
        self.sensors['air_humidity'] = round(msg.air_humidity, 1)
        self.sensors['co2'] = round(msg.air_co2, 0)
        self.sensors['soil_temp'] = round(msg.soil_temp, 1)
        self.sensors['soil_humidity'] = round(msg.soil_humidity, 1)
        self.sensors['leaf_wetness'] = round(msg.leaf_wetness, 1)
        # Environment carries no NPK fields; keep the payload keys present
        # (null) so the mini-program frontend degrades gracefully.
        self.sensors['soil_n'] = None
        self.sensors['soil_p'] = None
        self.sensors['soil_k'] = None
        self.sensors['data_source'] = msg.data_source
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
        r1 = lambda x: round(x, 1)
        self.weather = {
            'city': msg.city,
            'lat': r1(msg.lat),
            'lon': r1(msg.lon),
            'days': [{'day_offset': d.day_offset, 'temp_high': r1(d.temp_high),
                      'temp_low': r1(d.temp_low), 'humidity': r1(d.humidity),
                      'precipitation': r1(d.precipitation), 'wind_speed': r1(d.wind_speed),
                      'weather_desc': d.weather_desc}
                     for d in msg.days],
            'hours': [{'hour_offset': h.hour_offset, 'temp': r1(h.temp),
                       'humidity': r1(h.humidity), 'precipitation': r1(h.precipitation),
                       'wind_speed': r1(h.wind_speed)}
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

    def _on_camera_frame(self, msg):
        # /out/compressed is already JPEG — forward as-is. The previous
        # decode + re-encode per frame cost ~1 CPU core at 30fps.
        self._latest_jpeg = bytes(msg.data)

    def _now_ms(self) -> int:
        return int(time.time() * 1000)

    def _push_ws(self, data: dict):
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
        self._set_vision_paused(not auto)
        self.get_logger().info(f"Mode switched to {self.mode}")
        return True

    def _set_vision_paused(self, paused: bool):
        """Gate YOLO inference with cruise mode: run in AUTO, idle in MANUAL."""
        try:
            ready = self.vision_pause_srv.service_is_ready()
        except Exception:
            ready = False
        if not ready:
            return
        req = SetBool.Request()
        req.data = paused
        self.vision_pause_srv.call_async(req)

    def set_velocity(self, linear: float, angular: float):
        twist = Twist()
        twist.linear.x = max(-0.5, min(0.5, linear))
        twist.angular.z = max(-1.0, min(1.0, angular))
        self.cmd_pub.publish(twist)
        self.linear = twist.linear.x
        self.angular = twist.angular.z

    def emergency_stop(self):
        self.set_mode(False)
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

    def _on_llm_analysis(self, msg):
        self._push_ws({
            'type': 'llm',
            'ts': self._now_ms(),
            'data': {
                'status': msg.status,
                'summary': msg.summary,
                'suggestions': list(msg.suggestions),
                'risk_level': msg.risk_level,
                'focus_areas': list(msg.focus_areas),
                'next_check': msg.next_check,
                'trigger': msg.trigger,
            }
        })

    # --- Stack orchestration ---

    def _run_stack_script(self, script_path: str):
        path = Path(script_path)
        if not path.exists():
            return False, f'Script not found: {path}'
        try:
            result = subprocess.run(
                ['bash', str(path)],
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                timeout=self.stack_script_timeout,
                env=_stack_script_env())
        except subprocess.TimeoutExpired as exc:
            return False, f'Script timed out: {path}\n{exc.stdout or ""}'
        except Exception as exc:
            return False, f'Script failed to start: {path}: {exc}'
        output = result.stdout or ''
        self.last_stack_output = output[-4000:]
        if result.returncode != 0:
            return False, output
        return True, output

    def _set_stack_state(self, state: str, message: str = ''):
        self.stack_state = state
        self._push_ws({
            'type': 'stack_status',
            'ts': self._now_ms(),
            'data': {'state': state, 'message': message,
                     'alive': self._stack_alive()},
        })

    def _stack_alive(self) -> bool:
        """Process-level fact: the main stack is running.

        Checks the node graph first — it settles faster than service
        discovery right after the stack (re)starts, and a false negative
        here would trigger a full clean restart that kills a healthy stack.
        """
        try:
            names = {n for n, _ in self.get_node_names_and_namespaces()}
        except Exception:
            names = set()
        if 'mission_control_node' in names:
            return True
        return self.mode_srv.service_is_ready()

    def stack_preheat(self):
        def work():
            with self.stack_lock:
                if self._stack_alive():
                    # Adopt the resident stack instead of a clean restart.
                    self._set_stack_state(
                        'idle', 'Stack already resident; preheat skipped')
                    return
                self._set_stack_state('preheating', 'Running start_robot_stack.sh')
                ok, output = self._run_stack_script(self.stack_start_script)
                if ok and self._stack_alive():
                    self._set_stack_state('idle', 'Preheated; ready to cruise')
                else:
                    self._set_stack_state('error', output[-500:])
        threading.Thread(target=work, daemon=True).start()

    def stack_start(self):
        def work():
            with self.stack_lock:
                self._set_stack_state('starting', 'Running start_robot_stack.sh')
                if not self._stack_alive():
                    ok, output = self._run_stack_script(self.stack_start_script)
                    if not ok:
                        self._set_stack_state('error', output[-500:])
                        return
                if not self._stack_alive():
                    self._set_stack_state(
                        'error', '/set_auto_mode not available after stack start')
                    return
                if self.set_mode(True):
                    self._set_stack_state(
                        'cruising', 'Stack running, AUTO mode engaged')
                else:
                    self._set_stack_state('error', '/set_auto_mode call failed')
        threading.Thread(target=work, daemon=True).start()

    def stack_stop(self):
        """End the cruise but keep the robot stack resident (fast path)."""
        def work():
            with self.stack_lock:
                self._set_stack_state('stopping', 'Switching to MANUAL')
                self.emergency_stop()
                self._set_stack_state(
                    'idle', 'Cruise stopped; stack stays resident')
        threading.Thread(target=work, daemon=True).start()

    def stack_shutdown(self):
        """Full teardown of the robot stack; the bridge itself stays up."""
        def work():
            with self.stack_lock:
                self._set_stack_state('stopping', 'Running stop_robot_stack.sh')
                self.emergency_stop()
                ok, output = self._run_stack_script(self.stack_stop_script)
                if ok:
                    self._set_stack_state('idle', 'Stack shut down')
                else:
                    self._set_stack_state('error', output[-500:])
        threading.Thread(target=work, daemon=True).start()

    def get_stack_status(self) -> dict:
        return {
            'state': self.stack_state,
            'stack_alive': self._stack_alive(),
            'last_output': self.last_stack_output[-500:],
        }

    def get_status(self) -> dict:
        return {
            'mode': self.mode,
            'linear': self.linear,
            'angular': self.angular,
            'ros_connected': self.ros_connected,
            'sensors': self.sensors,
            'mission': self.mission,
            'plant_detect': self.plant_detect,
            'stack': self.get_stack_status(),
        }


# ============ FastAPI App ============

_node: MiniProgramBridgeNode = None
_app: FastAPI = None


def get_app() -> FastAPI:
    global _app, _node
    if _app is not None:
        return _app

    @asynccontextmanager
    async def lifespan(app: FastAPI):
        if _node is not None:
            _node._loop = asyncio.get_running_loop()
        yield
        if _node is not None:
            _node.ws_queues.clear()

    _app = FastAPI(lifespan=lifespan)

    # --- REST Endpoints ---

    @_app.get('/api/status')
    async def api_status():
        if _node is None:
            return {'mode': 'AUTO', 'ros_connected': False}
        return _node.get_status()

    @_app.get('/api/weather')
    async def api_weather():
        return _node.weather if _node else {}

    @_app.get('/api/forecast')
    async def api_forecast():
        if _node is None:
            return {'forecast': {}, 'advisory': None, 'diagnosis': None}
        return {
            'forecast': _node.forecast,
            'advisory': _node.advisory,
            'diagnosis': _node.diagnosis,
        }

    @_app.post('/api/mode')
    async def api_set_mode(data: dict):
        auto = data.get('auto', False)
        ok = _node.set_mode(auto) if _node else False
        return {'status': 'ok' if ok else 'error', 'mode': _node.mode if _node else 'AUTO'}

    @_app.post('/api/control')
    async def api_control(data: dict):
        linear = data.get('linear', 0.0)
        angular = data.get('angular', 0.0)
        if _node:
            _node.set_velocity(linear, angular)
        return {'status': 'ok'}

    @_app.post('/api/stop')
    async def api_stop():
        if _node:
            _node.emergency_stop()
        return {'status': 'stopped', 'mode': _node.mode if _node else 'MANUAL'}

    @_app.post('/api/crop_type')
    async def api_crop_type(data: dict):
        crop = data.get('crop_type', '')
        ok = _node.set_crop_type(crop) if _node else False
        return {'status': 'ok' if ok else 'error'}

    @_app.post('/api/llm/analyze')
    async def api_llm_analyze():
        if _node is None:
            return {'status': 'error', 'summary': 'Bridge node not ready'}
        from sentry_interfaces.srv import LLMAnalyze
        srv = _node.create_client(LLMAnalyze, '/llm/analyze')
        if not srv.wait_for_service(timeout_sec=5.0):
            return JSONResponse(
                status_code=503,
                content={'status': 'error',
                         'summary': 'LLM service not available (api_key 未配置或节点未启动)'})
        req = LLMAnalyze.Request()
        future = srv.call_async(req)
        event = threading.Event()
        result = {}
        def done_cb(fut):
            try:
                resp = fut.result()
                result['status'] = resp.status
                result['summary'] = resp.summary
                result['suggestions'] = list(resp.suggestions)
                result['risk_level'] = resp.risk_level
                result['focus_areas'] = list(resp.focus_areas)
                result['next_check'] = resp.next_check
                result['trigger'] = resp.trigger
            except Exception as e:
                result['status'] = 'error'
                result['summary'] = str(e)
            finally:
                event.set()
        future.add_done_callback(done_cb)
        if not event.wait(timeout=65.0):
            return {'status': 'timeout', 'summary': 'LLM request timed out'}
        return result

    @_app.get('/api/camera')
    async def api_camera():
        async def generate():
            while True:
                try:
                    if _node is not None and _node._latest_jpeg is not None:
                        yield (b'--frame\r\n'
                               b'Content-Type: image/jpeg\r\n\r\n'
                               + _node._latest_jpeg + b'\r\n')
                except Exception:
                    pass
                await asyncio.sleep(0.1)

        return StreamingResponse(
            generate(),
            media_type='multipart/x-mixed-replace; boundary=frame'
        )

    @_app.get('/api/camera/snapshot')
    async def api_camera_snapshot():
        if _node is None or _node._latest_jpeg is None:
            return {'status': 'error', 'message': 'No camera frame available'}
        return Response(
            content=_node._latest_jpeg,
            media_type='image/jpeg',
            headers={'Cache-Control': 'no-cache, no-store, must-revalidate'}
        )

    # --- Stack Orchestration Endpoints ---

    @_app.get('/stack/status')
    async def stack_status():
        if _node is None:
            return {'state': 'idle', 'stack_alive': False, 'last_output': ''}
        return _node.get_stack_status()

    @_app.post('/stack/preheat')
    async def stack_preheat():
        if _node is None:
            return {'status': 'error', 'message': 'Bridge node not ready'}
        _node.stack_preheat()
        return {'status': 'accepted', 'state': _node.stack_state}

    @_app.post('/stack/start')
    async def stack_start():
        if _node is None:
            return {'status': 'error', 'message': 'Bridge node not ready'}
        _node.stack_start()
        return {'status': 'accepted', 'state': _node.stack_state}

    @_app.post('/stack/stop')
    async def stack_stop():
        if _node is None:
            return {'status': 'error', 'message': 'Bridge node not ready'}
        _node.stack_stop()
        return {'status': 'accepted', 'state': _node.stack_state}

    @_app.post('/stack/shutdown')
    async def stack_shutdown():
        if _node is None:
            return {'status': 'error', 'message': 'Bridge node not ready'}
        _node.stack_shutdown()
        return {'status': 'accepted', 'state': _node.stack_state}

    # --- WebSocket Endpoint ---

    @_app.websocket('/ws')
    async def ws_endpoint(ws: WebSocket):
        await ws.accept()
        q: asyncio.Queue = asyncio.Queue(maxsize=256)
        if _node is not None:
            _node.ws_queues.append(q)
            # Push full state snapshot on connect
            await ws.send_json({
                'type': 'snapshot',
                'ts': _node._now_ms(),
                'data': _node.get_status()
            })
        try:
            while True:
                data = await q.get()
                await ws.send_json(data)
        except (WebSocketDisconnect, Exception):
            pass
        finally:
            if _node is not None and q in _node.ws_queues:
                _node.ws_queues.remove(q)

    return _app


# ============ Entry Point ============

def _start_fastapi():
    app = get_app()
    uvicorn.run(app, host='0.0.0.0', port=8765, log_level='info')


def main(args=None):
    global _node
    rclpy.init(args=args)
    _node = MiniProgramBridgeNode()

    api_thread = threading.Thread(target=_start_fastapi, daemon=True)
    api_thread.start()

    try:
        rclpy.spin(_node)
    except KeyboardInterrupt:
        pass
    finally:
        _node.destroy_node()
        rclpy.shutdown()
