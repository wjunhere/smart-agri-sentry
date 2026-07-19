"""Tests for miniprogram_bridge_node.

These tests require a ROS2 environment with rclpy available.
Run on RDK X5: cd ~/dev_ws && python3 -m pytest src/sentry_miniprogram/test/ -v
"""

import pytest
import sys
from unittest.mock import MagicMock, patch


# Build comprehensive mocks for all ROS2/fastapi imports BEFORE importing the module
def _setup_mocks():
    """Set up mock modules for all imports the bridge node needs.

    rclpy.node.Node must be a REAL base class (not a MagicMock instance):
    subclassing a MagicMock instance swallows every method defined in the
    class body, so MiniProgramBridgeNode() would return a plain mock and
    none of its real methods would exist. _FakeNode is a minimal stand-in
    whose unknown attributes are per-instance cached MagicMocks.
    """
    import types

    class _FakeNode:
        def __init__(self, *args, **kwargs):
            self.__dict__['_infra_mocks'] = {}

        def __getattr__(self, name):
            return self.__dict__['_infra_mocks'].setdefault(name, MagicMock())

    rclpy_mod = types.ModuleType('rclpy')
    node_mod = types.ModuleType('rclpy.node')
    node_mod.Node = _FakeNode
    rclpy_mod.node = node_mod
    sys.modules['rclpy'] = rclpy_mod
    sys.modules['rclpy.node'] = node_mod

    mocks = {
        'std_msgs': MagicMock(),
        'std_msgs.msg': MagicMock(),
        'std_srvs': MagicMock(),
        'std_srvs.srv': MagicMock(),
        'geometry_msgs': MagicMock(),
        'geometry_msgs.msg': MagicMock(),
        'sentry_interfaces': MagicMock(),
        'sentry_interfaces.msg': MagicMock(),
        'sentry_interfaces.srv': MagicMock(),
        'sensor_msgs': MagicMock(),
        'sensor_msgs.msg': MagicMock(),
        'cv2': MagicMock(),
        'cv_bridge': MagicMock(),
        'numpy': MagicMock(),
    }
    for name, mock in mocks.items():
        sys.modules[name] = mock


_setup_mocks()

# Now safe to import
from sentry_miniprogram.miniprogram_bridge_node import get_app, MiniProgramBridgeNode, _node as bridge_node_module


@pytest.fixture
def node():
    """Create a bridge node with mocked ROS2 infrastructure."""
    # Ensure the global _node is set for get_app()
    import sentry_miniprogram.miniprogram_bridge_node as bm
    old_node = bm._node
    node = MiniProgramBridgeNode()
    bm._node = node
    yield node
    bm._node = old_node


@pytest.fixture
def client(node):
    """Create FastAPI TestClient with a real bridge node."""
    from fastapi.testclient import TestClient
    app = get_app()
    return TestClient(app)


def test_status_endpoint(client):
    """GET /api/status returns valid JSON structure."""
    resp = client.get('/api/status')
    assert resp.status_code == 200
    data = resp.json()
    assert 'mode' in data
    assert data['mode'] in ('AUTO', 'MANUAL')
    assert 'ros_connected' in data


def test_mode_switch(node, client):
    """POST /api/mode with auto=true."""
    node.mode_srv.service_is_ready = MagicMock(return_value=True)
    node.mode_srv.call_async = MagicMock()
    resp = client.post('/api/mode', json={'auto': True})
    assert resp.status_code == 200
    data = resp.json()
    assert data['mode'] == 'AUTO'


def test_stop(node, client):
    """POST /api/stop triggers emergency stop."""
    node.mode_srv.service_is_ready = MagicMock(return_value=True)
    node.mode_srv.call_async = MagicMock()
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


def test_crop_type(node, client):
    """POST /api/crop_type switches crop."""
    node.crop_type_srv.wait_for_service = MagicMock(return_value=True)
    node.crop_type_srv.call_async = MagicMock()
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


def test_sensor_topic_names():
    """Bridge must subscribe to the topics uart_bridge actually publishes."""
    node = MiniProgramBridgeNode()
    subscribed = [c.args[1] for c in node.create_subscription.call_args_list]
    assert '/sensor/environment_mobile' in subscribed
    assert '/sensor/soil_nutrition' in subscribed
    assert '/sentry/sensor/environment_mobile' not in subscribed
    assert '/sentry/sensor/soil_nutrition' not in subscribed


def test_stack_status_idle(client):
    """GET /stack/status returns state machine state."""
    resp = client.get('/stack/status')
    assert resp.status_code == 200
    data = resp.json()
    assert data['state'] in ('idle', 'preheating', 'starting', 'cruising', 'stopping', 'error')


def test_stack_preheat_accepted(node, client):
    """POST /stack/preheat runs the start script in background."""
    node._run_stack_script = MagicMock(return_value=(True, 'ok'))
    resp = client.post('/stack/preheat')
    assert resp.status_code == 200
    assert resp.json()['status'] == 'accepted'


def test_stack_start_calls_script(node, client):
    """POST /stack/start triggers start script."""
    node._run_stack_script = MagicMock(return_value=(True, 'ok'))
    node.mode_srv.service_is_ready = MagicMock(return_value=True)
    node.mode_srv.call_async = MagicMock()
    resp = client.post('/stack/start')
    assert resp.status_code == 200
    assert resp.json()['status'] == 'accepted'


def test_stack_stop_accepted(node, client):
    """POST /stack/stop runs the stop script."""
    node._run_stack_script = MagicMock(return_value=(True, 'ok'))
    resp = client.post('/stack/stop')
    assert resp.status_code == 200
    assert resp.json()['status'] == 'accepted'


def test_llm_analyze_503_when_unavailable(node, client):
    """POST /api/llm/analyze returns 503 when LLM service is absent (no key)."""
    mock_srv = MagicMock()
    mock_srv.wait_for_service = MagicMock(return_value=False)
    node.create_client = MagicMock(return_value=mock_srv)
    resp = client.post('/api/llm/analyze')
    assert resp.status_code == 503
    assert resp.json()['status'] == 'error'
