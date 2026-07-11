"""Tests for miniprogram_bridge_node.

These tests require a ROS2 environment with rclpy available.
Run on RDK X5: cd ~/dev_ws && python3 -m pytest src/sentry_miniprogram/test/ -v
"""

import pytest
import sys
from unittest.mock import MagicMock, patch


# Build comprehensive mocks for all ROS2/fastapi imports BEFORE importing the module
def _setup_mocks():
    """Set up mock modules for all imports the bridge node needs."""
    mocks = {
        'rclpy': MagicMock(),
        'rclpy.node': MagicMock(),
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
