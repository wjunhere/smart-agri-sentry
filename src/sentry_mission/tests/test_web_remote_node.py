"""Tests for web_remote_node service call handling."""

import sys
import threading
import types
from pathlib import Path
from unittest import mock

import pytest

PKG_ROOT = Path(__file__).resolve().parents[1]
if str(PKG_ROOT) not in sys.path:
    sys.path.insert(0, str(PKG_ROOT))


@pytest.fixture(scope='module', autouse=True)
def mock_ros2():
    """Mock ROS2 modules so web_remote_node can be imported off-board."""
    modules = {
        'rclpy': types.ModuleType('rclpy'),
        'rclpy.node': types.ModuleType('rclpy.node'),
        'geometry_msgs': types.ModuleType('geometry_msgs'),
        'geometry_msgs.msg': types.ModuleType('geometry_msgs.msg'),
        'std_srvs': types.ModuleType('std_srvs'),
        'std_srvs.srv': types.ModuleType('std_srvs.srv'),
        'sentry_interfaces': types.ModuleType('sentry_interfaces'),
        'sentry_interfaces.srv': types.ModuleType('sentry_interfaces.srv'),
        'sentry_interfaces.msg': types.ModuleType('sentry_interfaces.msg'),
    }

    modules['rclpy'].init = mock.MagicMock()
    modules['sentry_interfaces'].__path__ = []
    modules['rclpy'].shutdown = mock.MagicMock()
    modules['rclpy'].ok = mock.MagicMock(return_value=True)
    modules['rclpy'].Node = object
    modules['rclpy.node'].Node = object

    Twist = type('Twist', (), {})
    modules['geometry_msgs.msg'].Twist = Twist

    SetBool = type('SetBool', (), {})
    SetBool.Request = type('Request', (), {})
    modules['std_srvs.srv'].SetBool = SetBool

    SetCropType = type('SetCropType', (), {})
    modules['sentry_interfaces.srv'].SetCropType = SetCropType
    MissionStatus = type('MissionStatus', (), {})
    modules['sentry_interfaces.msg'].MissionStatus = MissionStatus


    for name, mod in modules.items():
        sys.modules[name] = mod

    yield

    for name in modules:
        sys.modules.pop(name, None)


def test_set_mode_auto_adds_done_callback():
    from sentry_mission.web_remote_node import WebRemoteNode

    node = mock.MagicMock()
    node.create_publisher = mock.MagicMock()
    node.create_client = mock.MagicMock()
    node.declare_parameter = mock.MagicMock()
    node.get_parameter = mock.MagicMock(return_value=mock.MagicMock(value=1.0))
    node.get_logger = mock.MagicMock(return_value=mock.MagicMock())
    node.create_timer = mock.MagicMock()

    fake_future = mock.MagicMock()
    fake_client = mock.MagicMock()
    fake_client.service_is_ready = mock.MagicMock(return_value=True)
    fake_client.call_async = mock.MagicMock(return_value=fake_future)
    node.create_client.return_value = fake_client

    # Patch super().__init__ to avoid ROS2 node initialization
    with mock.patch('builtins.super'):
        web = WebRemoteNode.__new__(WebRemoteNode)
        web.__dict__['node'] = node
        # Manually set attributes normally set by __init__
        web.cmd_pub = node.create_publisher()
        web.mode_srv = fake_client
        web.max_linear = 0.5
        web.max_angular = 1.0
        web.mode = 'AUTO'
        web.linear = 0.0
        web.angular = 0.0
        web.lock = threading.Lock()
        web.last_cmd_time = 0.0
        web.TIMEOUT = 0.5
        web.timer = None
        web.get_logger = node.get_logger

        result = web.set_mode_auto(False)

    assert result is True
    fake_client.call_async.assert_called_once()
    fake_future.add_done_callback.assert_called_once()


def test_on_mode_response_logs_success():
    from sentry_mission.web_remote_node import WebRemoteNode

    web = mock.MagicMock()
    logger = mock.MagicMock()
    web.get_logger = mock.MagicMock(return_value=logger)

    fake_future = mock.MagicMock()
    response = mock.MagicMock()
    response.success = True
    response.message = 'OK'
    fake_future.result = mock.MagicMock(return_value=response)

    WebRemoteNode._on_mode_response(web, fake_future, True)
    logger.info.assert_called_once()


def test_on_mode_response_logs_failure():
    from sentry_mission.web_remote_node import WebRemoteNode

    web = mock.MagicMock()
    logger = mock.MagicMock()
    web.get_logger = mock.MagicMock(return_value=logger)

    fake_future = mock.MagicMock()
    response = mock.MagicMock()
    response.success = False
    response.message = 'Rejected'
    fake_future.result = mock.MagicMock(return_value=response)

    WebRemoteNode._on_mode_response(web, fake_future, True)
    logger.warn.assert_called_once()


def test_on_stop_response_logs_success():
    from sentry_mission.web_remote_node import WebRemoteNode

    web = mock.MagicMock()
    logger = mock.MagicMock()
    web.get_logger = mock.MagicMock(return_value=logger)

    fake_future = mock.MagicMock()
    response = mock.MagicMock()
    response.success = True
    response.message = 'Stopped'
    fake_future.result = mock.MagicMock(return_value=response)

    WebRemoteNode._on_stop_response(web, fake_future)
    logger.info.assert_called_once()

def test_stack_script_env_preserves_frontend_control_plane():
    from sentry_mission.web_remote_node import _stack_script_env

    env = _stack_script_env({'KEEP': 'yes'})

    assert env['KEEP'] == 'yes'
    assert env['SENTRY_PRESERVE_WEB'] == '1'
    assert env['ENABLE_WEB'] == 'false'


def test_stack_script_env_enables_vision_and_advisory_for_cruise():
    from sentry_mission.web_remote_node import _stack_script_env

    env = _stack_script_env({
        'ENABLE_VISION': 'false',
        'ENABLE_ADVISORY': 'false',
    })

    assert env['SENTRY_PRESERVE_WEB'] == '1'
    assert env['ENABLE_WEB'] == 'false'
    assert env['ENABLE_VISION'] == 'true'
    assert env['ENABLE_ADVISORY'] == 'true'


def test_mission_status_complete_when_all_waypoints_reached():
    from sentry_mission.web_remote_node import _mission_status_is_complete

    msg = types.SimpleNamespace(
        state='PATROL',
        current_wp_idx=3,
        total_wps=3,
    )

    assert _mission_status_is_complete(msg) is True


def test_mission_status_not_complete_before_last_waypoint():
    from sentry_mission.web_remote_node import _mission_status_is_complete

    msg = types.SimpleNamespace(
        state='PATROL',
        current_wp_idx=2,
        total_wps=3,
    )

    assert _mission_status_is_complete(msg) is False


def test_validate_vision_inference_mode_accepts_known_modes():
    from sentry_mission.web_remote_node import _validate_vision_inference_mode

    assert _validate_vision_inference_mode('triggered') == 'triggered'
    assert _validate_vision_inference_mode('independent') == 'independent'


def test_validate_vision_inference_mode_rejects_unknown_mode():
    from sentry_mission.web_remote_node import _validate_vision_inference_mode

    with pytest.raises(ValueError):
        _validate_vision_inference_mode('always')


def test_set_vision_inference_mode_updates_mode_on_success():
    from sentry_mission.web_remote_node import WebRemoteNode

    web = WebRemoteNode.__new__(WebRemoteNode)
    web.stack_lock = threading.Lock()
    web.lock = threading.Lock()
    web.vision_inference_mode = 'triggered'
    web._start_independent_diagnosis_locked = mock.MagicMock(
        return_value=(True, 'started'))
    web._stop_independent_diagnosis_locked = mock.MagicMock()

    ok, message = web.set_vision_inference_mode('independent')

    assert ok is True
    assert message == 'started'
    assert web.vision_inference_mode == 'independent'
