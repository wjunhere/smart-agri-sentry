"""Tests for web_remote_node service call handling."""

import sys
import threading
import types
from unittest import mock

import pytest


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
    }

    modules['rclpy'].init = mock.MagicMock()
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
