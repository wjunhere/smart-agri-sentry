"""Tests for Hikrobot camera node helpers."""

import importlib
import os
import sys
import types
from pathlib import Path
from unittest import mock

import pytest

PKG_ROOT = Path(__file__).resolve().parents[1]
if str(PKG_ROOT) not in sys.path:
    sys.path.insert(0, str(PKG_ROOT))


@pytest.fixture(autouse=True)
def mock_ros2_and_cv():
    modules = {
        'rclpy': types.ModuleType('rclpy'),
        'rclpy.node': types.ModuleType('rclpy.node'),
        'sensor_msgs': types.ModuleType('sensor_msgs'),
        'sensor_msgs.msg': types.ModuleType('sensor_msgs.msg'),
        'cv_bridge': types.ModuleType('cv_bridge'),
        'cv2': types.ModuleType('cv2'),
    }
    modules['rclpy'].init = mock.MagicMock()
    modules['rclpy'].shutdown = mock.MagicMock()
    modules['rclpy'].ok = mock.MagicMock(return_value=True)
    modules['rclpy'].spin = mock.MagicMock()
    modules['rclpy.node'].Node = object
    modules['sensor_msgs.msg'].Image = type('Image', (), {})
    modules['cv_bridge'].CvBridge = type(
        'CvBridge', (), {'cv2_to_imgmsg': mock.MagicMock()})
    modules['cv2'].COLOR_RGB2BGR = 4
    modules['cv2'].INTER_AREA = 3
    modules['cv2'].cvtColor = mock.MagicMock()
    modules['cv2'].resize = mock.MagicMock()

    for name, mod in modules.items():
        sys.modules[name] = mod

    yield

    for name in modules:
        sys.modules.pop(name, None)
    sys.modules.pop('sentry_bringup.hikrobot_camera_node', None)


def test_default_topic_matches_vision_pipeline():
    from sentry_bringup.hikrobot_camera_node import DEFAULT_IMAGE_TOPIC

    assert DEFAULT_IMAGE_TOPIC == '/sentry/camera/image_raw'


def test_decode_ctypes_string_handles_null_terminated_bytes():
    from sentry_bringup.hikrobot_camera_node import _decode_ctypes_string

    assert _decode_ctypes_string(b'MV-CS016-10UC\x00ignored') == 'MV-CS016-10UC'


def test_prepend_unique_env_path(monkeypatch):
    from sentry_bringup.hikrobot_camera_node import _prepend_unique_env_path

    monkeypatch.setenv('LD_LIBRARY_PATH', '/existing')
    _prepend_unique_env_path('LD_LIBRARY_PATH', '/opt/MVS/lib/aarch64')
    _prepend_unique_env_path('LD_LIBRARY_PATH', '/opt/MVS/lib/aarch64')

    assert os.environ['LD_LIBRARY_PATH'].split(os.pathsep) == [
        '/opt/MVS/lib/aarch64',
        '/existing',
    ]


def test_load_mvs_sdk_sets_paths_and_imports_module(tmp_path, monkeypatch):
    module_path = tmp_path / 'MvCameraControl_class.py'
    module_path.write_text('SDK_MARKER = 1\n', encoding='utf-8')
    sys.modules.pop('MvCameraControl_class', None)
    monkeypatch.delenv('MVCAM_COMMON_RUNENV', raising=False)
    monkeypatch.delenv('LD_LIBRARY_PATH', raising=False)

    camera_node = importlib.import_module('sentry_bringup.hikrobot_camera_node')
    mod = camera_node._load_mvs_sdk(
        '/opt/MVS/lib',
        str(tmp_path),
        '/opt/MVS/lib/aarch64',
    )

    assert mod.SDK_MARKER == 1
    assert os.environ['MVCAM_COMMON_RUNENV'] == '/opt/MVS/lib'
    assert os.environ['LD_LIBRARY_PATH'].split(os.pathsep)[0] == (
        '/opt/MVS/lib/aarch64')
    assert sys.path[0] == str(tmp_path)
