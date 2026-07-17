"""Tests for Hikrobot camera node helpers."""

import importlib
import os
import sys
import types
from pathlib import Path
from unittest import mock

import pytest
import numpy as np

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
    modules['cv2'].LUT = mock.MagicMock(side_effect=lambda image, lut: image)

    for name, mod in modules.items():
        sys.modules[name] = mod

    yield

    for name in modules:
        sys.modules.pop(name, None)
    sys.modules.pop('sentry_bringup.hikrobot_camera_node', None)


def test_default_topic_matches_vision_pipeline():
    from sentry_bringup.hikrobot_camera_node import DEFAULT_IMAGE_TOPIC

    assert DEFAULT_IMAGE_TOPIC == '/sentry/camera/image_raw'


def test_hikrobot_is_the_default_launch_backend():
    repo_root = Path(__file__).parents[2]
    launch_source = (repo_root / 'sentry_bringup' / 'launch' /
                     'sentry_v2.launch.py').read_text(encoding='utf-8')
    start_script = (repo_root.parents[0] / 'scripts' / 'rdk' /
                    'start_robot_stack.sh').read_text(encoding='utf-8')

    assert "DeclareLaunchArgument('camera_backend', default_value='hikrobot')" in launch_source
    assert 'CAMERA_BACKEND="${CAMERA_BACKEND:-hikrobot}"' in start_script


def test_hikrobot_launch_uses_adaptive_exposure_for_low_light():
    repo_root = Path(__file__).parents[2]
    launch_source = (repo_root / 'sentry_bringup' / 'launch' /
                     'sentry_v2.launch.py').read_text(encoding='utf-8')

    assert "'exposure_auto': True" in launch_source
    assert "'gain_auto': True" in launch_source
    assert "'auto_exposure_max_us': 40000.0" in launch_source
    assert "'auto_gain_max': 12.0" in launch_source
    assert "'enable_image_enhancement': True" in launch_source
    assert "'gamma': 2.0" in launch_source


def test_gamma_lut_brightens_dark_pixels():
    from sentry_bringup.hikrobot_camera_node import HikrobotCameraNode

    lut = HikrobotCameraNode._build_gamma_lut(2.0)

    assert int(lut[32]) > 32
    assert int(lut[255]) == 255


def test_image_enhancement_applies_gamma_lut_when_enabled():
    import cv2
    from sentry_bringup.hikrobot_camera_node import HikrobotCameraNode

    node = HikrobotCameraNode.__new__(HikrobotCameraNode)
    node.enable_image_enhancement = True
    node.gamma_lut = np.arange(256, dtype=np.uint8)
    frame = np.array([[[12, 24, 36]]], dtype=np.uint8)

    assert node._apply_image_enhancement(frame) is frame
    cv2.LUT.assert_called_once_with(frame, node.gamma_lut)


def test_optional_enum_writes_auto_mode_off():
    from sentry_bringup.hikrobot_camera_node import HikrobotCameraNode

    node = HikrobotCameraNode.__new__(HikrobotCameraNode)
    node.cam = mock.MagicMock()
    node.cam.MV_CC_SetEnumValue.return_value = 0
    node.get_logger = mock.MagicMock()

    node._set_optional_enum('ExposureAuto', 0)

    node.cam.MV_CC_SetEnumValue.assert_called_once_with('ExposureAuto', 0)


def test_read_optional_float_returns_camera_hardware_value():
    from sentry_bringup.hikrobot_camera_node import HikrobotCameraNode

    node = HikrobotCameraNode.__new__(HikrobotCameraNode)
    node.mvs = types.SimpleNamespace(
        MVCC_FLOATVALUE=lambda: types.SimpleNamespace(fCurValue=0.0))
    node.cam = mock.MagicMock()

    def set_camera_value(name, value):
        value.fCurValue = 23456.0
        return 0

    node.cam.MV_CC_GetFloatValue.side_effect = set_camera_value

    assert node._read_optional_float('ExposureTime') == 23456.0


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
