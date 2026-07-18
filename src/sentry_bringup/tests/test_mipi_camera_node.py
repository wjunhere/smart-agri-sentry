"""Tests for mipi_camera_node constants and topic alignment."""

import sys
import types
from unittest import mock

import numpy as np
import pytest


@pytest.fixture(scope='module', autouse=True)
def mock_ros2_and_hobot():
    """Mock ROS2 and hobot_vio so mipi_camera_node can be imported off-board."""
    modules = {
        'rclpy': types.ModuleType('rclpy'),
        'rclpy.node': types.ModuleType('rclpy.node'),
        'rclpy.clock': types.ModuleType('rclpy.clock'),
        'sensor_msgs': types.ModuleType('sensor_msgs'),
        'sensor_msgs.msg': types.ModuleType('sensor_msgs.msg'),
        'cv_bridge': types.ModuleType('cv_bridge'),
        'cv2': types.ModuleType('cv2'),
        'hobot_vio': types.ModuleType('hobot_vio'),
        'hobot_vio.libsrcampy': types.ModuleType('hobot_vio.libsrcampy'),
    }
    # Provide minimal stubs
    modules['rclpy'].init = mock.MagicMock()
    modules['rclpy'].shutdown = mock.MagicMock()
    modules['rclpy'].ok = mock.MagicMock(return_value=True)
    modules['rclpy'].Node = object
    modules['rclpy.node'].Node = object

    Image = type('Image', (), {})
    modules['sensor_msgs.msg'].Image = Image

    CvBridge = type('CvBridge', (), {'cv2_to_imgmsg': mock.MagicMock()})
    modules['cv_bridge'].CvBridge = CvBridge
    modules['cv2'].COLOR_YUV2BGR_NV12 = 90
    modules['cv2'].COLOR_YUV2BGR_NV21 = 92
    modules['cv2'].COLOR_BGR2HSV = 40
    modules['cv2'].COLOR_HSV2BGR = 54
    modules['cv2'].cvtColor = mock.MagicMock()
    modules['cv2'].fastNlMeansDenoisingColored = mock.MagicMock(
        side_effect=lambda image, *args, **kwargs: image)
    modules['cv2'].GaussianBlur = mock.MagicMock(
        side_effect=lambda image, *args, **kwargs: image)
    modules['cv2'].addWeighted = mock.MagicMock(
        side_effect=lambda image, *args, **kwargs: image)
    modules['cv2'].LUT = mock.MagicMock(
        side_effect=lambda image, *args, **kwargs: image)

    Camera = type('Camera', (), {
        'open_cam': mock.MagicMock(return_value=0),
        'get_img': mock.MagicMock(return_value=b''),
        'close_cam': mock.MagicMock(),
    })
    modules['hobot_vio.libsrcampy'].Camera = Camera

    for name, mod in modules.items():
        sys.modules[name] = mod

    yield

    for name in modules:
        sys.modules.pop(name, None)


def test_default_image_topic():
    from sentry_bringup.mipi_camera_node import DEFAULT_IMAGE_TOPIC
    assert DEFAULT_IMAGE_TOPIC == '/sentry/camera/image_raw'


def test_yuv_format_selects_nv21_conversion_code():
    import cv2
    from sentry_bringup.mipi_camera_node import MipiCameraNode

    node = MipiCameraNode.__new__(MipiCameraNode)
    node.yuv_format = 'nv21'

    assert node._yuv_to_bgr_code() == cv2.COLOR_YUV2BGR_NV21


def test_color_correction_applies_bgr_channel_gains():
    from sentry_bringup.mipi_camera_node import MipiCameraNode

    node = MipiCameraNode.__new__(MipiCameraNode)
    node.enable_color_correction = True
    node.blue_gain = 1.0
    node.green_gain = 0.5
    node.red_gain = 2.0
    frame = np.array([[[10, 100, 120]]], dtype=np.uint8)

    corrected = node._apply_color_correction(frame)

    assert corrected.tolist() == [[[10, 50, 240]]]


def test_gamma_lut_brightens_midtones_when_gamma_above_one():
    from sentry_bringup.mipi_camera_node import MipiCameraNode

    node = MipiCameraNode.__new__(MipiCameraNode)
    lut = node._build_gamma_lut(1.4)

    assert int(lut[64]) > 64
    assert int(lut[255]) == 255


def test_low_light_enhancement_runs_denoise_and_sharpen():
    import cv2
    from sentry_bringup.mipi_camera_node import MipiCameraNode

    node = MipiCameraNode.__new__(MipiCameraNode)
    node.enable_low_light_enhancement = True
    node.denoise_h = 5.0
    node.gamma = 1.0
    node.saturation_scale = 1.0
    node.sharpen_amount = 0.4
    frame = np.array([[[10, 20, 30]]], dtype=np.uint8)

    node._apply_low_light_enhancement(frame)

    assert cv2.fastNlMeansDenoisingColored.called
    assert cv2.GaussianBlur.called
    assert cv2.addWeighted.called
