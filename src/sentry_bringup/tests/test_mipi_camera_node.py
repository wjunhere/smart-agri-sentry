"""Tests for mipi_camera_node constants and topic alignment."""

import sys
import types
from unittest import mock

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
