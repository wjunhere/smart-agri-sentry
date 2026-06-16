import pytest
import rclpy
from unittest.mock import patch

from sentry_sensors.uart_bridge_node import UartBridgeNode


@pytest.fixture(scope='module')
def ros_context():
    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.fixture
def node(ros_context):
    with patch('sentry_sensors.uart_bridge_node.serial.Serial'):
        n = UartBridgeNode()
        yield n
        n.destroy_node()


def test_servo_subscription_disabled_by_default(node):
    assert not hasattr(node, 'sub_servo')


def test_servo_subscription_enabled_when_configured(ros_context):
    with patch('sentry_sensors.uart_bridge_node.serial.Serial'):
        n = UartBridgeNode(parameter_overrides=[
            rclpy.parameter.Parameter(
                'forward_servo_cmd', rclpy.Parameter.Type.BOOL, True),
        ])
        assert hasattr(n, 'sub_servo')
        n.destroy_node()
