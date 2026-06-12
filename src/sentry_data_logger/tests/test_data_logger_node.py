import os
import shutil
import sys
import tempfile
import time
import pytest
import rclpy
from unittest.mock import MagicMock

from sentry_data_logger.bag_writer import BagWriter
from sentry_data_logger.data_logger_node import DataLoggerNode, ALERT_CRITICAL
from sentry_interfaces.msg import FusionResult, ForecastAlert


@pytest.fixture(scope='module')
def ros_context():
    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.fixture
def tmp_bag_dir():
    path = tempfile.mkdtemp()
    yield path
    shutil.rmtree(path, ignore_errors=True)


def test_bag_writer_opens_and_closes(tmp_bag_dir):
    writer = BagWriter(tmp_bag_dir, split_duration_sec=1)
    writer.open()
    assert writer._current_dir is not None
    writer.close()


def test_bag_writer_json_fallback_on_missing_rosbag(tmp_bag_dir, monkeypatch):
    monkeypatch.setitem(sys.modules, 'rosbag2_py', None)
    writer = BagWriter(tmp_bag_dir)
    writer.open()
    assert writer._json_fallback is True
    writer.close()


@pytest.fixture
def node(ros_context, tmp_bag_dir):
    n = DataLoggerNode()
    n.writer = MagicMock()
    n.writer._current_dir = tmp_bag_dir
    yield n
    n.destroy_node()


def test_normal_fusion_writes_only(node):
    msg = FusionResult()
    msg.header.stamp = node.get_clock().now().to_msg()
    msg.alert_level = 1  # SUSPICION
    node._on_msg('/fusion/diagnosis', msg)
    assert node.writer.write.called
    assert not node.writer.snapshot_critical.called


def test_critical_fusion_triggers_snapshot(node):
    msg = FusionResult()
    msg.header.stamp = node.get_clock().now().to_msg()
    msg.alert_level = ALERT_CRITICAL
    node._on_msg('/fusion/diagnosis', msg)
    assert node.writer.snapshot_critical.called


def test_duplicate_critical_not_double_snapshot(node):
    msg = FusionResult()
    msg.header.stamp = node.get_clock().now().to_msg()
    msg.alert_level = ALERT_CRITICAL
    node._on_msg('/fusion/diagnosis', msg)
    node._on_msg('/fusion/diagnosis', msg)
    assert node.writer.snapshot_critical.call_count == 1
