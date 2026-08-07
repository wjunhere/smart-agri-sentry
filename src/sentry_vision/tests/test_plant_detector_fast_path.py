"""Tests for the plant detector fast-path (single-frame strong hit)."""

import pytest
import rclpy
from unittest.mock import patch

from sentry_vision.plant_detector_node import PlantDetectorNode


@pytest.fixture(scope='module')
def ros_context():
    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.fixture
def node(ros_context):
    with patch.object(PlantDetectorNode, '_load_model'):
        n = PlantDetectorNode()
        yield n
        n.destroy_node()


def test_fast_path_bypasses_voting(node):
    """A strong single-frame hit reports immediately, no second frame needed."""
    detected, bbox, conf, area = node._vote(
        True, [0.1, 0.1, 0.5, 0.5], 0.8, 0.10, force=True)
    assert detected is True
    assert conf == pytest.approx(0.8)
    assert area == pytest.approx(0.10)


def test_single_weak_frame_waits_for_voting(node):
    """A lone weak hit must not report until the vote minimum is met."""
    detected, _, _, _ = node._vote(True, [0.1, 0.1, 0.5, 0.5], 0.4, 0.02)
    assert detected is False
    # second consecutive hit reaches vote_min=2 of window=3
    detected, _, conf, _ = node._vote(True, [0.1, 0.1, 0.5, 0.5], 0.4, 0.02)
    assert detected is True
    assert conf == pytest.approx(0.4)
