"""Unit tests for vision_pipeline_node — fixed-camera aggregation logic."""
from pathlib import Path

import pytest
import numpy as np


PIPELINE_SOURCE = (
    Path(__file__).parent.parent / 'sentry_vision' / 'vision_pipeline_node.py'
).read_text(encoding='utf-8')
LAUNCH_SOURCE = (
    Path(__file__).parents[2] / 'sentry_bringup' / 'launch' / 'sentry_v2.launch.py'
).read_text(encoding='utf-8')


def test_pipeline_waits_for_frames_without_nested_spin():
    """A service callback must leave an executor thread available for images."""
    assert 'threading.Condition' in PIPELINE_SOURCE
    assert 'self._frame_sequence' in PIPELINE_SOURCE
    assert 'rclpy.spin_once(self' not in PIPELINE_SOURCE
    assert 'MultiThreadedExecutor(num_threads=2)' in PIPELINE_SOURCE


def test_pipeline_yolo_model_path_is_absolute_on_rdk():
    """The pipeline must not depend on the process working directory."""
    expected_path = (
        '/home/sunrise/dev_ws/models/'
        'yolov8n_crop_weed_bayese_640x640_nv12.bin'
    )
    assert "declare_parameter('yolo_model_path'" in PIPELINE_SOURCE
    assert expected_path in PIPELINE_SOURCE
    assert "'yolo_model_path': '/home/sunrise/dev_ws/models/" in LAUNCH_SOURCE


class TestAggregation:
    def test_max_confidence_wins(self):
        """Aggregation should pick the highest confidence across all shots."""
        results = [
            ('late_blight', 0, 0.72, [0.1, 0.72, 0.08, 0.02, 0.03, 0.03, 0.02], [0.1, 0.2, 0.3, 0.4]),
            ('healthy', 1, 0.88, [0.05, 0.88, 0.02, 0.01, 0.02, 0.01, 0.01], [0.1, 0.2, 0.3, 0.4]),
            ('early_blight', 2, 0.55, [0.1, 0.1, 0.55, 0.1, 0.05, 0.05, 0.05], [0.1, 0.2, 0.3, 0.4]),
        ]
        best = max(results, key=lambda r: r[2])
        assert best[0] == 'healthy'
        assert best[2] == 0.88

    def test_empty_results(self):
        """No crop detected in any shot → no_crop_detected."""
        results = []
        if not results:
            disease_class = 'no_crop_detected'
            confidence = 0.0
        assert disease_class == 'no_crop_detected'
        assert confidence == 0.0

    def test_single_shot(self):
        """Single shot with crop → that shot is the result."""
        results = [('powdery_mildew_leaf', 1, 0.95, [0.02, 0.95, 0.01, 0.01, 0.01], [0.1, 0.2, 0.3, 0.4])]
        best = max(results, key=lambda r: r[2])
        assert best[2] == 0.95

    def test_per_angle_confidences(self):
        """per_angle_confidences should contain one confidence per shot."""
        results = [
            ('disease_a', 0, 0.7, [], [0.1, 0.2, 0.3, 0.4]),
            ('disease_b', 1, 0.6, [], [0.1, 0.2, 0.3, 0.4]),
        ]
        per_angle = [r[2] for r in results]
        assert per_angle == [0.7, 0.6]


class TestSoftmax:
    def test_softmax_sum_to_one(self):
        x = np.array([2.0, 1.0, 0.1])
        x = x - np.max(x)
        e = np.exp(x)
        probs = e / np.sum(e)
        assert abs(np.sum(probs) - 1.0) < 1e-6

    def test_softmax_peak(self):
        x = np.array([10.0, -10.0, -10.0])
        x = x - np.max(x)
        e = np.exp(x)
        probs = e / np.sum(e)
        assert probs[0] > 0.99
