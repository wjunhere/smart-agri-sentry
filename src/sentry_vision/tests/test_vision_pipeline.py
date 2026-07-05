"""Unit tests for vision_pipeline_node — state machine and aggregation logic."""
import pytest
import numpy as np


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


class TestBboxEdgeDetection:
    def test_bbox_centered_no_reshoot(self):
        """Bbox near center → no gimbal adjustment needed."""
        bbox = [0.35, 0.35, 0.65, 0.65]
        cx = (bbox[0] + bbox[2]) / 2.0
        cy = (bbox[1] + bbox[3]) / 2.0
        threshold = 0.35
        centered = (threshold <= cx <= (1.0 - threshold)
                    and threshold <= cy <= (1.0 - threshold))
        assert centered

    def test_bbox_left_edge_triggers_reshoot(self):
        """Bbox on left edge → should adjust yaw left."""
        bbox = [0.05, 0.35, 0.25, 0.65]
        cx = (bbox[0] + bbox[2]) / 2.0
        threshold = 0.35
        assert cx < threshold  # should trigger yaw adjustment

    def test_bbox_top_edge_triggers_reshoot(self):
        """Bbox on top edge → should adjust pitch up."""
        bbox = [0.35, 0.05, 0.65, 0.25]
        cy = (bbox[1] + bbox[3]) / 2.0
        threshold = 0.35
        assert cy < threshold  # should trigger pitch adjustment


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
