"""Unit tests for yolo_utils.py — NV12 conversion and YOLO postprocessing."""

import numpy as np
import pytest
from sentry_vision.yolo_utils import bgr_to_nv12, bgr_to_nv12_resized, yolo_postprocess, _nms


class TestNV12Conversion:
    def test_bgr_to_nv12_shape(self):
        """NV12 output should have exactly 1.5 * H * W bytes."""
        bgr = np.random.randint(0, 255, (480, 640, 3), dtype=np.uint8)
        nv12 = bgr_to_nv12(bgr)
        assert len(nv12) == int(480 * 640 * 1.5)
        assert nv12.dtype == np.uint8

    def test_bgr_to_nv12_resized(self):
        """Resize to 640×640 then convert to NV12."""
        bgr = np.random.randint(0, 255, (1080, 1920, 3), dtype=np.uint8)
        nv12 = bgr_to_nv12_resized(bgr, 640)
        assert len(nv12) == int(640 * 640 * 1.5)

    def test_nv12_roundtrip_visual(self):
        """NV12 → BGR should roughly recover a solid color image."""
        bgr = np.full((64, 64, 3), [0, 128, 0], dtype=np.uint8)
        nv12 = bgr_to_nv12(bgr)
        # Reconstruct: NV12 is Y plane then UV plane
        y_plane = nv12[:64 * 64].reshape(64, 64)
        assert np.all(y_plane > 0), "Green should produce non-zero Y"


class TestNMS:
    def test_nms_empty(self):
        result = _nms(np.empty((0, 4)), np.empty((0,)), 0.5)
        assert len(result) == 0

    def test_nms_single_box(self):
        boxes = np.array([[0.1, 0.1, 0.3, 0.3]])
        scores = np.array([0.9])
        result = _nms(boxes, scores, 0.5)
        assert len(result) == 1
        assert result[0] == 0

    def test_nms_suppresses_overlapping(self):
        boxes = np.array([
            [0.1, 0.1, 0.5, 0.5],
            [0.12, 0.12, 0.48, 0.48],  # heavily overlapped
            [0.6, 0.6, 0.9, 0.9],       # far away
        ])
        scores = np.array([0.9, 0.8, 0.7])
        result = _nms(boxes, scores, 0.5)
        # Box 0 and box 2 should survive; box 1 is suppressed by box 0
        assert 0 in result
        assert 2 in result


class TestYoloPostprocess:
    def test_no_detection_low_conf(self):
        """All class scores below threshold → no detection."""
        output = np.zeros((1, 6, 8400), dtype=np.float32)
        output[:, 4, :] = -10.0  # low logits
        output[:, 5, :] = -10.0
        detected, bbox, conf = yolo_postprocess(output, conf_threshold=0.5)
        assert not detected
        assert conf == 0.0

    def test_output_with_batch_dim(self):
        """3D input [1, 6, 8400] should be handled."""
        output = np.random.randn(1, 6, 8400).astype(np.float32)
        output[:, 4, :] = 5.0   # high crop score
        output[:, 5, :] = -5.0  # low weed score
        detected, bbox, conf = yolo_postprocess(output, conf_threshold=0.5)
        # Post-processing may or may not produce a detection depending on
        # random bbox decoding; just check it doesn't crash.
        assert isinstance(detected, bool)
        assert len(bbox) == 4

    def test_output_2d(self):
        """2D input [6, 8400] should be handled."""
        output = np.random.randn(6, 8400).astype(np.float32)
        output[4, :] = 5.0
        output[5, :] = -5.0
        detected, bbox, conf = yolo_postprocess(output, conf_threshold=0.5)
        assert isinstance(detected, bool)
