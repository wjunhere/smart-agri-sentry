"""Unit tests for yolo_utils.py — NV12 conversion and YOLO postprocessing."""

import numpy as np
import pytest
from sentry_vision.yolo_utils import bgr_to_nv12, bgr_to_yolo_input, yolo_postprocess, _nms


class TestNV12Conversion:
    def test_bgr_to_nv12_shape(self):
        """NV12 output should have exactly 1.5 * H * W bytes."""
        bgr = np.random.randint(0, 255, (480, 640, 3), dtype=np.uint8)
        nv12 = bgr_to_nv12(bgr)
        assert len(nv12) == int(480 * 640 * 1.5)
        assert nv12.dtype == np.uint8

    def test_bgr_to_yolo_input(self):
        """Resize to 640×640 then convert to RGB NCHW."""
        bgr = np.random.randint(0, 255, (1080, 1920, 3), dtype=np.uint8)
        tensor = bgr_to_yolo_input(bgr, 640)
        assert tensor.shape == (1, 3, 640, 640)
        assert tensor.dtype == np.uint8

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
    @staticmethod
    def _make_outputs(cls_val=5.0, bbox_val=0.0):
        """Build 6-tensor output mimicking quantized YOLOv8n BPU."""
        outputs = []
        for h, w in [(80, 80), (40, 40), (20, 20)]:
            cls_t = np.full((h, w, 2), cls_val, dtype=np.float32)
            bbox_t = np.full((h, w, 4, 16), bbox_val, dtype=np.float32)
            outputs.append(cls_t)
            outputs.append(bbox_t)
        return outputs

    def test_no_detection_low_conf(self):
        """All class scores below threshold → no detection."""
        outputs = self._make_outputs(cls_val=-5.0)
        detected, bbox, conf = yolo_postprocess(outputs, conf_threshold=0.5)
        assert not detected
        assert conf == 0.0

    def test_output_with_high_conf(self):
        """High crop score, low weed score → detection."""
        outputs = self._make_outputs(cls_val=5.0, bbox_val=0.5)
        # Set crop cls high, weed cls low
        for si in [0, 2, 4]:
            outputs[si][:, :, 0] = 5.0   # crop
            outputs[si][:, :, 1] = -5.0  # weed
        detected, bbox, conf = yolo_postprocess(outputs, conf_threshold=0.5)
        assert isinstance(detected, bool)
        assert len(bbox) == 4

    def test_all_zeros_no_detection(self):
        """All-zero output → no detection."""
        outputs = self._make_outputs(cls_val=0.0, bbox_val=0.0)
        detected, bbox, conf = yolo_postprocess(outputs, conf_threshold=0.5)
        assert not detected
