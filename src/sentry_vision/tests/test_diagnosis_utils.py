"""Tests for vision diagnosis utilities (ROS2-independent)."""

import os
import pytest

from sentry_vision.diagnosis_utils import get_labels, resolve_model_path


class TestResolveModelPath:
    def test_empty_path_uses_bin_default(self):
        path = resolve_model_path('tomato', '', 224)
        assert path.endswith('.bin')
        assert 'tomato' in path

    def test_empty_path_for_wheat(self):
        path = resolve_model_path('wheat', '', 224)
        assert path.endswith('.bin')
        assert 'wheat' in path

    def test_empty_path_for_strawberry(self):
        path = resolve_model_path('strawberry', '', 224)
        assert path.endswith('.bin')
        assert 'strawberry' in path

    def test_explicit_bin_path_preserved(self):
        explicit = os.path.abspath('/opt/model/tomato.bin')
        path = resolve_model_path('tomato', explicit, 224)
        assert path == explicit
        assert os.path.isabs(path)

    def test_explicit_tflite_rewritten_to_bin(self):
        path = resolve_model_path('tomato', 'models/tomato_mobilenetv2_int8.tflite', 224)
        assert path.endswith('.bin')
        assert not path.endswith('.tflite')

    def test_relative_path_resolved_with_search(self, tmp_path, monkeypatch):
        # Create a fake model file so resolution succeeds
        model_dir = tmp_path / 'models'
        model_dir.mkdir()
        (model_dir / 'tomato_mobilenetv3.bin').write_text('fake')

        monkeypatch.chdir(tmp_path)
        path = resolve_model_path('tomato', '', 224)
        assert os.path.isabs(path)
        assert path.endswith('tomato_mobilenetv3.bin')


class TestGetLabels:
    def test_tomato_labels(self):
        labels = get_labels('tomato')
        assert len(labels) == 10
        assert labels[2] == 'healthy'
        assert 'late_blight' in labels

    def test_wheat_labels(self):
        labels = get_labels('wheat')
        assert len(labels) == 5
        assert 'healthy' in labels
        assert 'wheat_powdery_mildew' in labels

    def test_strawberry_labels(self):
        labels = get_labels('strawberry')
        assert len(labels) == 8
        assert 'healthy' in labels
        assert 'gray_mold' in labels

    def test_unknown_crop_returns_tomato_labels(self):
        labels = get_labels('unknown')
        assert len(labels) == 10
        assert labels[0] == 'bacterial_spot'
