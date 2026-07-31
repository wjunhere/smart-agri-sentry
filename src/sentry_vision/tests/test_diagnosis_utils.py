"""Tests for vision diagnosis utilities (ROS2-independent)."""

import os
import pytest

from sentry_vision.diagnosis_utils import get_labels, resolve_model_path, get_input_format


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
        # Create a fake quantized model file at the new default path so
        # resolution succeeds.
        rel = 'models/quantization/tomato_mobilenetv3_v5_output/tomato_mobilenetv3_v5_bayese_224x224_nv12.bin'
        model_file = tmp_path / rel
        model_file.parent.mkdir(parents=True)
        model_file.write_text('fake')

        monkeypatch.chdir(tmp_path)
        path = resolve_model_path('tomato', '', 224)
        assert os.path.isabs(path)
        assert path.endswith('tomato_mobilenetv3_v5_bayese_224x224_nv12.bin')


class TestGetLabels:
    def test_tomato_labels(self):
        labels = get_labels('tomato')
        assert len(labels) == 7
        assert 'healthy' in labels
        assert 'late_blight' in labels
        assert labels[0] == 'late_blight'

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
        assert len(labels) == 7
        assert labels[0] == 'late_blight'


class TestGetInputFormat:
    def test_tomato_is_nv12(self):
        assert get_input_format('tomato') == 'nv12'

    def test_wheat_is_nv12(self):
        assert get_input_format('wheat') == 'nv12'

    def test_strawberry_is_rgb(self):
        assert get_input_format('strawberry') == 'rgb'

    def test_unknown_falls_back_to_nv12(self):
        assert get_input_format('unknown') == 'nv12'
