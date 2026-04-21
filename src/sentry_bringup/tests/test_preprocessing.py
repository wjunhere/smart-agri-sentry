import pytest
import numpy as np
from sentry_bringup.camera_node import preprocess_image


def test_preprocess_shape():
    img = np.zeros((480, 640, 3), dtype=np.uint8)
    out = preprocess_image(img, target_size=(224, 224))
    assert out.shape == (1, 224, 224, 3)
    assert out.dtype == np.float32


def test_preprocess_normalization():
    img = np.full((480, 640, 3), 128, dtype=np.uint8)
    out = preprocess_image(img, target_size=(224, 224))
    assert out.min() >= -1.0
    assert out.max() <= 1.0
