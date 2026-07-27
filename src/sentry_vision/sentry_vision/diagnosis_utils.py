"""ROS2-independent utilities for vision diagnosis node."""

import os


# Quantized BPU model paths (relative to workspace root)
_QUANTIZED_MODEL_PATHS = {
    'tomato': 'models/quantization/tomato_mobilenetv3_output/tomato_mobilenetv3_bayese_224x224_nv12.bin',
    'wheat': 'models/quantization/wheat_mobilenetv3_output/wheat_mobilenetv3_bayese_224x224_nv12.bin',
    'strawberry': 'models/quantization/strawberry_mobilenetv3_output/strawberry_mobilenetv3_bayese_224x224_rgb.bin',
}

# Input format per crop type (nv12 = packed NV12 uint8, rgb = NCHW uint8)
_INPUT_FORMATS = {
    'tomato': 'nv12',
    'wheat': 'nv12',
    'strawberry': 'rgb',
}


def get_input_format(crop_type: str) -> str:
    """Return the BPU input format for the given crop type."""
    return _INPUT_FORMATS.get(crop_type, 'nv12')


# 7-class tomato disease labels (model outputs 8; class 7 reserved)
TOMATO_LABELS = [
    'late_blight',
    'healthy',
    'early_blight',
    'bacterial_spot',
    'leaf_mold',
    'septoria_leaf_spot',
    'tomato_yellow_leaf_curl_virus',
]

# 5-class wheat disease labels
WHEAT_LABELS = [
    'healthy',
    'wheat_powdery_mildew',
    'wheat_scab',
    'wheat_stripe_rust',
    'wheat_yellow_dwarf',
]

# 8-class strawberry disease labels
STRAWBERRY_LABELS = [
    'leaf_spot',
    'powdery_mildew_leaf',
    'gray_mold',
    'angular_leaf_spot',
    'blossom_blight',
    'powdery_mildew_fruit',
    'anthracnose_fruit_rot',
    'healthy',
]

LABEL_MAP = {
    'tomato': TOMATO_LABELS,
    'wheat': WHEAT_LABELS,
    'strawberry': STRAWBERRY_LABELS,
}


def get_labels(crop_type: str) -> list:
    """Return disease label list for the given crop type.

    Falls back to tomato labels for unknown crop types.
    """
    return LABEL_MAP.get(crop_type, TOMATO_LABELS)


def resolve_model_path(crop_type: str, model_path: str, input_size: int) -> str:
    """Resolve the absolute path to the BPU inference model (.bin)."""
    if not model_path:
        model_path = _QUANTIZED_MODEL_PATHS.get(crop_type, _QUANTIZED_MODEL_PATHS['tomato'])

    if model_path.endswith('.tflite'):
        model_path = model_path[:-7] + '.bin'

    if os.path.isabs(model_path):
        return model_path

    candidates = []
    candidates.append(os.path.join('/home/sunrise/dev_ws', model_path))
    colcon_prefix = os.environ.get('COLCON_PREFIX_PATH')
    if colcon_prefix:
        # COLCON_PREFIX_PATH points to install/<pkg>; workspace root is two
        # levels up.
        candidates.append(os.path.join(colcon_prefix, '..', '..', model_path))
        candidates.append(os.path.join(colcon_prefix, model_path))
    candidates.append(model_path)

    for candidate in candidates:
        if os.path.exists(candidate):
            return os.path.abspath(candidate)

    return os.path.abspath(model_path)
