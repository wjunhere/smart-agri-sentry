"""ROS2-independent utilities for vision diagnosis node."""

import os


# 10-class tomato disease labels
TOMATO_LABELS = [
    'bacterial_spot',
    'early_blight',
    'healthy',
    'late_blight',
    'leaf_mold',
    'septoria_leaf_spot',
    'spider_mites_two-spotted_spider_mite',
    'target_spot',
    'tomato_mosaic_virus',
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
    """Resolve the absolute path to the inference model.

    Board-side models are .bin files converted from ONNX. Explicit .tflite
    paths are rewritten to .bin with a warning.
    """
    if not model_path:
        model_path = f'models/{crop_type}_mobilenetv3.bin'

    if model_path.endswith('.tflite'):
        model_path = model_path[:-7] + '.bin'

    if os.path.isabs(model_path):
        return model_path

    candidates = []
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
