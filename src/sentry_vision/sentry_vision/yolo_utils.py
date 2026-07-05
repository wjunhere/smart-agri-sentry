"""YOLOv8n utilities: NV12 preprocessing and multi-output postprocessing.

The quantized YOLOv8n BPU model outputs 6 tensors (2 per stride level):
  - Class scores: (1, H, W, 2) per stride
  - Bbox DFL features: (1, H, W, 64) per stride  (64 = 4 coords * 16 bins)
"""

import cv2
import numpy as np


def bgr_to_nv12_resized(bgr: np.ndarray, size: int = 640) -> np.ndarray:
    """Resize BGR image to size×size and convert to flat NV12 uint8 for BPU."""
    resized = cv2.resize(bgr, (size, size))
    return bgr_to_nv12(resized)


def bgr_to_nv12(bgr: np.ndarray) -> np.ndarray:
    """Convert BGR uint8 (H, W, 3) to flat packed NV12 uint8."""
    h, w = bgr.shape[:2]
    yuv = cv2.cvtColor(bgr, cv2.COLOR_BGR2YUV_I420)
    y = yuv[:h, :w]
    u = yuv[h:h + h // 4, :w]
    v = yuv[h + h // 4:, :w]
    uv = np.empty((h // 2, w), dtype=np.uint8)
    uv[0::2, :] = u
    uv[1::2, :] = v
    return np.concatenate([y.reshape(-1), uv.reshape(-1)])


# Strides and grid sizes for 640×640 input
STRIDES = [8, 16, 32]
GRID_SIZES = [(80, 80), (40, 40), (20, 20)]
REG_MAX = 16  # DFL regression bins per coordinate


def yolo_postprocess(
    outputs: list,
    input_size: int = 640,
    conf_threshold: float = 0.5,
    iou_threshold: float = 0.45,
) -> tuple:
    """Decode multi-output YOLOv8n BPU output and apply NMS.

    Args:
        outputs: list of 6 numpy arrays from model.forward() — each is the
                 .buffer of a model output tensor (already flat u8 → float32).
                 Order: [cls_s0, bbox_s0, cls_s1, bbox_s1, cls_s2, bbox_s2].
        input_size: model input resolution (square).
        conf_threshold: minimum class confidence.
        iou_threshold: NMS IoU threshold.

    Returns:
        (detected: bool, bbox: [xmin, ymin, xmax, ymax] normalized, confidence: float)
    """
    all_boxes = []
    all_scores = []

    for si, stride in enumerate(STRIDES):
        h, w = GRID_SIZES[si]

        cls_out = outputs[si * 2]       # (1, H, W, 2)
        bbox_out = outputs[si * 2 + 1]  # (1, H, W, 64)

        cls_out = cls_out.reshape(h, w, 2)
        bbox_out = bbox_out.reshape(h, w, 4, REG_MAX)

        # Class scores: sigmoid
        cls_probs = 1.0 / (1.0 + np.exp(-cls_out))  # (H, W, 2)

        # Bbox DFL: softmax on last dim → integral
        bbox_out = bbox_out - bbox_out.max(axis=-1, keepdims=True)
        bbox_exp = np.exp(bbox_out)
        bbox_soft = bbox_exp / bbox_exp.sum(axis=-1, keepdims=True)  # (H, W, 4, 16)
        dfl_bins = np.arange(REG_MAX, dtype=np.float32)
        offsets = np.sum(bbox_soft * dfl_bins, axis=-1)  # (H, W, 4)

        # Pre-compute anchor grid centers
        yv, xv = np.meshgrid(np.arange(h), np.arange(w), indexing='ij')
        anchors_x = xv.astype(np.float32)  # (H, W)
        anchors_y = yv.astype(np.float32)  # (H, W)

        # Decode: lt_rb offsets to x1,y1,x2,y2 normalized
        left = offsets[:, :, 0]
        top = offsets[:, :, 1]
        right = offsets[:, :, 2]
        bottom = offsets[:, :, 3]

        x1 = (anchors_x - left) * stride / input_size
        y1 = (anchors_y - top) * stride / input_size
        x2 = (anchors_x + right) * stride / input_size
        y2 = (anchors_y + bottom) * stride / input_size

        x1 = np.clip(x1, 0.0, 1.0)
        y1 = np.clip(y1, 0.0, 1.0)
        x2 = np.clip(x2, 0.0, 1.0)
        y2 = np.clip(y2, 0.0, 1.0)

        boxes = np.stack([x1.ravel(), y1.ravel(), x2.ravel(), y2.ravel()], axis=1)

        # Crop class (index 0) scores
        scores = cls_probs[:, :, 0].ravel()
        all_boxes.append(boxes)
        all_scores.append(scores)

    if not all_boxes:
        return False, [0.0, 0.0, 0.0, 0.0], 0.0

    boxes = np.concatenate(all_boxes, axis=0)
    scores = np.concatenate(all_scores, axis=0)

    # Filter by confidence
    mask = scores > conf_threshold
    if not mask.any():
        return False, [0.0, 0.0, 0.0, 0.0], 0.0

    boxes = boxes[mask]
    scores = scores[mask]

    # NMS
    keep = _nms(boxes, scores, iou_threshold)
    if len(keep) == 0:
        return False, [0.0, 0.0, 0.0, 0.0], 0.0

    best_idx = np.argmax(scores[keep])
    best_box = boxes[keep][best_idx]
    best_conf = float(scores[keep][best_idx])

    bbox = [float(best_box[0]), float(best_box[1]),
            float(best_box[2]), float(best_box[3])]
    return True, bbox, best_conf


def _nms(boxes: np.ndarray, scores: np.ndarray, iou_threshold: float) -> np.ndarray:
    """Simple NMS — return indices of kept boxes."""
    if len(boxes) == 0:
        return np.array([], dtype=np.int32)

    x1, y1, x2, y2 = boxes[:, 0], boxes[:, 1], boxes[:, 2], boxes[:, 3]
    areas = (x2 - x1) * (y2 - y1)
    order = scores.argsort()[::-1]

    keep = []
    while len(order) > 0:
        i = order[0]
        keep.append(i)

        xx1 = np.maximum(x1[i], x1[order[1:]])
        yy1 = np.maximum(y1[i], y1[order[1:]])
        xx2 = np.minimum(x2[i], x2[order[1:]])
        yy2 = np.minimum(y2[i], y2[order[1:]])

        w = np.maximum(0.0, xx2 - xx1)
        h = np.maximum(0.0, yy2 - yy1)
        inter = w * h
        iou = inter / np.maximum(areas[i] + areas[order[1:]] - inter, 1e-12)

        remaining = np.where(iou <= iou_threshold)[0]
        order = order[remaining + 1]

    return np.array(keep, dtype=np.int32)
