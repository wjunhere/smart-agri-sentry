"""YOLOv8n utilities: NV12 preprocessing and postprocessing (decode + NMS)."""

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


def yolo_postprocess(
    output: np.ndarray,
    input_size: int = 640,
    conf_threshold: float = 0.5,
    iou_threshold: float = 0.45,
) -> tuple:
    """Decode YOLOv8n raw output and apply NMS.

    Args:
        output: raw BPU output tensor, shape [1, 6, 8400] or [6, 8400].
        input_size: model input resolution (square).
        conf_threshold: minimum class confidence.
        iou_threshold: NMS IoU threshold.

    Returns:
        (detected: bool, bbox: [xmin, ymin, xmax, ymax] normalized, confidence: float)
        bbox is empty list if no detection.
    """
    if output.ndim == 3:
        output = output[0]  # [1, 6, 8400] -> [6, 8400]

    # Split: [cx, cy, w, h, cls0, cls1] x 8400
    bbox_raw = output[:4, :]    # [4, 8400]
    cls_raw = output[4:, :]     # [2, 8400]

    # Class confidence via softmax, take max across classes
    cls_max = cls_raw.max(axis=0) - np.log(np.sum(np.exp(cls_raw - cls_raw.max(axis=0)), axis=0) + 1e-8)
    # Numerically stable softmax-max per column:
    cls_exp = np.exp(cls_raw - cls_raw.max(axis=0, keepdims=True))
    cls_probs = cls_exp / cls_exp.sum(axis=0, keepdims=True)
    cls_conf = cls_probs.max(axis=0)  # [8400]
    cls_id = cls_probs.argmax(axis=0)  # [8400]

    # Filter by confidence
    mask = cls_conf > conf_threshold
    if not mask.any():
        return False, [0.0, 0.0, 0.0, 0.0], 0.0

    bbox_raw = bbox_raw[:, mask]
    cls_conf = cls_conf[mask]
    cls_id = cls_id[mask]

    # Decode boxes: cx, cy are sigmoid offsets from grid cell centers
    # Pre-compute grid cell offsets
    grid = []
    strides = [8, 16, 32]
    for si, stride in enumerate(strides):
        h = input_size // stride
        w = input_size // stride
        yv, xv = np.meshgrid(np.arange(h), np.arange(w), indexing='ij')
        grid.append(np.stack([xv.ravel(), yv.ravel()], axis=0))  # [2, h*w]
    anchors = np.concatenate(grid, axis=1).astype(np.float32)  # [2, 8400]

    # Rebuild strides array matching each detection index
    all_strides = []
    for stride in strides:
        h = input_size // stride
        w = input_size // stride
        all_strides.append(np.full(h * w, stride, dtype=np.float32))
    all_strides = np.concatenate(all_strides)

    # Filter anchors and strides to surviving detections
    anchors = anchors[:, mask]
    strides_f = all_strides[mask]

    # Apply sigmoid to cx, cy
    cx = (1.0 / (1.0 + np.exp(-bbox_raw[0, :]))) * 2.0 - 0.5
    cy = (1.0 / (1.0 + np.exp(-bbox_raw[1, :]))) * 2.0 - 0.5

    cx = (cx + anchors[0, :]) * strides_f / input_size
    cy = (cy + anchors[1, :]) * strides_f / input_size
    bw = bbox_raw[2, :] * bbox_raw[2, :] * 4.0 * strides_f / input_size
    bh = bbox_raw[3, :] * bbox_raw[3, :] * 4.0 * strides_f / input_size

    x1 = np.clip(cx - bw / 2.0, 0.0, 1.0)
    y1 = np.clip(cy - bh / 2.0, 0.0, 1.0)
    x2 = np.clip(cx + bw / 2.0, 0.0, 1.0)
    y2 = np.clip(cy + bh / 2.0, 0.0, 1.0)

    boxes = np.stack([x1, y1, x2, y2], axis=1)

    # NMS (only crop class = 0)
    crop_mask = cls_id == 0
    if not crop_mask.any():
        return False, [0.0, 0.0, 0.0, 0.0], 0.0

    keep = _nms(boxes[crop_mask], cls_conf[crop_mask], iou_threshold)
    if len(keep) == 0:
        return False, [0.0, 0.0, 0.0, 0.0], 0.0

    best_idx = np.argmax(cls_conf[crop_mask][keep])
    best_box = boxes[crop_mask][keep][best_idx]
    best_conf = float(cls_conf[crop_mask][keep][best_idx])

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
        iou = inter / (areas[i] + areas[order[1:]] - inter)

        remaining = np.where(iou <= iou_threshold)[0]
        order = order[remaining + 1]

    return np.array(keep, dtype=np.int32)
