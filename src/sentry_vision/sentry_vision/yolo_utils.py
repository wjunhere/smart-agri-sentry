"""YOLOv8n utilities: NV12 preprocessing and multi-output postprocessing.

The quantized YOLOv8n BPU model outputs 6 tensors (2 per stride level):
  - Class scores: (1, H, W, C) per stride (C=2 for crop/weed, C=1 for plant)
  - Bbox DFL features: (1, H, W, 64) per stride  (64 = 4 coords * 16 bins)
"""

import cv2
import numpy as np


def bgr_to_yolo_input(bgr: np.ndarray, size: int = 640) -> tuple:
    """Letterbox BGR image to size×size and convert to flat NV12 uint8.

    Matches the training-time letterbox (aspect-preserving resize + gray
    padding) instead of a distorting stretch — small/thin objects keep
    their shape. Returns (nv12, scale, pad_x, pad_y); the meta is needed
    by yolo_box_to_image() to map detections back to the original frame.
    """
    h, w = bgr.shape[:2]
    scale = min(size / w, size / h)
    nw, nh = int(round(w * scale)), int(round(h * scale))
    resized = cv2.resize(bgr, (nw, nh))
    canvas = np.full((size, size, 3), 114, dtype=np.uint8)
    pad_x, pad_y = (size - nw) // 2, (size - nh) // 2
    canvas[pad_y:pad_y + nh, pad_x:pad_x + nw] = resized
    return bgr_to_nv12(canvas), scale, pad_x, pad_y


def yolo_box_to_image(bbox, scale: float, pad_x: int, pad_y: int,
                      orig_w: int, orig_h: int, size: int = 640) -> list:
    """Map a normalized letterbox-space bbox back to normalized coords of
    the original image (clamped to [0, 1])."""
    x0 = (bbox[0] * size - pad_x) / scale / orig_w
    y0 = (bbox[1] * size - pad_y) / scale / orig_h
    x1 = (bbox[2] * size - pad_x) / scale / orig_w
    y1 = (bbox[3] * size - pad_y) / scale / orig_h
    return [min(1.0, max(0.0, v)) for v in (x0, y0, x1, y1)]


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
_DFL_BINS = np.arange(REG_MAX, dtype=np.float32)


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

        cls_out = outputs[si * 2]       # (1, H, W, C)
        bbox_out = outputs[si * 2 + 1]  # (1, H, W, 64)

        cls_out = cls_out.reshape(h, w, -1)

        # Class scores: sigmoid
        cls_probs = 1.0 / (1.0 + np.exp(-cls_out))  # (H, W, C)

        # Take max confidence across classes (works for both the old
        # 2-class crop/weed model and the new single-class plant model)
        scores = cls_probs.max(axis=-1)  # (H, W)

        # Confidence-first filtering: DFL-decode only promising anchors.
        # >99% of the 8400 anchors are background; decoding them all costs
        # ~65 ms per frame in numpy, decoding only survivors costs <5 ms.
        mask = scores > conf_threshold
        if not mask.any():
            continue
        ys, xs = np.nonzero(mask)
        sel_scores = scores[ys, xs]

        # Bbox DFL on selected anchors only: softmax on last dim → integral
        bbox_sel = bbox_out.reshape(h, w, 4, REG_MAX)[ys, xs]  # (N, 4, 16)
        bbox_sel = bbox_sel - bbox_sel.max(axis=-1, keepdims=True)
        bbox_exp = np.exp(bbox_sel)
        bbox_soft = bbox_exp / bbox_exp.sum(axis=-1, keepdims=True)
        offsets = np.sum(bbox_soft * _DFL_BINS, axis=-1)  # (N, 4)

        # Decode: lt_rb offsets to x1,y1,x2,y2 normalized
        x1 = np.clip((xs - offsets[:, 0]) * stride / input_size, 0.0, 1.0)
        y1 = np.clip((ys - offsets[:, 1]) * stride / input_size, 0.0, 1.0)
        x2 = np.clip((xs + offsets[:, 2]) * stride / input_size, 0.0, 1.0)
        y2 = np.clip((ys + offsets[:, 3]) * stride / input_size, 0.0, 1.0)

        all_boxes.append(np.stack([x1, y1, x2, y2], axis=1))
        all_scores.append(sel_scores)

    if not all_boxes:
        return False, [0.0, 0.0, 0.0, 0.0], 0.0

    boxes = np.concatenate(all_boxes, axis=0)
    scores = np.concatenate(all_scores, axis=0)

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
