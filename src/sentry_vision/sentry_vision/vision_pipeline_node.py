"""Vision pipeline node: fixed-camera scan + YOLO detect + MobileNet classify + aggregate.

Provides a synchronous service /vision/pipeline/trigger that executes a complete
multi-frame scan-and-diagnose cycle without moving the gimbal. Loads BPU models
on-demand during scan and unloads them before returning.
"""
import time
import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from sentry_interfaces.msg import Diagnosis
from sentry_interfaces.srv import PipelineTrigger
from cv_bridge import CvBridge

from .yolo_utils import bgr_to_yolo_input, yolo_postprocess
from .diagnosis_utils import get_labels, resolve_model_path, get_input_format


# Scan limits
SCAN_TIMEOUT_SEC = 15.0


class VisionPipelineNode(Node):
    def __init__(self):
        super().__init__('vision_pipeline_node')
        self.declare_parameter('timeout_sec', SCAN_TIMEOUT_SEC)
        self.timeout_sec = self.get_parameter('timeout_sec').value
        self.bridge = CvBridge()
        self._latest_frame = None
        self._frame_received = False

        self.sub = self.create_subscription(
            Image, '/sentry/camera/image_raw', self._on_frame, 1)
        self.srv = self.create_service(
            PipelineTrigger, '/vision/pipeline/trigger', self.on_trigger)

        self.get_logger().info('Vision pipeline node ready')

    def _on_frame(self, msg: Image):
        self._latest_frame = msg
        self._frame_received = True

    # ── frame wait helper ───────────────────────────────────────────

    def _wait_for_frame(self, timeout: float = 2.0) -> Image | None:
        """Spin until a fresh frame arrives or timeout."""
        self._frame_received = False
        start = time.monotonic()
        while rclpy.ok() and not self._frame_received:
            rclpy.spin_once(self, timeout_sec=0.05)
            if time.monotonic() - start > timeout:
                self.get_logger().warn('Frame wait timeout')
                return None
        return self._latest_frame

    # ── model helpers ───────────────────────────────────────────────

    def _load_yolo(self):
        import os
        from hobot_dnn import pyeasy_dnn as dnn
        candidates = [
            os.path.join(os.getcwd(), 'models',
                         'yolov8n_crop_weed_bayese_640x640_nv12.bin'),
            os.path.join(os.path.dirname(__file__), '..', '..', '..',
                         'models', 'yolov8n_crop_weed_bayese_640x640_nv12.bin'),
        ]
        for c in candidates:
            if os.path.exists(c):
                self.get_logger().info(f'Loading YOLO: {c}')
                return dnn.load(c)[0]
        return None

    def _load_mobilenet(self, crop_type: str):
        from hobot_dnn import pyeasy_dnn as dnn
        path = resolve_model_path(crop_type, '', 224)
        self.get_logger().info(f'Loading MobileNet: {path}')
        return dnn.load(path)[0]

    def _unload_models(self):
        pass  # Python GC handles; models go out of scope after trigger returns

    # ── preprocessing for MobileNet ─────────────────────────────────

    def _preprocess_mobilenet(self, bgr, crop_type: str) -> np.ndarray:
        """Resize and convert to BPU input format matching the model."""
        from .vision_diagnosis_node import bgr_to_nv12, bgr_to_rgb_nchw
        import cv2

        resized = cv2.resize(bgr, (224, 224))
        fmt = get_input_format(crop_type)
        if fmt == 'nv12':
            return bgr_to_nv12(resized)
        else:
            return bgr_to_rgb_nchw(resized)

    # ── trigger service ─────────────────────────────────────────────

    def on_trigger(self, request, response):
        crop_type = request.crop_type
        max_shots = max(1, min(request.max_shots, 5))
        labels = get_labels(crop_type)

        self.get_logger().info(
            f'Pipeline triggered: crop={crop_type}, max_shots={max_shots}')

        t_start = time.monotonic()

        # 1. load models
        yolo = self._load_yolo()
        mobilenet = self._load_mobilenet(crop_type)
        if yolo is None or mobilenet is None:
            response.success = False
            self.get_logger().error('Failed to load BPU models')
            return response

        # 2. scan loop (fixed camera)
        results = []  # list of (disease_class, class_id, confidence, probs, bbox)

        for shot in range(max_shots):
            if time.monotonic() - t_start > self.timeout_sec:
                self.get_logger().warn('Scan timeout')
                break

            # Wait for frame
            frame_msg = self._wait_for_frame()
            if frame_msg is None:
                self.get_logger().warn(f'Shot {shot}: no frame, skipping')
                continue

            cv_image = self.bridge.imgmsg_to_cv2(frame_msg, desired_encoding='bgr8')

            # YOLO detect
            yolo_input = bgr_to_yolo_input(cv_image, 640)
            yolo_out = yolo.forward([yolo_input])

            detected, bbox, conf = yolo_postprocess(
                [o.buffer for o in yolo_out],
                input_size=640,
                conf_threshold=0.3,  # lower threshold during scan
            )

            if not detected:
                self.get_logger().info(f'Shot {shot}: no crop detected')
                break

            # MobileNet classify
            mb_input = self._preprocess_mobilenet(cv_image, crop_type)
            mb_out = mobilenet.forward([mb_input])
            mb_raw = mb_out[0].buffer.reshape(-1)

            probs = self._softmax(mb_raw)
            class_idx = int(np.argmax(probs))
            class_conf = float(probs[class_idx])
            disease_class = labels[class_idx] if class_idx < len(labels) else 'unknown'

            results.append((disease_class, class_idx, class_conf,
                            probs.tolist(), bbox))
            self.get_logger().info(
                f'Shot {shot}: {disease_class} conf={class_conf:.3f} '
                f'bbox=[{bbox[0]:.2f},{bbox[1]:.2f},{bbox[2]:.2f},{bbox[3]:.2f}]')

        # 3. aggregate
        diag = Diagnosis()
        diag.header.stamp = self.get_clock().now().to_msg()
        diag.header.frame_id = 'camera'
        diag.crop_type = crop_type

        if results:
            best = max(results, key=lambda r: r[2])
            diag.disease_class = best[0]
            diag.class_id = best[1]
            diag.confidence = best[2]
            diag.probabilities = best[3]
            per_angle = [r[2] for r in results]
        else:
            diag.disease_class = 'no_crop_detected'
            diag.class_id = 255
            diag.confidence = 0.0
            diag.probabilities = []
            per_angle = []

        diag.per_angle_confidences = per_angle

        response.success = True
        response.result = diag

        elapsed = time.monotonic() - t_start
        self.get_logger().info(
            f'Pipeline done: {len(results)} shots, '
            f'result={diag.disease_class}, elapsed={elapsed:.1f}s')
        return response

    @staticmethod
    def _softmax(x: np.ndarray) -> np.ndarray:
        x = x - np.max(x)
        e = np.exp(x)
        return e / np.sum(e)


def main(args=None):
    rclpy.init(args=args)
    node = VisionPipelineNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
