import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import Image
from std_msgs.msg import String
from sentry_interfaces.msg import Diagnosis, PlantDetection
from cv_bridge import CvBridge
import numpy as np
import cv2

from .diagnosis_utils import (
    get_labels, resolve_model_path, get_input_format, crop_letterbox,
)


def bgr_to_nv12(bgr: np.ndarray) -> np.ndarray:
    """Convert BGR uint8 (H, W, 3) to packed NV12 flat uint8."""
    h, w = bgr.shape[:2]
    yuv = cv2.cvtColor(bgr, cv2.COLOR_BGR2YUV_I420)
    y = yuv[:h, :w]
    u = yuv[h:h + h // 4, :w]
    v = yuv[h + h // 4:, :w]
    uv = np.empty((h // 2, w), dtype=np.uint8)
    uv[0::2, :] = u
    uv[1::2, :] = v
    return np.concatenate([y.reshape(-1), uv.reshape(-1)])


def bgr_to_rgb_nchw(bgr: np.ndarray) -> np.ndarray:
    """Convert BGR uint8 (H, W, 3) to RGB NCHW uint8 [1, 3, H, W]."""
    rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
    return np.transpose(rgb, (2, 0, 1))[np.newaxis, :, :, :].copy()


class VisionDiagnosisNode(Node):
    def __init__(self):
        super().__init__('vision_diagnosis_node')
        self.declare_parameter('crop_type', 'tomato')
        self.declare_parameter('model_path', '')
        self.declare_parameter('input_size', 224)

        self.declare_parameter('healthy_threshold', 0.0)

        self.crop_type = self.get_parameter('crop_type').value
        self.input_size = self.get_parameter('input_size').value
        model_path = self.get_parameter('model_path').value
        self.healthy_threshold = self.get_parameter('healthy_threshold').value
        self._model_path_param = model_path

        from hobot_dnn import pyeasy_dnn as dnn
        self._dnn = dnn
        self.model = None
        self._load_model(self.crop_type)

        self.bridge = CvBridge()
        self.sub = self.create_subscription(
            Image, '/sentry/camera/image_raw', self.on_image, 1)
        self.pub = self.create_publisher(Diagnosis, '/vision/diagnosis', 10)
        # Latest YOLO plant box from plant_detector_node; when fresh, the
        # classifier sees the crop (letterboxed) instead of the full frame.
        self._plant_bbox = None
        self._plant_stamp = 0.0
        self._plant_sub = self.create_subscription(
            PlantDetection, '/vision/plant_detected', self._on_plant, 10)
        # Frontend crop switching: gateway publishes the latched selection on
        # /vision/crop_type; reload the BPU model to match.
        self._crop_type_sub = self.create_subscription(
            String, '/vision/crop_type', self._on_crop_type,
            QoSProfile(depth=1,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.get_logger().info('Vision diagnosis node ready (BPU) '
                               f'healthy_threshold={self.healthy_threshold}')

    def _load_model(self, crop_type: str):
        model_path = resolve_model_path(
            crop_type, self._model_path_param, self.input_size)
        self.get_logger().info(
            f'Loading BPU model: {model_path} '
            f'(format={get_input_format(crop_type)})')
        model = self._dnn.load(model_path)[0]
        self.crop_type = crop_type
        self.labels = get_labels(crop_type)
        self.input_format = get_input_format(crop_type)
        self.model = model
        self.get_logger().info(
            f'BPU model loaded. name={self.model.name} '
            f'inputs={len(self.model.inputs)} outputs={len(self.model.outputs)}')

    def _on_crop_type(self, msg: String):
        crop = msg.data
        if crop and crop != self.crop_type:
            self.get_logger().info(f'Crop type switched: {self.crop_type} -> {crop}')
            try:
                self._load_model(crop)
            except Exception as exc:
                self.get_logger().error(
                    f'Failed to load model for {crop}, keeping {self.crop_type}: {exc}')

    def _on_plant(self, msg: PlantDetection):
        if msg.detected and len(msg.bbox) == 4:
            self._plant_bbox = list(msg.bbox)
            self._plant_stamp = self.get_clock().now().nanoseconds * 1e-9
        else:
            self._plant_bbox = None

    def _fresh_bbox(self):
        if self._plant_bbox is None:
            return None
        now = self.get_clock().now().nanoseconds * 1e-9
        if now - self._plant_stamp > 1.0:
            return None
        return self._plant_bbox

    def on_image(self, msg: Image):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'CV bridge error: {e}')
            return

        input_tensor = self.preprocess(cv_image, bbox=self._fresh_bbox())
        outputs = self.model.forward([input_tensor])
        output = outputs[0].buffer.reshape(-1)

        probs = self._softmax(output)
        healthy_idx = self.labels.index('healthy') if 'healthy' in self.labels else -1
        healthy_prob = float(probs[healthy_idx]) if healthy_idx >= 0 else 0.0

        # Healthy threshold (0 = disabled): if healthy class probability
        # exceeds threshold, predict healthy; otherwise plain argmax.
        if (healthy_idx >= 0 and self.healthy_threshold > 0
                and healthy_prob >= self.healthy_threshold):
            class_idx = healthy_idx
        else:
            class_idx = int(np.argmax(probs))
        confidence = float(probs[class_idx])

        out_msg = Diagnosis()
        out_msg.header.stamp = self.get_clock().now().to_msg()
        out_msg.header.frame_id = 'camera'
        out_msg.crop_type = self.crop_type
        out_msg.disease_class = (
            self.labels[class_idx] if class_idx < len(self.labels) else 'unknown')
        out_msg.class_id = class_idx
        out_msg.confidence = confidence
        out_msg.probabilities = probs.tolist()
        self.pub.publish(out_msg)

    def preprocess(self, image: np.ndarray, bbox=None) -> np.ndarray:
        if bbox is not None:
            resized = crop_letterbox(image, bbox, size=self.input_size)
        else:
            resized = cv2.resize(image, (self.input_size, self.input_size))
        if self.input_format == 'nv12':
            return bgr_to_nv12(resized)
        else:
            return bgr_to_rgb_nchw(resized)

    @staticmethod
    def _softmax(x: np.ndarray) -> np.ndarray:
        x = x - np.max(x)
        e = np.exp(x)
        return e / np.sum(e)


def main(args=None):
    rclpy.init(args=args)
    node = VisionDiagnosisNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
