import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from sentry_interfaces.msg import Diagnosis
from cv_bridge import CvBridge
import numpy as np
import cv2

from .diagnosis_utils import get_labels, resolve_model_path, get_input_format


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

        self.declare_parameter('healthy_threshold', 0.15)

        self.crop_type = self.get_parameter('crop_type').value
        self.input_size = self.get_parameter('input_size').value
        model_path = self.get_parameter('model_path').value
        self.healthy_threshold = self.get_parameter('healthy_threshold').value

        model_path = resolve_model_path(self.crop_type, model_path, self.input_size)
        self.labels = get_labels(self.crop_type)
        self.input_format = get_input_format(self.crop_type)

        self.get_logger().info(
            f'Loading BPU model: {model_path} (format={self.input_format})')
        from hobot_dnn import pyeasy_dnn as dnn
        self._dnn = dnn
        self.model = dnn.load(model_path)[0]
        self.get_logger().info(
            f'BPU model loaded. name={self.model.name} '
            f'inputs={len(self.model.inputs)} outputs={len(self.model.outputs)}')

        self.bridge = CvBridge()
        self.sub = self.create_subscription(
            Image, '/sentry/camera/image_raw', self.on_image, 1)
        self.pub = self.create_publisher(Diagnosis, '/vision/diagnosis', 10)
        self.get_logger().info('Vision diagnosis node ready (BPU) '
                               f'healthy_threshold={self.healthy_threshold}')

    def on_image(self, msg: Image):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'CV bridge error: {e}')
            return

        input_tensor = self.preprocess(cv_image)
        outputs = self.model.forward([input_tensor])
        output = outputs[0].buffer.reshape(-1)

        probs = self._softmax(output)
        healthy_idx = self.labels.index('healthy') if 'healthy' in self.labels else -1
        healthy_prob = float(probs[healthy_idx]) if healthy_idx >= 0 else 0.0

        # Healthy threshold: if healthy class probability >= threshold, predict healthy
        if healthy_idx >= 0 and healthy_prob >= self.healthy_threshold:
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

    def preprocess(self, image: np.ndarray) -> np.ndarray:
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
