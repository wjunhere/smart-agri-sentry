import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from sentry_interfaces.msg import Diagnosis
from cv_bridge import CvBridge
import numpy as np
import tflite_runtime.interpreter as tflite
import os
import cv2


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

LABEL_MAP = {
    'tomato': TOMATO_LABELS,
}


class VisionDiagnosisNode(Node):
    def __init__(self):
        super().__init__('vision_diagnosis_node')
        self.declare_parameter('crop_type', 'tomato')
        self.declare_parameter('model_path', '')
        self.declare_parameter('input_size', 224)

        self.crop_type = self.get_parameter('crop_type').value
        self.input_size = self.get_parameter('input_size').value
        model_path = self.get_parameter('model_path').value

        # Resolve model path: explicit > auto-pattern > fallback
        if not model_path:
            model_path = f'models/{self.crop_type}_mobilenetv2_int8.tflite'
        if not os.path.isabs(model_path):
            ws = os.environ.get('COLCON_PREFIX_PATH', os.getcwd())
            candidates = [
                os.path.join(ws, '..', '..', model_path),
                os.path.join(ws, model_path),
                model_path,
            ]
            for c in candidates:
                if os.path.exists(c):
                    model_path = c
                    break

        self.labels = LABEL_MAP.get(self.crop_type, TOMATO_LABELS)

        self.get_logger().info(f'Loading model: {model_path}')
        self.interpreter = tflite.Interpreter(model_path=model_path)
        self.interpreter.allocate_tensors()
        self.input_details = self.interpreter.get_input_details()
        self.output_details = self.interpreter.get_output_details()

        self.bridge = CvBridge()
        self.sub = self.create_subscription(
            Image, '/sentry/camera/image_raw', self.on_image, 1)
        self.pub = self.create_publisher(Diagnosis, '/vision/diagnosis', 10)
        self.get_logger().info('Vision diagnosis node ready')

    def on_image(self, msg: Image):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'CV bridge error: {e}')
            return

        resized = self.preprocess(cv_image)
        self.interpreter.set_tensor(self.input_details[0]['index'], resized)
        self.interpreter.invoke()
        output = self.interpreter.get_tensor(self.output_details[0]['index'])[0]

        # Softmax if logits
        exp_out = np.exp(output - np.max(output))
        probs = exp_out / np.sum(exp_out)

        class_idx = int(np.argmax(probs))
        confidence = float(probs[class_idx])

        out_msg = Diagnosis()
        out_msg.header.stamp = self.get_clock().now().to_msg()
        out_msg.header.frame_id = 'camera'
        out_msg.crop_type = self.crop_type
        out_msg.disease_class = self.labels[class_idx] if class_idx < len(self.labels) else 'unknown'
        out_msg.class_id = class_idx
        out_msg.confidence = confidence
        out_msg.probabilities = probs.tolist()
        self.pub.publish(out_msg)

    def preprocess(self, image: np.ndarray) -> np.ndarray:
        resized = cv2.resize(image, (self.input_size, self.input_size))
        rgb = cv2.cvtColor(resized, cv2.COLOR_BGR2RGB)
        normalized = rgb.astype(np.float32) / 127.5 - 1.0
        return np.expand_dims(normalized, axis=0)


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
