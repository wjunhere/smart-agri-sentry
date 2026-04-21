import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from sentry_interfaces.msg import AiDiagnosis
from cv_bridge import CvBridge
import numpy as np
import tflite_runtime.interpreter as tflite
import os
import cv2


# 10-class tomato disease labels
LABELS = [
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


class AiInferenceNode(Node):
    def __init__(self):
        super().__init__('ai_inference_node')
        self.declare_parameter('model_path', 'models/finetuned_mobilenetv2_int8.tflite')
        self.declare_parameter('input_size', 224)

        model_path = self.get_parameter('model_path').value
        self.input_size = self.get_parameter('input_size').value

        # Resolve model path relative to workspace or absolute
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

        self.get_logger().info(f'Loading model: {model_path}')
        self.interpreter = tflite.Interpreter(model_path=model_path)
        self.interpreter.allocate_tensors()
        self.input_details = self.interpreter.get_input_details()
        self.output_details = self.interpreter.get_output_details()

        self.bridge = CvBridge()
        self.sub = self.create_subscription(Image, '/sentry/camera/image_raw', self.on_image, 1)
        self.pub = self.create_publisher(AiDiagnosis, '/sentry/ai/diagnosis', 10)
        self.get_logger().info('AI inference node ready')

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

        out_msg = AiDiagnosis()
        out_msg.header.stamp = self.get_clock().now().to_msg()
        out_msg.header.frame_id = 'camera'
        out_msg.disease_class = LABELS[class_idx]
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
    node = AiInferenceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
