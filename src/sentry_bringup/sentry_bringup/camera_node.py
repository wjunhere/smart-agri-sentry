import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np


def preprocess_image(image: np.ndarray, target_size=(224, 224)) -> np.ndarray:
    """Resize and normalize image for MobileNetV2 input."""
    resized = cv2.resize(image, target_size)
    rgb = cv2.cvtColor(resized, cv2.COLOR_BGR2RGB)
    normalized = rgb.astype(np.float32) / 127.5 - 1.0
    return np.expand_dims(normalized, axis=0)


class CameraNode(Node):
    def __init__(self):
        super().__init__('camera_node')
        self.declare_parameter('device_id', 0)
        self.declare_parameter('fps', 2.0)

        dev_id = self.get_parameter('device_id').value
        fps = self.get_parameter('fps').value

        self.cap = cv2.VideoCapture(dev_id)
        if not self.cap.isOpened():
            self.get_logger().error(f'Failed to open camera {dev_id}')
        else:
            self.get_logger().info(f'Camera opened: {dev_id}')
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
            self.cap.set(cv2.CAP_PROP_FPS, fps)

        self.bridge = CvBridge()
        self.pub = self.create_publisher(Image, '/sentry/camera/image_raw', 10)
        self.timer = self.create_timer(1.0 / fps, self.capture)
        self.frame_count = 0

    def capture(self):
        if not self.cap.isOpened():
            return
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn('Camera capture failed')
            return
        self.frame_count += 1
        msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'camera'
        self.pub.publish(msg)

    def destroy_node(self):
        self.cap.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
