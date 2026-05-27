import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from sentry_interfaces.msg import PlantDetection
from cv_bridge import CvBridge
import cv2
import numpy as np


class PlantDetectorNode(Node):
    """Lightweight plant detector using color-based segmentation.

    Phase 1 uses a simplified green-channel threshold approach.
    Replace with a trained TFLite model (plant_detector_nano.tflite)
    before competition.
    """

    def __init__(self):
        super().__init__('plant_detector_node')
        self.declare_parameter('confidence_threshold', 0.6)
        self.declare_parameter('min_area_ratio', 0.1)
        self.declare_parameter('use_simulation', False)

        self.confidence_threshold = self.get_parameter(
            'confidence_threshold').value
        self.min_area_ratio = self.get_parameter('min_area_ratio').value
        self.use_simulation = self.get_parameter('use_simulation').value

        self.bridge = CvBridge()
        self.sub = self.create_subscription(
            Image, '/sentry/camera/image_raw', self.on_image, 1)
        self.pub = self.create_publisher(
            PlantDetection, '/vision/plant_detected', 10)
        self.get_logger().info('Plant detector node ready')

    def on_image(self, msg: Image):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'CV bridge error: {e}')
            return

        if self.use_simulation:
            detected, bbox, confidence, area_ratio = self._simulate(cv_image)
        else:
            detected, bbox, confidence, area_ratio = self._detect(cv_image)

        out = PlantDetection()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = 'camera'
        out.detected = detected
        out.bbox = bbox
        out.confidence = confidence
        out.area_ratio = area_ratio
        self.pub.publish(out)

    def _detect(self, image: np.ndarray):
        """Color-based plant segmentation.

        Returns (detected, bbox[4], confidence, area_ratio)
        """
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        # Green hue range in HSV
        lower_green = np.array([35, 40, 40])
        upper_green = np.array([85, 255, 255])
        mask = cv2.inRange(hsv, lower_green, upper_green)

        # Morphological cleanup
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        total_pixels = mask.shape[0] * mask.shape[1]
        plant_pixels = cv2.countNonZero(mask)
        area_ratio = plant_pixels / total_pixels

        if area_ratio < self.min_area_ratio:
            return False, [0.0, 0.0, 0.0, 0.0], 0.0, area_ratio

        # Find largest contour for bbox
        contours, _ = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return False, [0.0, 0.0, 0.0, 0.0], 0.0, area_ratio

        largest = max(contours, key=cv2.contourArea)
        x, y, w, h = cv2.boundingRect(largest)
        h_img, w_img = image.shape[:2]

        bbox = [
            float(x) / w_img,
            float(y) / h_img,
            float(x + w) / w_img,
            float(y + h) / h_img,
        ]

        # Confidence scales with area_ratio relative to min threshold
        confidence = min(1.0, area_ratio / self.min_area_ratio * 0.8)
        detected = confidence >= self.confidence_threshold

        return detected, bbox, confidence, area_ratio

    def _simulate(self, image: np.ndarray):
        """Simulation mode: always detect a centered plant."""
        return True, [0.3, 0.3, 0.7, 0.7], 0.95, 0.25


def main(args=None):
    rclpy.init(args=args)
    node = PlantDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
