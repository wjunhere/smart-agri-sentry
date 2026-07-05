"""Plant detector node using YOLOv8n BPU inference (crop/weed)."""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_srvs.srv import SetBool
from sentry_interfaces.msg import PlantDetection
from cv_bridge import CvBridge

from .yolo_utils import bgr_to_nv12_resized, yolo_postprocess


class PlantDetectorNode(Node):
    def __init__(self):
        super().__init__('plant_detector_node')
        self.declare_parameter('confidence_threshold', 0.5)
        self.declare_parameter('min_area_ratio', 0.05)
        self.declare_parameter('use_simulation', False)
        self.declare_parameter('model_path', '')

        self.conf_threshold = self.get_parameter('confidence_threshold').value
        self.min_area_ratio = self.get_parameter('min_area_ratio').value
        self.use_simulation = self.get_parameter('use_simulation').value
        model_path = self.get_parameter('model_path').value

        self._model = None
        self._dnn = None
        self._paused = False

        if not self.use_simulation:
            self._load_model(model_path)

        self.bridge = CvBridge()
        self.sub = self.create_subscription(
            Image, '/sentry/camera/image_raw', self.on_image, 1)
        self.pub = self.create_publisher(
            PlantDetection, '/vision/plant_detected', 10)

        self.pause_srv = self.create_service(
            SetBool, '/vision/plant_detector/pause', self.on_pause)

        self.get_logger().info('Plant detector node ready (YOLOv8n BPU)')

    def _load_model(self, model_path: str):
        from hobot_dnn import pyeasy_dnn as dnn
        self._dnn = dnn

        resolved = model_path
        if not resolved:
            import os
            candidates = [
                os.path.join(os.getcwd(), 'models',
                             'yolov8n_crop_weed_bayese_640x640_nv12.bin'),
                os.path.join(os.path.dirname(__file__), '..', '..', '..',
                             'models', 'yolov8n_crop_weed_bayese_640x640_nv12.bin'),
            ]
            for c in candidates:
                if os.path.exists(c):
                    resolved = c
                    break

        self.get_logger().info(f'Loading YOLO model: {resolved}')
        self._model = dnn.load(resolved)[0]
        self.get_logger().info(
            f'YOLO model loaded. name={self._model.name}')

    def _unload_model(self):
        self._model = None

    def on_pause(self, request, response):
        if request.data:
            if not self._paused:
                self._paused = True
                self._unload_model()
                self.get_logger().info('Paused — model unloaded')
            response.success = True
            response.message = 'paused'
        else:
            if self._paused:
                self._paused = False
                model_path = self.get_parameter('model_path').value
                self._load_model(model_path)
                self.get_logger().info('Resumed — model reloaded')
            response.success = True
            response.message = 'resumed'
        return response

    def on_image(self, msg: Image):
        if self._paused:
            return

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

    def _detect(self, image):
        """Run YOLOv8n BPU inference."""
        if self._model is None:
            return False, [0.0, 0.0, 0.0, 0.0], 0.0, 0.0

        input_tensor = bgr_to_nv12_resized(image, 640)
        outputs = self._model.forward([input_tensor])
        output = outputs[0].buffer.reshape(-1)

        # Reshape based on actual size
        expected = 6 * 8400
        if output.size != expected:
            self.get_logger().warn(
                f'Unexpected YOLO output size: {output.size}, expected {expected}')
            return False, [0.0, 0.0, 0.0, 0.0], 0.0, 0.0

        output = output.reshape(1, 6, 8400)
        detected, bbox, confidence = yolo_postprocess(
            output, input_size=640, conf_threshold=self.conf_threshold)

        if not detected:
            return False, bbox, confidence, 0.0

        x1, y1, x2, y2 = bbox
        area_ratio = float((x2 - x1) * (y2 - y1))

        if area_ratio < self.min_area_ratio:
            return False, bbox, confidence, area_ratio

        return True, bbox, confidence, area_ratio

    def _simulate(self, image):
        """Simulation mode: always detect a centered plant."""
        return True, [0.3, 0.3, 0.7, 0.7], 0.95, 0.16

    def destroy_node(self):
        self._unload_model()
        super().destroy_node()


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
