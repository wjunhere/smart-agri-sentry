#!/usr/bin/env python3
"""MIPI camera driver node for RDK X5.

Publishes BGR8 images from the MIPI camera to /camera/image_raw.
Requires hobot_vio which is only available on the RDK X5 board.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import signal
import sys


class MipiCameraNode(Node):
    def __init__(self):
        super().__init__('mipi_camera_node')

        # Parameters
        self.declare_parameter('width', 1920)
        self.declare_parameter('height', 1080)
        self.declare_parameter('fps', 30.0)
        self.declare_parameter('frame_id', 'camera')
        self.declare_parameter('sensor_width', 1920)
        self.declare_parameter('sensor_height', 1080)

        self.width = self.get_parameter('width').value
        self.height = self.get_parameter('height').value
        self.fps = self.get_parameter('fps').value
        self.frame_id = self.get_parameter('frame_id').value
        self.sensor_width = self.get_parameter('sensor_width').value
        self.sensor_height = self.get_parameter('sensor_height').value

        # Import hobot_vio (only available on RDK X5)
        try:
            from hobot_vio import libsrcampy as srcampy
            self.srcampy = srcampy
        except ImportError as e:
            self.get_logger().error(
                f'Failed to import hobot_vio: {e}\n'
                'This node must run on the RDK X5 board.'
            )
            rclpy.shutdown()
            sys.exit(1)

        # Initialize MIPI camera
        self.cam = self.srcampy.Camera()
        self.get_logger().info(
            f'Opening MIPI camera: output={self.width}x{self.height}, '
            f'sensor={self.sensor_width}x{self.sensor_height}'
        )

        # RDK Camera API requires at least two output channels in the list.
        # Channel 0: main resolution for publishing
        # Channel 1: auxiliary (used by the VSE pipeline, can be any valid size)
        out_w = [self.width, self.width]
        out_h = [self.height, self.height]

        self.get_logger().info(
            f'Calling open_cam(0, -1, -1, out_w={out_w}, out_h={out_h}, '
            f'sensor_h={self.sensor_height}, sensor_w={self.sensor_width})'
        )

        ret = self.cam.open_cam(
            0,                      # device_id
            -1,                     # fps (auto)
            -1,                     # format (auto, usually NV12)
            out_w,                  # output width list
            out_h,                  # output height list
            self.sensor_height,     # sensor height
            self.sensor_width,      # sensor width
        )
        if ret != 0:
            self.get_logger().error(
                f'Failed to open MIPI camera (ret={ret}).\n'
                'Common causes:\n'
                '  - Camera already in use by another process\n'
                '  - Resolution not supported by the sensor\n'
                '  - MIPI pipeline not released after previous crash (reboot required)\n'
                'Try: sudo reboot, then run again.'
            )
            rclpy.shutdown()
            sys.exit(1)

        self.get_logger().info('MIPI camera opened successfully')

        self.bridge = CvBridge()
        self.pub = self.create_publisher(Image, '/camera/image_raw', 10)

        # Timer drives capture loop at target fps
        timer_period = 1.0 / self.fps
        self.timer = self.create_timer(timer_period, self.capture)
        self.frame_count = 0

    def capture(self):
        # get_img(type=2, w, h) -> NV12 format
        # type 2 corresponds to NV12 in hobot_vio
        img_buf = self.cam.get_img(2, self.width, self.height)
        if img_buf is None or len(img_buf) == 0:
            self.get_logger().warn('Failed to get image from MIPI camera')
            return

        # NV12 layout: Y plane (h * w) + UV plane (h * w / 2)
        # Total bytes = w * h * 1.5
        expected_size = int(self.width * self.height * 1.5)
        if len(img_buf) != expected_size:
            self.get_logger().warn(
                f'NV12 buffer size mismatch: got {len(img_buf)}, expected {expected_size}'
            )
            return

        # Convert to numpy array and reshape for OpenCV
        img_arr = np.frombuffer(img_buf, dtype=np.uint8)
        img_nv12 = img_arr.reshape((int(self.height * 1.5), self.width))

        # NV12 -> BGR8
        img_bgr = cv2.cvtColor(img_nv12, cv2.COLOR_YUV2BGR_NV12)

        # Build ROS Image message
        msg = self.bridge.cv2_to_imgmsg(img_bgr, encoding='bgr8')
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        self.pub.publish(msg)

        self.frame_count += 1
        if self.frame_count % 30 == 0:
            self.get_logger().info(f'Published {self.frame_count} frames')

    def destroy_node(self):
        self.get_logger().info('Closing MIPI camera...')
        if hasattr(self, 'cam') and self.cam is not None:
            self.cam.close_cam()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MipiCameraNode()

    # Ensure clean shutdown on SIGINT (Ctrl+C)
    def signal_handler(sig, frame):
        node.get_logger().info('SIGINT received, shutting down...')
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(0)

    signal.signal(signal.SIGINT, signal_handler)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
