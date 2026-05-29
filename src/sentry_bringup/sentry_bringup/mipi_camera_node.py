#!/usr/bin/env python3
"""MIPI camera driver node for RDK X5.

Publishes BGR8 images from the MIPI camera to /camera/image_raw.
Requires hobot_vio which is only available on the RDK X5 board.

Reference: https://forum.d-robotics.cc/t/topic/34355
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

        # RDK Camera API requires two output channels with DIFFERENT resolutions.
        # Channel 0 (index 0, accessed via get_img type=2): main resolution
        # Channel 1 (index 1, accessed via get_img type=0): auxiliary
        # Passing identical resolutions causes VSE init failure (ret=-10).
        # See forum post Section 3 & 7.1.
        out_w = [self.width, 512]
        out_h = [self.height, 512]

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

    def _nv12_to_bgr(self, nv12_data, width, height, actual_size):
        """Convert NV12 bytes to BGR, handling stride alignment.

        The camera hardware may pad each row to 64-byte or 32-byte boundaries.
        We must inspect actual_size and derive the real stride before reshaping.
        See forum post Section 4.3.
        """
        expected_size = int(width * height * 1.5)

        if actual_size == expected_size:
            # No padding — width is already stride-aligned (e.g. 1920 % 64 == 0)
            yuv = np.frombuffer(nv12_data, dtype=np.uint8).reshape(
                (int(height * 1.5), width)
            )
            return cv2.cvtColor(yuv, cv2.COLOR_YUV2BGR_NV12)

        # Try 64-byte stride alignment first
        stride = (width + 63) // 64 * 64
        y_aligned = stride * height
        uv_aligned = stride * height // 2
        if actual_size == y_aligned + uv_aligned:
            pass  # stride confirmed
        else:
            # Fall back to 32-byte alignment
            stride = (width + 31) // 32 * 32
            y_aligned = stride * height
            uv_aligned = stride * height // 2
            if actual_size != y_aligned + uv_aligned:
                self.get_logger().warn(
                    f'NV12 size mismatch: actual={actual_size}, '
                    f'expected={expected_size}, stride64={stride} gives '
                    f'{y_aligned + uv_aligned}. Skipping frame.'
                )
                return None

        raw = np.frombuffer(nv12_data, dtype=np.uint8)
        y_plane = raw[:y_aligned].reshape((height, stride))[:, :width]
        uv_plane = raw[y_aligned:y_aligned + uv_aligned].reshape(
            (height // 2, stride)
        )[:, :width]

        yuv = np.zeros((int(height * 1.5), width), dtype=np.uint8)
        yuv[:height, :] = y_plane
        yuv[height:, :] = uv_plane
        return cv2.cvtColor(yuv, cv2.COLOR_YUV2BGR_NV12)

    def capture(self):
        # type=2 accesses the first output channel (index 0) defined in open_cam.
        # In the official HDMI sample this is 512x512 for AI; here it is 1920x1080.
        img_buf = self.cam.get_img(2, self.width, self.height)
        if img_buf is None:
            self.get_logger().warn('get_img returned None')
            return

        actual_size = len(img_buf)
        if actual_size == 0:
            self.get_logger().warn('get_img returned empty buffer')
            return

        self.get_logger().debug(
            f'get_img returned {actual_size} bytes '
            f'(expected {int(self.width * self.height * 1.5)})'
        )

        frame = self._nv12_to_bgr(img_buf, self.width, self.height, actual_size)
        if frame is None:
            return

        msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
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
