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

        # RDK Camera API: the FIRST output channel resolution is limited by ISP
        # tuning. The official sample puts the SMALL resolution (512x512) first
        # and the FULL resolution (1920x1080) second.
        # Reversing the order causes vp_isp_init failure (ret=-10).
        # See forum analysis: https://forum.d-robotics.cc/t/topic/34355
        out_w = [512, self.width]
        out_h = [512, self.height]

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
        try:
            # type=2 corresponds to the FIRST output channel (index 0).
            # With [512, 1920] this returns the 512x512 frame.
            img_buf = self.cam.get_img(2, 512, 512)
            if img_buf is None:
                self.get_logger().warn('get_img returned None')
                return

            actual_size = len(img_buf)
            if actual_size == 0:
                self.get_logger().warn('get_img returned empty buffer')
                return

            # Convert 512x512 NV12 -> BGR
            frame_512 = self._nv12_to_bgr(img_buf, 512, 512, actual_size)
            if frame_512 is None:
                return

            # Upscale to target publish resolution (1920x1080)
            # TODO: experiment with get_img(0, 1920, 1080) to fetch directly
            # from the second output channel and avoid this CPU resize.
            frame = cv2.resize(frame_512, (self.width, self.height))

            msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = self.frame_id
            self.pub.publish(msg)

            self.frame_count += 1
            if self.frame_count % 30 == 0:
                self.get_logger().info(
                    f'Published {self.frame_count} frames'
                )
        except Exception as e:
            self.get_logger().error(f'Capture error: {e}')

    def destroy_node(self):
        self.get_logger().info('Closing MIPI camera...')
        # Cancel timer first so capture() stops firing during teardown
        if hasattr(self, 'timer') and self.timer is not None:
            self.timer.cancel()
        # close_cam() with guard against double-close
        if hasattr(self, 'cam') and self.cam is not None:
            try:
                self.cam.close_cam()
            except Exception as e:
                self.get_logger().warn(f'close_cam warning (may already be closed): {e}')
            self.cam = None
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MipiCameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
