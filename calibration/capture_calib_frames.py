#!/usr/bin/env python3
"""Capture frames from /sentry/camera/image_raw for camera calibration.

Saves one frame every INTERVAL_SEC seconds, up to MAX_FRAMES frames,
into OUT_DIR. Wave a checkerboard in front of the camera while it runs.
"""
import os
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

OUT_DIR = '/tmp/calib_imgs'
INTERVAL_SEC = 2.0
MAX_FRAMES = 30


class CalibCapture(Node):
    def __init__(self):
        super().__init__('calib_capture')
        self.bridge = CvBridge()
        self.latest = None
        self.saved = 0
        self.last_save = 0.0
        os.makedirs(OUT_DIR, exist_ok=True)
        self.sub = self.create_subscription(
            Image, '/sentry/camera/image_raw', self._on_frame, 1)
        self.timer = self.create_timer(0.2, self._tick)
        self.get_logger().info(
            f'Capturing to {OUT_DIR}: 1 frame / {INTERVAL_SEC}s, max {MAX_FRAMES}')

    def _on_frame(self, msg):
        self.latest = msg

    def _tick(self):
        if self.latest is None or self.saved >= MAX_FRAMES:
            return
        now = time.monotonic()
        if now - self.last_save < INTERVAL_SEC:
            return
        self.last_save = now
        frame = self.bridge.imgmsg_to_cv2(self.latest, 'bgr8')
        path = os.path.join(OUT_DIR, f'calib_{self.saved:02d}.jpg')
        cv2.imwrite(path, frame)
        self.saved += 1
        self.get_logger().info(f'[{self.saved}/{MAX_FRAMES}] {path}')
        if self.saved >= MAX_FRAMES:
            self.get_logger().info('Done.')
            raise SystemExit(0)


def main():
    rclpy.init()
    node = CalibCapture()
    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
