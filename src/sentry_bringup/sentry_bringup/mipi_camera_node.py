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
import signal
import atexit


DEFAULT_IMAGE_TOPIC = '/sentry/camera/image_raw'


class MipiCameraNode(Node):
    def __init__(self):
        super().__init__('mipi_camera_node')

        # Parameters
        self.declare_parameter('device_id', 0)
        self.declare_parameter('width', 1920)
        self.declare_parameter('height', 1080)
        self.declare_parameter('fps', 30.0)
        self.declare_parameter('frame_id', 'camera')
        self.declare_parameter('sensor_width', 1920)
        self.declare_parameter('sensor_height', 1080)
        self.declare_parameter('yuv_format', 'nv12')
        # cv2.flip code: -2 = disabled, -1 = rotate 180 (upside-down mount),
        # 0 = vertical flip, 1 = horizontal flip
        self.declare_parameter('flip_code', -2)
        self.declare_parameter('enable_color_correction', False)
        self.declare_parameter('blue_gain', 1.0)
        self.declare_parameter('green_gain', 1.0)
        self.declare_parameter('red_gain', 1.0)
        self.declare_parameter('enable_low_light_enhancement', False)
        self.declare_parameter('denoise_h', 0.0)
        self.declare_parameter('gamma', 1.0)
        self.declare_parameter('saturation_scale', 1.0)
        self.declare_parameter('sharpen_amount', 0.0)
        self.declare_parameter('enable_undistort', False)
        self.declare_parameter('undistort_calib_file', '')
        self.declare_parameter('undistort_alpha', 0.0)

        self.device_id = int(self.get_parameter('device_id').value)
        self.width = self.get_parameter('width').value
        self.height = self.get_parameter('height').value
        self.fps = self.get_parameter('fps').value
        self.frame_id = self.get_parameter('frame_id').value
        self.sensor_width = self.get_parameter('sensor_width').value
        self.sensor_height = self.get_parameter('sensor_height').value
        self.yuv_format = str(self.get_parameter('yuv_format').value).lower()
        if self.yuv_format not in ('nv12', 'nv21'):
            self.get_logger().warn(
                f'Unsupported yuv_format={self.yuv_format}; using nv12')
            self.yuv_format = 'nv12'
        self.flip_code = int(self.get_parameter('flip_code').value)
        if self.flip_code not in (-2, -1, 0, 1):
            self.get_logger().warn(
                f'Unsupported flip_code={self.flip_code}; disabling flip')
            self.flip_code = -2
        self.enable_color_correction = self.get_parameter(
            'enable_color_correction').value
        self.blue_gain = float(self.get_parameter('blue_gain').value)
        self.green_gain = float(self.get_parameter('green_gain').value)
        self.red_gain = float(self.get_parameter('red_gain').value)
        self.enable_low_light_enhancement = self.get_parameter(
            'enable_low_light_enhancement').value
        self.denoise_h = max(0.0, float(self.get_parameter('denoise_h').value))
        self.gamma = max(0.1, float(self.get_parameter('gamma').value))
        self.saturation_scale = max(
            0.0, float(self.get_parameter('saturation_scale').value))
        self.sharpen_amount = max(
            0.0, float(self.get_parameter('sharpen_amount').value))
        self.enable_undistort = self.get_parameter('enable_undistort').value
        self.undistort_calib_file = str(
            self.get_parameter('undistort_calib_file').value)
        self.undistort_alpha = float(
            self.get_parameter('undistort_alpha').value)
        self._undistort_map1 = None
        self._undistort_map2 = None
        if self.enable_undistort:
            self._init_undistort_maps()

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
            f'sensor={self.sensor_width}x{self.sensor_height}, '
            f'yuv_format={self.yuv_format}, '
            f'color_correction={self.enable_color_correction}, '
            f'low_light_enhancement={self.enable_low_light_enhancement}'
        )

        # RDK Camera API: the FIRST output channel resolution is limited by ISP
        # tuning. The official sample puts the SMALL resolution (512x512) first
        # and the FULL resolution (1920x1080) second.
        # Reversing the order causes vp_isp_init failure (ret=-10).
        # See forum analysis: https://forum.d-robotics.cc/t/topic/34355
        out_w = [512, self.width]
        out_h = [512, self.height]

        self.get_logger().info(
            f'Calling open_cam({self.device_id}, -1, -1, out_w={out_w}, '
            f'out_h={out_h}, sensor_h={self.sensor_height}, '
            f'sensor_w={self.sensor_width})'
        )

        ret = self.cam.open_cam(
            self.device_id,         # device_id (mipi host: CAM1=0, CAM2=2)
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
        self.pub = self.create_publisher(Image, DEFAULT_IMAGE_TOPIC, 10)

        # Timer drives capture loop at target fps
        timer_period = 1.0 / self.fps
        self.timer = self.create_timer(timer_period, self.capture)
        self.frame_count = 0

        self.add_on_set_parameters_callback(self._on_param_change)

    def _on_param_change(self, params):
        """Apply runtime parameter updates from the settings panel."""
        from rcl_interfaces.msg import SetParametersResult
        for p in params:
            value = p.value
            if p.name == 'enable_low_light_enhancement':
                self.enable_low_light_enhancement = bool(value)
            elif p.name == 'gamma':
                self.gamma = max(0.1, float(value))
            elif p.name == 'saturation_scale':
                self.saturation_scale = max(0.0, float(value))
            elif p.name == 'sharpen_amount':
                self.sharpen_amount = max(0.0, float(value))
            elif p.name == 'denoise_h':
                self.denoise_h = max(0.0, float(value))
            else:
                continue
            self.get_logger().info(f'param {p.name} -> {value}')
        return SetParametersResult(successful=True)

    def _init_undistort_maps(self):
        """Load calibration YAML and precompute undistort remap tables."""
        fs = cv2.FileStorage(self.undistort_calib_file, cv2.FILE_STORAGE_READ)
        if not fs.isOpened():
            self.get_logger().error(
                f'Cannot open undistort calib file: '
                f'{self.undistort_calib_file}; undistort disabled')
            self.enable_undistort = False
            return
        K = fs.getNode('camera_matrix').mat()
        dist = fs.getNode('distortion_coefficients').mat()
        calib_w = int(fs.getNode('image_width').real())
        calib_h = int(fs.getNode('image_height').real())
        fs.release()
        if K is None or dist is None:
            self.get_logger().error(
                'Calib file missing camera_matrix/distortion_coefficients; '
                'undistort disabled')
            self.enable_undistort = False
            return
        if (calib_w, calib_h) != (self.width, self.height):
            self.get_logger().warn(
                f'Calib resolution {calib_w}x{calib_h} != output '
                f'{self.width}x{self.height}; undistort disabled')
            self.enable_undistort = False
            return
        new_K, _ = cv2.getOptimalNewCameraMatrix(
            K, dist, (self.width, self.height),
            self.undistort_alpha, (self.width, self.height))
        self._undistort_map1, self._undistort_map2 = \
            cv2.initUndistortRectifyMap(
                K, dist, None, new_K, (self.width, self.height),
                cv2.CV_16SC2)
        self.get_logger().info(
            f'Undistort enabled: calib={self.undistort_calib_file}, '
            f'alpha={self.undistort_alpha}')

    def _apply_undistort(self, frame):
        if not self.enable_undistort or self._undistort_map1 is None:
            return frame
        return cv2.remap(frame, self._undistort_map1, self._undistort_map2,
                         cv2.INTER_LINEAR)

    def _yuv_to_bgr_code(self):
        if self.yuv_format == 'nv21':
            return cv2.COLOR_YUV2BGR_NV21
        return cv2.COLOR_YUV2BGR_NV12

    def _apply_color_correction(self, frame):
        if not self.enable_color_correction:
            return frame
        corrected = frame.astype(np.float32)
        corrected[:, :, 0] *= self.blue_gain
        corrected[:, :, 1] *= self.green_gain
        corrected[:, :, 2] *= self.red_gain
        return np.clip(corrected, 0, 255).astype(np.uint8)

    def _build_gamma_lut(self, gamma):
        inv_gamma = 1.0 / max(0.1, gamma)
        return np.array([
            ((i / 255.0) ** inv_gamma) * 255.0 for i in range(256)
        ], dtype=np.uint8)

    def _apply_low_light_enhancement(self, frame):
        if not self.enable_low_light_enhancement:
            return frame

        enhanced = frame
        if self.denoise_h > 0.0:
            enhanced = cv2.fastNlMeansDenoisingColored(
                enhanced, None, self.denoise_h, self.denoise_h, 7, 21)

        if abs(self.gamma - 1.0) > 1e-3:
            enhanced = cv2.LUT(enhanced, self._build_gamma_lut(self.gamma))

        if abs(self.saturation_scale - 1.0) > 1e-3:
            hsv = cv2.cvtColor(enhanced, cv2.COLOR_BGR2HSV)
            hsv = hsv.astype(np.float32)
            hsv[:, :, 1] *= self.saturation_scale
            hsv[:, :, 1] = np.clip(hsv[:, :, 1], 0, 255)
            enhanced = cv2.cvtColor(
                hsv.astype(np.uint8), cv2.COLOR_HSV2BGR)

        if self.sharpen_amount > 0.0:
            blurred = cv2.GaussianBlur(enhanced, (0, 0), 1.0)
            enhanced = cv2.addWeighted(
                enhanced,
                1.0 + self.sharpen_amount,
                blurred,
                -self.sharpen_amount,
                0)

        return enhanced

    def _nv12_to_bgr(self, nv12_data, width, height, actual_size):
        """Convert NV12/NV21 bytes to BGR, handling stride alignment.

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
            return cv2.cvtColor(yuv, self._yuv_to_bgr_code())

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
            if actual_size == y_aligned + uv_aligned:
                pass  # stride confirmed
            else:
                # Auto-detect stride from actual buffer size (NV12: Y + UV/2)
                stride = int(actual_size / (height * 1.5))
                y_aligned = stride * height
                uv_aligned = stride * height // 2
                if actual_size != y_aligned + uv_aligned:
                    self.get_logger().warn(
                        f'NV12 size mismatch: actual={actual_size}, '
                        f'expected={expected_size}, auto-stride={stride} '
                        f'gives {y_aligned + uv_aligned}. Skipping frame.'
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
        return cv2.cvtColor(yuv, self._yuv_to_bgr_code())

    def capture(self):
        try:
            # Fetch from the SECOND output channel (full resolution) directly.
            # hobot_vio get_img: type=2 is NV12 format. Channel is selected
            # by matching width/height against the out_w/out_h list from open_cam.
            # Pass self.width x self.height to get channel 1 (full res).
            img_buf = self.cam.get_img(2, self.width, self.height)
            if img_buf is None:
                self.get_logger().warn('get_img returned None')
                return

            actual_size = len(img_buf)
            if actual_size == 0:
                self.get_logger().warn('get_img returned empty buffer')
                return

            # Convert NV12 -> BGR at target resolution
            frame = self._nv12_to_bgr(img_buf, self.width, self.height, actual_size)
            if frame is None:
                return
            if self.flip_code != -2:
                frame = cv2.flip(frame, self.flip_code)
            frame = self._apply_undistort(frame)
            frame = self._apply_color_correction(frame)
            frame = self._apply_low_light_enhancement(frame)

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

    def _cleanup():
        if hasattr(node, 'destroy_node'):
            node.destroy_node()

    atexit.register(_cleanup)
    signal.signal(signal.SIGTERM, lambda signum, frame: sys.exit(0))

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
