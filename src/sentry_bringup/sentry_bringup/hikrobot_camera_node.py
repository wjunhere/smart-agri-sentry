#!/usr/bin/env python3
"""Hikrobot MVS camera driver node.

Publishes BGR8 images from a Hikrobot USB3 Vision/GigE camera to the same
topic used by the existing vision pipeline.
"""

import atexit
import importlib
import math
import os
import signal
import sys
import time
from ctypes import (
    POINTER,
    byref,
    c_ubyte,
    cast,
    memmove,
    memset,
    sizeof,
)

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry

from sentry_bringup.auto_exposure import (
    AdaptiveExposureController,
    compute_luma_stats,
)


DEFAULT_IMAGE_TOPIC = '/sentry/camera/image_raw'
DEFAULT_MVS_COMMON_RUNENV = '/opt/MVS/lib'
DEFAULT_MVS_PYTHON_PATH = '/opt/MVS/Samples/aarch64/Python/MvImport'
DEFAULT_MVS_LIBRARY_PATH = '/opt/MVS/lib/aarch64'
MV_OK = 0


def _to_hex(value):
    if value < 0:
        value += 2 ** 32
    return f'0x{value:x}'


def _decode_ctypes_string(ctypes_char_array):
    raw = memoryview(ctypes_char_array).tobytes()
    nul = raw.find(b'\x00')
    if nul != -1:
        raw = raw[:nul]
    for encoding in ('gbk', 'utf-8', 'latin-1'):
        try:
            return raw.decode(encoding)
        except UnicodeDecodeError:
            continue
    return raw.decode('latin-1', errors='replace')


def _prepend_unique_env_path(name, value):
    if not value:
        return
    current = os.environ.get(name, '')
    parts = [part for part in current.split(os.pathsep) if part]
    if value not in parts:
        os.environ[name] = os.pathsep.join([value] + parts)


def _load_mvs_sdk(common_runenv, python_path, library_path):
    os.environ.setdefault('MVCAM_COMMON_RUNENV', common_runenv)
    _prepend_unique_env_path('LD_LIBRARY_PATH', library_path)
    if python_path not in sys.path:
        sys.path.insert(0, python_path)
    return importlib.import_module('MvCameraControl_class')


class HikrobotCameraNode(Node):
    def __init__(self):
        super().__init__('hikrobot_camera_node')

        self.declare_parameter('image_topic', DEFAULT_IMAGE_TOPIC)
        self.declare_parameter('frame_id', 'camera')
        self.declare_parameter('fps', 10.0)
        self.declare_parameter('device_index', 0)
        self.declare_parameter('grab_timeout_ms', 1000)
        self.declare_parameter('output_width', 640)
        self.declare_parameter('output_height', 480)
        self.declare_parameter('mvs_common_runenv', DEFAULT_MVS_COMMON_RUNENV)
        self.declare_parameter('mvs_python_path', DEFAULT_MVS_PYTHON_PATH)
        self.declare_parameter('mvs_library_path', DEFAULT_MVS_LIBRARY_PATH)
        self.declare_parameter('bayer_cvt_quality', 1)
        self.declare_parameter('exposure_time_us', 100000.0)
        self.declare_parameter('gain', 3.0)
        self.declare_parameter('exposure_auto', False)
        self.declare_parameter('gain_auto', False)
        self.declare_parameter('auto_exposure_min_us', 2000.0)
        self.declare_parameter('auto_exposure_max_us', 40000.0)
        self.declare_parameter('auto_gain_min', 0.0)
        self.declare_parameter('auto_gain_max', 12.0)
        self.declare_parameter('enable_image_enhancement', False)
        self.declare_parameter('gamma', 3.0)
        self.declare_parameter('ae_enabled', True)
        self.declare_parameter('ae_target_luma', 80.0)
        self.declare_parameter('ae_deadband', 0.05)
        self.declare_parameter('ae_max_step', 1.4)
        self.declare_parameter('ae_sat_limit', 0.02)
        self.declare_parameter('ae_exp_min_us', 2000.0)
        self.declare_parameter('ae_exp_max_moving_us', 20000.0)
        self.declare_parameter('ae_exp_max_still_us', 100000.0)
        self.declare_parameter('ae_gain_min', 0.0)
        self.declare_parameter('ae_gain_max', 12.0)
        self.declare_parameter('ae_move_speed_thresh', 0.05)
        self.declare_parameter('ae_still_speed_thresh', 0.02)
        self.declare_parameter('ae_update_period_s', 0.4)
        self.declare_parameter('odom_topic', '/wheel/odom')

        self.image_topic = self.get_parameter('image_topic').value
        self.frame_id = self.get_parameter('frame_id').value
        self.fps = max(0.1, float(self.get_parameter('fps').value))
        self.device_index = int(self.get_parameter('device_index').value)
        self.grab_timeout_ms = int(self.get_parameter('grab_timeout_ms').value)
        self.output_width = int(self.get_parameter('output_width').value)
        self.output_height = int(self.get_parameter('output_height').value)
        self.bayer_cvt_quality = int(
            self.get_parameter('bayer_cvt_quality').value)
        self.exposure_time_us = float(
            self.get_parameter('exposure_time_us').value)
        self.gain = float(self.get_parameter('gain').value)
        self.exposure_auto = bool(
            self.get_parameter('exposure_auto').value)
        self.gain_auto = bool(self.get_parameter('gain_auto').value)
        self.auto_exposure_min_us = float(
            self.get_parameter('auto_exposure_min_us').value)
        self.auto_exposure_max_us = float(
            self.get_parameter('auto_exposure_max_us').value)
        self.auto_gain_min = float(
            self.get_parameter('auto_gain_min').value)
        self.auto_gain_max = float(
            self.get_parameter('auto_gain_max').value)
        self.enable_image_enhancement = bool(
            self.get_parameter('enable_image_enhancement').value)
        self.gamma = max(0.1, float(self.get_parameter('gamma').value))
        self.gamma_lut = self._build_gamma_lut(self.gamma)

        self.ae_enabled = bool(self.get_parameter('ae_enabled').value)
        self.ae_move_speed_thresh = float(
            self.get_parameter('ae_move_speed_thresh').value)
        self.ae_still_speed_thresh = float(
            self.get_parameter('ae_still_speed_thresh').value)
        self.ae_controller = None
        self._moving = True
        self._last_odom_time = None
        self._still_since = None
        self._now = time.monotonic
        self._last_ae_exposure = None
        self._last_ae_gain = None
        if self.ae_enabled:
            self.ae_controller = AdaptiveExposureController(
                target_luma=float(self.get_parameter('ae_target_luma').value),
                deadband=float(self.get_parameter('ae_deadband').value),
                max_step=float(self.get_parameter('ae_max_step').value),
                sat_limit=float(self.get_parameter('ae_sat_limit').value),
                exp_min_us=float(self.get_parameter('ae_exp_min_us').value),
                exp_max_moving_us=float(
                    self.get_parameter('ae_exp_max_moving_us').value),
                exp_max_still_us=float(
                    self.get_parameter('ae_exp_max_still_us').value),
                gain_min=float(self.get_parameter('ae_gain_min').value),
                gain_max=float(self.get_parameter('ae_gain_max').value),
                update_period_s=float(
                    self.get_parameter('ae_update_period_s').value))

        self.mvs = _load_mvs_sdk(
            self.get_parameter('mvs_common_runenv').value,
            self.get_parameter('mvs_python_path').value,
            self.get_parameter('mvs_library_path').value,
        )
        self.sdk_initialized = False
        self.cam = None
        self.grabbing = False
        self._destroyed = False
        self.frame_count = 0

        self._open_camera()

        self.bridge = CvBridge()
        self.pub = self.create_publisher(Image, self.image_topic, 10)
        if self.ae_enabled:
            self.odom_sub = self.create_subscription(
                Odometry, self.get_parameter('odom_topic').value,
                self._on_odom, 10)
        self.timer = self.create_timer(1.0 / self.fps, self.capture)
        self.get_logger().info(
            f'Hikrobot camera publishing {self.image_topic} at {self.fps:.1f} fps')

    def _open_camera(self):
        self.mvs.MvCamera.MV_CC_Initialize()
        self.sdk_initialized = True

        sdk_version = self.mvs.MvCamera.MV_CC_GetSDKVersion()
        self.get_logger().info(f'MVS SDK version: {_to_hex(sdk_version)}')

        device_list = self.mvs.MV_CC_DEVICE_INFO_LIST()
        layer_type = self.mvs.MV_GIGE_DEVICE | self.mvs.MV_USB_DEVICE
        ret = self.mvs.MvCamera.MV_CC_EnumDevices(layer_type, device_list)
        if ret != MV_OK:
            raise RuntimeError(f'Enum Hikrobot devices failed: {_to_hex(ret)}')
        if device_list.nDeviceNum == 0:
            raise RuntimeError('No Hikrobot MVS camera found')
        if self.device_index >= device_list.nDeviceNum:
            raise RuntimeError(
                f'device_index={self.device_index} out of range; '
                f'found {device_list.nDeviceNum} device(s)')

        for i in range(device_list.nDeviceNum):
            self.get_logger().info(
                f'Hikrobot device [{i}]: {self._device_label(device_list, i)}')

        self.cam = self.mvs.MvCamera()
        selected = cast(
            device_list.pDeviceInfo[self.device_index],
            POINTER(self.mvs.MV_CC_DEVICE_INFO),
        ).contents

        ret = self.cam.MV_CC_CreateHandle(selected)
        if ret != MV_OK:
            raise RuntimeError(f'Create camera handle failed: {_to_hex(ret)}')

        ret = self.cam.MV_CC_OpenDevice(self.mvs.MV_ACCESS_Exclusive, 0)
        if ret != MV_OK:
            raise RuntimeError(f'Open Hikrobot camera failed: {_to_hex(ret)}')

        if selected.nTLayerType == self.mvs.MV_GIGE_DEVICE:
            packet_size = self.cam.MV_CC_GetOptimalPacketSize()
            if int(packet_size) > 0:
                ret = self.cam.MV_CC_SetIntValue(
                    'GevSCPSPacketSize', packet_size)
                if ret != MV_OK:
                    self.get_logger().warn(
                        f'Set GevSCPSPacketSize failed: {_to_hex(ret)}')

        self._configure_exposure_and_gain()
        if self.ae_enabled:
            exposure = (self._read_optional_float('ExposureTime')
                        or self.exposure_time_us)
            gain = self._read_optional_float('Gain') or self.gain
            self._seed_ae_controller(exposure, gain)

        ret = self.cam.MV_CC_SetEnumValue(
            'TriggerMode', self.mvs.MV_TRIGGER_MODE_OFF)
        if ret != MV_OK:
            raise RuntimeError(f'Set TriggerMode=Off failed: {_to_hex(ret)}')

        if self.bayer_cvt_quality >= 0:
            ret = self.cam.MV_CC_SetBayerCvtQuality(self.bayer_cvt_quality)
            if ret != MV_OK:
                self.get_logger().warn(
                    f'Set Bayer conversion quality failed: {_to_hex(ret)}')

        ret = self.cam.MV_CC_StartGrabbing()
        if ret != MV_OK:
            raise RuntimeError(f'Start grabbing failed: {_to_hex(ret)}')
        self.grabbing = True
        self.get_logger().info('Hikrobot camera opened and grabbing')

    def _set_optional_float(self, name, value):
        if value <= 0.0:
            return
        ret = self.cam.MV_CC_SetFloatValue(name, value)
        if ret != MV_OK:
            self.get_logger().warn(
                f'Set {name}={value} failed: {_to_hex(ret)}')

    def _read_optional_float(self, name):
        value = self.mvs.MVCC_FLOATVALUE()
        ret = self.cam.MV_CC_GetFloatValue(name, value)
        if ret != MV_OK:
            self.get_logger().warn(
                f'Read {name} failed: {_to_hex(ret)}')
            return None
        return float(value.fCurValue)

    def _read_optional_float_limits(self, name):
        value = self.mvs.MVCC_FLOATVALUE()
        ret = self.cam.MV_CC_GetFloatValue(name, value)
        if ret != MV_OK:
            self.get_logger().warn(
                f'Read {name} limits failed: {_to_hex(ret)}')
            return None
        return float(value.fMin), float(value.fMax)

    def _read_optional_enum(self, name):
        value = self.mvs.MVCC_ENUMVALUE()
        ret = self.cam.MV_CC_GetEnumValue(name, value)
        if ret != MV_OK:
            self.get_logger().warn(
                f'Read {name} failed: {_to_hex(ret)}')
            return None
        return int(value.nCurValue)

    def _log_hardware_exposure_and_gain(self):
        exposure = self._read_optional_float('ExposureTime')
        exposure_limits = self._read_optional_float_limits('ExposureTime')
        gain = self._read_optional_float('Gain')
        exposure_auto = self._read_optional_enum('ExposureAuto')
        gain_auto = self._read_optional_enum('GainAuto')
        self.get_logger().info(
            'Hardware exposure: '
            f'ExposureAuto={exposure_auto}, ExposureTime={exposure}, '
            f'ExposureTimeRange={exposure_limits}, '
            f'GainAuto={gain_auto}, Gain={gain}')

    @staticmethod
    def _build_gamma_lut(gamma):
        inv_gamma = 1.0 / max(0.1, gamma)
        return np.array([
            ((i / 255.0) ** inv_gamma) * 255.0 for i in range(256)
        ], dtype=np.uint8)

    def _apply_image_enhancement(self, frame):
        if not self.enable_image_enhancement:
            return frame
        return cv2.LUT(frame, self.gamma_lut)

    def _configure_exposure_and_gain(self):
        if self.exposure_auto:
            self._set_optional_float(
                'AutoExposureTimeLowerLimit', self.auto_exposure_min_us)
            self._set_optional_float(
                'AutoExposureTimeUpperLimit', self.auto_exposure_max_us)
            self._set_optional_enum('ExposureAuto', 2)
        else:
            self._set_optional_enum('ExposureAuto', 0)
            self._set_optional_float('ExposureTime', self.exposure_time_us)

        if self.gain_auto:
            self._set_optional_float('AutoGainLowerLimit', self.auto_gain_min)
            self._set_optional_float('AutoGainUpperLimit', self.auto_gain_max)
            self._set_optional_enum('GainAuto', 2)
        else:
            self._set_optional_enum('GainAuto', 0)
            self._set_optional_float('Gain', self.gain)

    def _seed_ae_controller(self, exposure_us, gain):
        self.ae_controller.seed(exposure_us, gain)
        self._last_ae_exposure = exposure_us
        self._last_ae_gain = gain
        self.get_logger().info(
            f'AE seeded from hardware: exposure={exposure_us:.0f}us '
            f'gain={gain:.2f}')

    def _on_odom(self, msg):
        speed = math.hypot(msg.twist.twist.linear.x,
                           msg.twist.twist.linear.y)
        now = self._now()
        self._last_odom_time = now
        if speed > self.ae_move_speed_thresh:
            self._moving = True
            self._still_since = None
        elif speed < self.ae_still_speed_thresh:
            if self._still_since is None:
                self._still_since = now
            elif now - self._still_since >= 1.0:
                self._moving = False
        else:
            self._still_since = None

    def _is_moving(self):
        if self._last_odom_time is None:
            return True
        if self._now() - self._last_odom_time > 2.0:
            return True
        return self._moving

    def _write_float_register(self, name, value):
        ret = self.cam.MV_CC_SetFloatValue(name, value)
        if ret != MV_OK:
            self.get_logger().warn(
                f'AE set {name}={value:.1f} failed: {_to_hex(ret)}')

    def _ae_update_from_frame(self, bgr, now_s):
        stats = compute_luma_stats(bgr)
        cmd = self.ae_controller.update(stats, self._is_moving(), now_s)
        if cmd is None:
            return
        if self._last_ae_exposure is None or abs(
                cmd.exposure_us - self._last_ae_exposure) > 1.0:
            self._write_float_register('ExposureTime', cmd.exposure_us)
            self._last_ae_exposure = cmd.exposure_us
        if self._last_ae_gain is None or abs(
                cmd.gain - self._last_ae_gain) > 0.01:
            self._write_float_register('Gain', cmd.gain)
            self._last_ae_gain = cmd.gain
        self.get_logger().info(
            f'AE: mean={stats.mean:.1f} sat={stats.saturated_ratio:.3f} '
            f'moving={self._is_moving()} -> exp={cmd.exposure_us:.0f}us '
            f'gain={cmd.gain:.2f}')

    def _set_optional_enum(self, name, value):
        ret = self.cam.MV_CC_SetEnumValue(name, value)
        if ret != MV_OK:
            self.get_logger().warn(
                f'Set {name}={value} failed: {_to_hex(ret)}')

    def _device_label(self, device_list, index):
        info = cast(
            device_list.pDeviceInfo[index],
            POINTER(self.mvs.MV_CC_DEVICE_INFO),
        ).contents
        if info.nTLayerType == self.mvs.MV_USB_DEVICE:
            usb = info.SpecialInfo.stUsb3VInfo
            return (
                f'USB model={_decode_ctypes_string(usb.chModelName)} '
                f'serial={_decode_ctypes_string(usb.chSerialNumber)}')
        if info.nTLayerType == self.mvs.MV_GIGE_DEVICE:
            gige = info.SpecialInfo.stGigEInfo
            ip = gige.nCurrentIp
            return (
                f'GigE model={_decode_ctypes_string(gige.chModelName)} '
                f'serial={_decode_ctypes_string(gige.chSerialNumber)} '
                f'ip={(ip >> 24) & 0xff}.{(ip >> 16) & 0xff}.'
                f'{(ip >> 8) & 0xff}.{ip & 0xff}')
        return f'tlayer={info.nTLayerType}'

    def _convert_frame_to_bgr(self, frame):
        width = int(frame.stFrameInfo.nWidth)
        height = int(frame.stFrameInfo.nHeight)
        rgb_size = width * height * 3

        convert_param = self.mvs.MV_CC_PIXEL_CONVERT_PARAM_EX()
        memset(byref(convert_param), 0, sizeof(convert_param))
        convert_param.nWidth = width
        convert_param.nHeight = height
        convert_param.pSrcData = frame.pBufAddr
        convert_param.nSrcDataLen = frame.stFrameInfo.nFrameLen
        convert_param.enSrcPixelType = frame.stFrameInfo.enPixelType
        convert_param.enDstPixelType = self.mvs.PixelType_Gvsp_RGB8_Packed
        convert_param.pDstBuffer = (c_ubyte * rgb_size)()
        convert_param.nDstBufferSize = rgb_size

        ret = self.cam.MV_CC_ConvertPixelTypeEx(convert_param)
        if ret != MV_OK:
            raise RuntimeError(f'Convert pixel type failed: {_to_hex(ret)}')

        rgb_buffer = (c_ubyte * convert_param.nDstLen)()
        memmove(byref(rgb_buffer), convert_param.pDstBuffer,
                convert_param.nDstLen)
        rgb = np.frombuffer(rgb_buffer, dtype=np.uint8).reshape(
            (height, width, 3))
        bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)

        if self.output_width > 0 and self.output_height > 0:
            if width != self.output_width or height != self.output_height:
                bgr = cv2.resize(
                    bgr, (self.output_width, self.output_height),
                    interpolation=cv2.INTER_AREA)
        return bgr

    def capture(self):
        frame = self.mvs.MV_FRAME_OUT()
        memset(byref(frame), 0, sizeof(frame))
        ret = self.cam.MV_CC_GetImageBuffer(frame, self.grab_timeout_ms)
        if ret != MV_OK or not frame.pBufAddr:
            self.get_logger().warn(
                f'Hikrobot frame timeout/error: {_to_hex(ret)}')
            return

        try:
            bgr = self._convert_frame_to_bgr(frame)
            if self.ae_enabled:
                self._ae_update_from_frame(bgr, time.monotonic())
            bgr = self._apply_image_enhancement(bgr)
            msg = self.bridge.cv2_to_imgmsg(bgr, encoding='bgr8')
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = self.frame_id
            self.pub.publish(msg)

            self.frame_count += 1
            if self.frame_count % 30 == 0:
                self.get_logger().info(
                    f'Published {self.frame_count} Hikrobot frames')
                self._log_hardware_exposure_and_gain()
        except Exception as exc:
            self.get_logger().error(f'Hikrobot capture error: {exc}')
        finally:
            self.cam.MV_CC_FreeImageBuffer(frame)

    def destroy_node(self):
        if self._destroyed:
            return
        self._destroyed = True
        self.get_logger().info('Closing Hikrobot camera...')
        if hasattr(self, 'timer') and self.timer is not None:
            self.timer.cancel()
        if self.cam is not None:
            try:
                if self.grabbing:
                    self.cam.MV_CC_StopGrabbing()
                    self.grabbing = False
                self.cam.MV_CC_CloseDevice()
                self.cam.MV_CC_DestroyHandle()
            except Exception as exc:
                self.get_logger().warn(f'Hikrobot close warning: {exc}')
            self.cam = None
        if self.sdk_initialized:
            self.mvs.MvCamera.MV_CC_Finalize()
            self.sdk_initialized = False
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = HikrobotCameraNode()

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
