#!/usr/bin/env python3
"""Rotation calibration pass 2 (clockwise): gyro-integration reference.

Compares three yaw deltas at rest after stopping at -90deg EKF yaw:
  - EKF fused yaw (what Nav2 uses)
  - wheel odometry yaw
  - IMU gyro z-rate integration (magnetometer-free physical reference)
"""

import math
import time
import urllib.request
import json

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu

TARGET = -math.pi / 2.0         # -90 deg as read by EKF yaw
ANGULAR_CMD = -0.20             # rad/s, clockwise
SETTLE_SEC = 2.0
CONTROL_URL = 'http://127.0.0.1:5000/control'


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def ang_diff(a, b):
    return math.atan2(math.sin(a - b), math.cos(a - b))


class RotCalib(Node):
    def __init__(self):
        super().__init__('rot_calib2')
        self.ekf_yaw = None
        self.wheel_yaw = None
        self.imu_yaw = None
        self.gyro_int = 0.0
        self._gyro_t = None
        self.create_subscription(
            Odometry, '/odometry/filtered', lambda m: self._set('ekf', m), 10)
        self.create_subscription(
            Odometry, '/wheel/odom', lambda m: self._set('wheel', m), 10)
        self.create_subscription(Imu, '/sensor/imu/data', self._on_imu, 50)

    def _set(self, which, msg):
        yaw = yaw_of(msg.pose.pose.orientation)
        if which == 'ekf':
            self.ekf_yaw = yaw
        else:
            self.wheel_yaw = yaw

    def _on_imu(self, msg):
        self.imu_yaw = yaw_of(msg.orientation)
        now = time.time()
        if self._gyro_t is not None:
            dt = now - self._gyro_t
            if dt < 0.1:
                self.gyro_int += msg.angular_velocity.z * dt
        self._gyro_t = now


def post_control(linear, angular):
    req = urllib.request.Request(
        CONTROL_URL,
        data=json.dumps({'linear': linear, 'angular': angular}).encode(),
        headers={'Content-Type': 'application/json'})
    try:
        urllib.request.urlopen(req, timeout=1.0).read()
        return True
    except Exception as exc:
        print(f'control POST failed: {exc}')
        return False


def main():
    rclpy.init()
    node = RotCalib()

    t0 = time.time()
    while (node.ekf_yaw is None or node.wheel_yaw is None
           or node.imu_yaw is None):
        rclpy.spin_once(node, timeout_sec=0.1)
        if time.time() - t0 > 10:
            print('TIMEOUT waiting for yaw topics')
            return

    # gyro bias: sample while stationary
    time.sleep(1.0)
    for _ in range(20):
        rclpy.spin_once(node, timeout_sec=0.05)
    gyro0 = node.gyro_int

    ekf0, wheel0, imu0 = node.ekf_yaw, node.wheel_yaw, node.imu_yaw
    print(f'start: ekf={math.degrees(ekf0):.2f} wheel={math.degrees(wheel0):.2f} '
          f'imu_abs={math.degrees(imu0):.2f} gyro_int=0.00')

    t0 = time.time()
    while True:
        rclpy.spin_once(node, timeout_sec=0.02)
        d = ang_diff(node.ekf_yaw, ekf0)
        post_control(0.0, ANGULAR_CMD)
        if d <= TARGET:
            break
        if time.time() - t0 > 40:
            print('TIMEOUT during rotation')
            break
        time.sleep(0.05)

    post_control(0.0, 0.0)
    time.sleep(0.2)
    post_control(0.0, 0.0)
    print('stop command sent, settling...')
    time.sleep(SETTLE_SEC)
    for _ in range(20):
        rclpy.spin_once(node, timeout_sec=0.05)

    d_ekf = ang_diff(node.ekf_yaw, ekf0)
    d_wheel = ang_diff(node.wheel_yaw, wheel0)
    d_imu_abs = ang_diff(node.imu_yaw, imu0)
    d_gyro = node.gyro_int - gyro0
    print(f'DELTA: ekf={math.degrees(d_ekf):.2f}deg '
          f'wheel={math.degrees(d_wheel):.2f}deg '
          f'imu_abs={math.degrees(d_imu_abs):.2f}deg '
          f'gyro_int={math.degrees(d_gyro):.2f}deg')
    if abs(d_gyro) > 0.05:
        ratio = abs(d_wheel / d_gyro)
        print(f'|wheel/gyro| ratio: {ratio:.3f}')
        print(f'effective wheel_base estimate: {0.23 * ratio:.3f} m '
              f'(geometric 0.23 m)')
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
