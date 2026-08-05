#!/usr/bin/env python3
"""90-degree rotation calibration: closed-loop on EKF yaw (what Nav2 sees),
then compare wheel-odom / EKF / IMU yaw deltas at rest."""

import math
import time
import urllib.request
import json

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu

TARGET = math.pi / 2.0          # 90 deg as read by EKF yaw
ANGULAR_CMD = 0.25              # rad/s, counterclockwise
SETTLE_SEC = 2.0
CONTROL_URL = 'http://127.0.0.1:5000/control'


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def ang_diff(a, b):
    return math.atan2(math.sin(a - b), math.cos(a - b))


class RotCalib(Node):
    def __init__(self):
        super().__init__('rot_calib')
        self.ekf_yaw = None
        self.wheel_yaw = None
        self.imu_yaw = None
        self.create_subscription(
            Odometry, '/odometry/filtered', lambda m: self._set('ekf', m), 10)
        self.create_subscription(
            Odometry, '/wheel/odom', lambda m: self._set('wheel', m), 10)
        self.create_subscription(
            Imu, '/sensor/imu/data', self._set_imu, 10)

    def _set(self, which, msg):
        yaw = yaw_of(msg.pose.pose.orientation)
        if which == 'ekf':
            self.ekf_yaw = yaw
        else:
            self.wheel_yaw = yaw

    def _set_imu(self, msg):
        self.imu_yaw = yaw_of(msg.orientation)


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

    # Wait for first samples
    t0 = time.time()
    while (node.ekf_yaw is None or node.wheel_yaw is None
           or node.imu_yaw is None):
        rclpy.spin_once(node, timeout_sec=0.1)
        if time.time() - t0 > 10:
            print('TIMEOUT waiting for yaw topics')
            return

    ekf0, wheel0, imu0 = node.ekf_yaw, node.wheel_yaw, node.imu_yaw
    print(f'start: ekf={math.degrees(ekf0):.2f} wheel={math.degrees(wheel0):.2f} '
          f'imu={math.degrees(imu0):.2f}')

    # Rotate counterclockwise, keep web command alive, stop at 90deg EKF yaw
    t0 = time.time()
    while True:
        rclpy.spin_once(node, timeout_sec=0.05)
        d = ang_diff(node.ekf_yaw, ekf0)
        post_control(0.0, ANGULAR_CMD)
        if d >= TARGET:
            break
        if time.time() - t0 > 30:
            print('TIMEOUT during rotation')
            break
        time.sleep(0.1)

    post_control(0.0, 0.0)
    time.sleep(0.2)
    post_control(0.0, 0.0)
    print('stop command sent, settling...')
    time.sleep(SETTLE_SEC)
    for _ in range(10):
        rclpy.spin_once(node, timeout_sec=0.1)

    d_ekf = ang_diff(node.ekf_yaw, ekf0)
    d_wheel = ang_diff(node.wheel_yaw, wheel0)
    d_imu = ang_diff(node.imu_yaw, imu0)
    print(f'final: ekf={math.degrees(node.ekf_yaw):.2f} '
          f'wheel={math.degrees(node.wheel_yaw):.2f} '
          f'imu={math.degrees(node.imu_yaw):.2f}')
    print(f'DELTA: ekf={math.degrees(d_ekf):.2f}deg '
          f'wheel={math.degrees(d_wheel):.2f}deg '
          f'imu={math.degrees(d_imu):.2f}deg')
    if abs(d_imu) > 0.05:
        ratio = d_wheel / d_imu
        print(f'wheel/imu ratio: {ratio:.3f}')
        print(f'effective wheel_base estimate: {0.23 * ratio:.3f} m '
              f'(geometric 0.23 m)')
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
