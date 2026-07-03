#!/usr/bin/env python3
"""Wheel encoder odometry node.

Subscribes to /sentry/chassis/status for encoder pulses,
computes differential-drive dead reckoning, publishes /wheel/odom.
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
from sentry_interfaces.msg import ChassisStatus
import math


class WheelOdomNode(Node):
    def __init__(self):
        super().__init__('wheel_odom_node')
        self.declare_parameter('wheel_base', 0.23)
        self.declare_parameter('pulses_per_meter', 11035)
        self.declare_parameter('max_pulse_delta', 1000)

        self.wheel_base = self.get_parameter('wheel_base').value
        self.pulses_per_m = self.get_parameter('pulses_per_meter').value
        self.MAX_PULSE_DELTA = self.get_parameter('max_pulse_delta').value

        self.sub = self.create_subscription(
            ChassisStatus, '/sentry/chassis/status', self.on_chassis, 10)
        self.pub = self.create_publisher(Odometry, '/wheel/odom', 10)

        self.last_left = None
        self.last_right = None
        self.last_time = None
        self.x = self.y = self.theta = 0.0
        self.last_timeout_log_time = None

        # Pose covariance (empirical)
        self.pose_cov = [
            0.01, 0.0,  0.0,  0.0,  0.0,  0.0,
            0.0,  0.01, 0.0,  0.0,  0.0,  0.0,
            0.0,  0.0,  0.01, 0.0,  0.0,  0.0,
            0.0,  0.0,  0.0,  0.0,  0.0,  0.0,
            0.0,  0.0,  0.0,  0.0,  0.0,  0.0,
            0.0,  0.0,  0.0,  0.0,  0.0,  0.01,
        ]
        # Twist covariance
        self.twist_cov = [
            0.01, 0.0,  0.0,  0.0,  0.0,  0.0,
            0.0,  0.01, 0.0,  0.0,  0.0,  0.0,
            0.0,  0.0,  0.0,  0.0,  0.0,  0.0,
            0.0,  0.0,  0.0,  0.0,  0.0,  0.0,
            0.0,  0.0,  0.0,  0.0,  0.0,  0.0,
            0.0,  0.0,  0.0,  0.0,  0.0,  0.01,
        ]

        self.get_logger().info(
            f'Wheel odom ready: wheel_base={self.wheel_base}, '
            f'pulses_per_m={self.pulses_per_m}')

    def on_chassis(self, msg: ChassisStatus):
        if msg.comm_timeout:
            now = self.get_clock().now()
            if (self.last_timeout_log_time is None or
                    (now - self.last_timeout_log_time).nanoseconds > 5e9):
                self.get_logger().warning(
                    'Chassis communication timeout, skipping odometry update')
                self.last_timeout_log_time = now
            return

        self.last_timeout_log_time = None

        left_pulse = msg.left_pulse
        right_pulse = msg.right_pulse

        # Skip if no encoder data yet (old firmware or not initialized)
        if left_pulse == 0 and right_pulse == 0 and self.last_left is None:
            return

        now = self.get_clock().now()

        if self.last_left is None:
            self.last_left = left_pulse
            self.last_right = right_pulse
            self.last_time = now
            return

        # Pulse jump detection
        d_left = left_pulse - self.last_left
        d_right = right_pulse - self.last_right
        if (abs(d_left) > self.MAX_PULSE_DELTA or
                abs(d_right) > self.MAX_PULSE_DELTA):
            self.get_logger().warn(
                f'Pulse jump: left={d_left}, right={d_right}, skipping')
            self.last_left = left_pulse
            self.last_right = right_pulse
            self.last_time = now
            return

        dt = (now - self.last_time).nanoseconds / 1e9
        if dt <= 0:
            dt = 0.05
        self.last_time = now
        self.last_left = left_pulse
        self.last_right = right_pulse

        dl = d_left / self.pulses_per_m
        dr = d_right / self.pulses_per_m
        d_center = (dl + dr) / 2.0
        d_theta = (dr - dl) / self.wheel_base

        self.theta += d_theta
        self.x += d_center * math.cos(self.theta)
        self.y += d_center * math.sin(self.theta)

        linear = d_center / dt
        angular = d_theta / dt

        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        q = Quaternion()
        q.z = math.sin(self.theta / 2.0)
        q.w = math.cos(self.theta / 2.0)
        odom.pose.pose.orientation = q
        odom.pose.covariance = self.pose_cov

        odom.twist.twist.linear.x = linear
        odom.twist.twist.angular.z = angular
        odom.twist.covariance = self.twist_cov

        self.pub.publish(odom)


def main(args=None):
    rclpy.init(args=args)
    node = WheelOdomNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
