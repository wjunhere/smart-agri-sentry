#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
import math

import rclpy
from rclpy.node import Node
from rclpy.clock import Clock
from sensor_msgs.msg import Imu, MagneticField
from std_msgs.msg import Float32MultiArray

# External library: YbImuLib must be installed on target platform
from YbImuLib import YbImuSerial


class ImuNode(Node):
    """YB-IMU sensor driver for Smart Agri Sentry."""

    def __init__(self):
        super().__init__('imu_node')
        self.robot = None

        # Declare parameters
        self.declare_parameter('port', '/dev/myimu')
        self.declare_parameter('frame_id', 'imu_link')
        self.declare_parameter('pub_rate_hz', 100.0)
        self.declare_parameter('use_mag', True)
        self.declare_parameter(
            'linear_accel_cov',
            [0.0005, 0.0, 0.0, 0.0, 0.0005, 0.0, 0.0, 0.0, 0.0008]
        )
        self.declare_parameter(
            'angular_vel_cov',
            [0.00002, 0.0, 0.0, 0.0, 0.00002, 0.0, 0.0, 0.0, 0.00005]
        )
        self.declare_parameter(
            'orientation_cov',
            [0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.2]
        )

        self.port = self.get_parameter('port').value
        self.frame_id = self.get_parameter('frame_id').value
        self.pub_rate_hz = self.get_parameter('pub_rate_hz').value
        self.use_mag = self.get_parameter('use_mag').value

        self.linear_accel_cov = self._build_covariance_matrix(
            self.get_parameter('linear_accel_cov').value
        )
        self.angular_vel_cov = self._build_covariance_matrix(
            self.get_parameter('angular_vel_cov').value
        )
        self.orientation_cov = self._build_covariance_matrix(
            self.get_parameter('orientation_cov').value
        )

        self._init_serial()
        if self.robot is None:
            self.get_logger().error('Failed to initialize IMU serial port')
            return

        self._init_publishers()
        self._init_timer()

    @staticmethod
    def _normalize_quaternion_static(q):
        """Normalize a quaternion. Returns unit quaternion if input is zero."""
        norm = math.sqrt(sum(x * x for x in q))
        if norm < 1e-6:
            return [1.0, 0.0, 0.0, 0.0]
        return [x / norm for x in q]

    @staticmethod
    def _build_covariance_matrix(flat_list):
        """Build a 9-element covariance list from flat input."""
        if len(flat_list) != 9:
            return [0.0] * 9
        return list(flat_list)

    def _init_serial(self):
        """Initialize serial connection to IMU."""
        try:
            self.robot = YbImuSerial(self.port)
            self.get_logger().info(f'Opened IMU serial port: {self.port}')
            self._patch_ch340_read()
            self.robot.create_receive_threading()
        except Exception as e:
            self.get_logger().error(
                f'Failed to open IMU serial port {self.port}: {e}'
            )
            self.robot = None

    def _patch_ch340_read(self):
        """Monkey-patch serial read to handle CH340 driver quirks on ARM.

        The CH340 driver on RDK X5 (ARM Linux) sometimes reports data
        available via in_waiting but returns 0 bytes on read().  Replace
        read_all() with a version that catches this and retries with a
        fixed-size read as fallback.
        """
        import serial as _serial
        dev = self.robot._dev
        _orig_read_all = dev.read_all

        def _safe_read_all():
            for _ in range(3):
                try:
                    return _orig_read_all()
                except _serial.SerialException:
                    # CH340 lied about in_waiting; try a small fixed read
                    try:
                        chunk = dev.read(64)
                        if chunk:
                            return chunk
                    except _serial.SerialException:
                        pass
            return b''

        dev.read_all = _safe_read_all
        self.get_logger().info('CH340 read_all patch applied')

    def _init_publishers(self):
        self.imu_publisher = self.create_publisher(
            Imu, '/sensor/imu/data_raw', 100
        )
        self.mag_publisher = self.create_publisher(
            MagneticField, '/sensor/imu/mag', 100
        )
        self.baro_publisher = self.create_publisher(
            Float32MultiArray, '/sensor/imu/baro', 100
        )
        self.euler_publisher = self.create_publisher(
            Float32MultiArray, '/sensor/imu/euler', 100
        )

    def _init_timer(self):
        period = 1.0 / self.pub_rate_hz
        self.timer = self.create_timer(period, self._pub_data)
        self.get_logger().info(
            f'IMU publisher timer started at {self.pub_rate_hz} Hz'
        )

    def _pub_data(self):
        if self.robot is None:
            return

        time_stamp = Clock().now()
        imu = Imu()
        mag = MagneticField()
        baro = Float32MultiArray()
        euler = Float32MultiArray()

        try:
            [ax, ay, az] = self.robot.get_accelerometer_data()
            [gx, gy, gz] = self.robot.get_gyroscope_data()
            [mx, my, mz] = self.robot.get_magnetometer_data()
            [q0, q1, q2, q3] = self.robot.get_imu_quaternion_data()
            [height, temperature, pressure, pressure_contrast] = self.robot.get_baro_data()
            [roll, pitch, yaw] = self.robot.get_imu_attitude_data(True)
        except Exception as e:
            self.get_logger().warning(f'Failed to read IMU data: {e}')
            return

        # Normalize quaternion and warn if significantly off
        q = self._normalize_quaternion_static([q0, q1, q2, q3])
        norm_before = math.sqrt(sum(x * x for x in [q0, q1, q2, q3]))
        if abs(norm_before - 1.0) > 0.01:
            self.get_logger().warning(
                f'Quaternion norm deviated: {norm_before:.4f}, normalized'
            )

        # Fill IMU message
        imu.header.stamp = time_stamp.to_msg()
        imu.header.frame_id = self.frame_id
        imu.linear_acceleration.x = float(ax)
        imu.linear_acceleration.y = float(ay)
        imu.linear_acceleration.z = float(az)
        imu.linear_acceleration_covariance = self.linear_accel_cov
        imu.angular_velocity.x = float(gx)
        imu.angular_velocity.y = float(gy)
        imu.angular_velocity.z = float(gz)
        imu.angular_velocity_covariance = self.angular_vel_cov
        imu.orientation.w = q[0]
        imu.orientation.x = q[1]
        imu.orientation.y = q[2]
        imu.orientation.z = q[3]
        imu.orientation_covariance = self.orientation_cov

        # Fill magnetometer message
        mag.header.stamp = time_stamp.to_msg()
        mag.header.frame_id = self.frame_id
        mag.magnetic_field.x = float(mx)
        mag.magnetic_field.y = -float(my)  # Y-axis sign flip for ENU convention
        mag.magnetic_field.z = float(mz)

        # Fill barometer and euler messages
        baro.data = [float(height), float(temperature),
                     float(pressure), float(pressure_contrast)]
        euler.data = [float(roll), float(pitch), float(yaw)]

        self.imu_publisher.publish(imu)
        if self.use_mag:
            self.mag_publisher.publish(mag)
        self.baro_publisher.publish(baro)
        self.euler_publisher.publish(euler)


def main(args=None):
    rclpy.init(args=args)
    node = ImuNode()
    if node.robot is None:
        node.destroy_node()
        rclpy.shutdown()
        return 1
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
