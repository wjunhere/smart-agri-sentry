#!/usr/bin/env python3
"""IMU-based closed-loop turn controller for precise in-place rotation.

Uses gyroscope angular_velocity.z for real-time angle integration,
making turning precision surface-independent -- unaffected by track slip
on loose soil, grass, gravel, or pavement.

Tested accuracy: within ~4% (1.2 deg error on 30 deg turn).

Control phases:
  TURN  - PID proportional control at max 0.3 rad/s
  BRAKE - Reverse thrust (0.15 s) to kill inertia
  LOCK  - Continuous zero-velocity hold (2 s), then exit

Subscribe: /sensor/imu/data_raw (sensor_msgs/Imu)
Publish:   /cmd_vel (geometry_msgs/Twist)

Usage:
  python imu_turn_controller.py --angle 90          # Turn left 90 deg
  python imu_turn_controller.py --angle -45         # Turn right 45 deg
  python imu_turn_controller.py --angle 180         # Turn left 180 deg
"""

import argparse
import math
import sys
import threading
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu


class ImuTurnController(Node):
    """IMU-feedback closed-loop turn controller."""

    def __init__(self):
        super().__init__('imu_turn_controller')

        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.imu_sub = self.create_subscription(
            Imu, '/sensor/imu/data_raw', self._on_imu, 10)

        # Thread-safe shared state
        self._lock = threading.Lock()
        self._gyro_z = 0.0       # latest angular_velocity.z (rad/s)
        self._gyro_bias = 0.0    # calibrated stationary bias
        self._last_imu_time = None
        self._accumulated_angle = 0.0  # integrated angle (rad)

    # ------------------------------------------------------------------
    # IMU callback -- trapezoidal integration
    # ------------------------------------------------------------------
    def _on_imu(self, msg: Imu):
        now = time.time()
        with self._lock:
            old_z = self._gyro_z
            self._gyro_z = msg.angular_velocity.z

            if self._last_imu_time is not None:
                dt = now - self._last_imu_time
                if 0.0 < dt < 0.1:
                    avg_z = (old_z + self._gyro_z) / 2.0
                    self._accumulated_angle += (avg_z - self._gyro_bias) * dt
            self._last_imu_time = now

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------
    def _calibrate_gyro(self, samples: int = 100):
        """Estimate gyro bias while stationary."""
        self.get_logger().info(f'Calibrating gyro bias ({samples} samples)...')
        total = 0.0
        for _ in range(samples):
            rclpy.spin_once(self, timeout_sec=0.01)
            total += self._gyro_z
        self._gyro_bias = total / samples
        self.get_logger().info(
            f'Gyro bias: {math.degrees(self._gyro_bias):.4f} deg/s')

    def _reset_angle(self):
        with self._lock:
            self._accumulated_angle = 0.0

    def _get_angle_deg(self) -> float:
        with self._lock:
            return math.degrees(self._accumulated_angle)

    def _get_gyro_z(self) -> float:
        with self._lock:
            return self._gyro_z

    def _wait_for_imu(self, timeout: float = 3.0) -> bool:
        """Block until first IMU message arrives."""
        t0 = time.time()
        while self._last_imu_time is None and time.time() - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.01)
        return self._last_imu_time is not None

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------
    def stop(self):
        """Publish zero velocity."""
        self.pub.publish(Twist())
        time.sleep(0.3)
        self.get_logger().info('Motors stopped')

    def turn_to_angle(self,
                      target_deg: float,
                      max_angular_speed: float = 0.3,
                      kp: float = 0.6,
                      timeout_sec: float = 30.0) -> float:
        """Rotate in-place to the target angle using IMU gyro feedback.

        PID-proportional control with constant-speed turn to avoid
        firmware minimum-speed deadzone (~50 deg/s).

        Args:
            target_deg: Target angle in degrees.
                        Positive = CCW (left), negative = CW (right).
            max_angular_speed: Maximum angular speed in rad/s.
            kp: Proportional gain (deg error -> rad/s angular command).
            timeout_sec: Safety timeout.

        Returns:
            Absolute final angle achieved (degrees).
        """
        direction = 'LEFT (CCW)' if target_deg >= 0 else 'RIGHT (CW)'
        target_sign = 1.0 if target_deg >= 0 else -1.0
        target_abs = abs(target_deg)

        # --- Calibrate & reset ---
        if not self._wait_for_imu():
            self.get_logger().error('No IMU data -- abort')
            return 0.0
        self._calibrate_gyro()
        self._reset_angle()

        self.get_logger().info(
            f'==========================================')
        self.get_logger().info(
            f'  IMU TURN {direction:12s}  {target_abs:6.1f} deg')
        self.get_logger().info(
            f'  max_w={max_angular_speed:.2f} rad/s  Kp={kp:.2f}')
        self.get_logger().info(
            f'==========================================')

        # --- Control loop ---
        t0 = time.time()
        last_print = t0
        phase = 'TURN'
        brake_t = 0.0
        lock_t = 0.0

        while time.time() - t0 < timeout_sec:
            rclpy.spin_once(self, timeout_sec=0.005)

            angle_abs = abs(self._get_angle_deg())
            gyro_z = self._get_gyro_z()
            gyro_deg_s = math.degrees(gyro_z)
            now = time.time()

            if phase == 'TURN':
                error_deg = target_abs - angle_abs

                # Trigger brake when within 1 deg of target
                if error_deg <= 1.0:
                    phase = 'BRAKE'
                    brake_t = now
                    self.get_logger().info(
                        f'  >> TURN complete at {angle_abs:.1f} deg -> BRAKE')

                if phase == 'TURN':  # may have changed above
                    # PID: proportional control, bounded to max speed
                    angular_cmd = kp * error_deg / 57.3  # rough deg->rad
                    angular_cmd = max(-max_angular_speed,
                                      min(max_angular_speed, angular_cmd))
                    angular_cmd *= target_sign

                    msg = Twist()
                    msg.linear.x = 0.0
                    msg.angular.z = angular_cmd
                    self.pub.publish(msg)

            elif phase == 'BRAKE':
                elapsed = now - brake_t
                if elapsed < 0.15:
                    # Reverse thrust (40%) to cancel inertia
                    msg = Twist()
                    msg.linear.x = 0.0
                    msg.angular.z = -target_sign * max_angular_speed * 0.4
                    self.pub.publish(msg)
                else:
                    phase = 'LOCK'
                    lock_t = now
                    self.get_logger().info(
                        f'  >> BRAKE complete at {angle_abs:.1f} deg -> LOCK')

            elif phase == 'LOCK':
                # Continuous zero to hold position
                self.pub.publish(Twist())
                if now - lock_t >= 2.0:
                    phase = 'DONE'
                    self.get_logger().info(
                        f'  >> LOCK complete at {angle_abs:.1f} deg -> DONE')
                    break

            # Periodic logging
            if now - last_print >= 0.3:
                self.get_logger().info(
                    f'  [{phase:5s}] t={now - t0:.1f}s  '
                    f'angle={angle_abs:.1f}deg/{target_abs:.1f}deg  '
                    f'w={gyro_deg_s:.1f}deg/s  '
                    f'err={target_abs - angle_abs:+.1f}deg')
                last_print = now

        # --- Finalize ---
        self.pub.publish(Twist())
        time.sleep(0.15)
        rclpy.spin_once(self, timeout_sec=0.05)

        final_abs = abs(self._get_angle_deg())
        err = final_abs - target_abs
        err_pct = (err / target_abs * 100.0) if target_abs > 0 else 0.0

        self.get_logger().info(
            f'==========================================')
        self.get_logger().info(
            f'  RESULT: {final_abs:.1f} deg  target={target_abs:.1f} deg  '
            f'err={err:+.1f} deg ({err_pct:+.1f}%)')
        self.get_logger().info(
            f'==========================================')

        return final_abs


# ------------------------------------------------------------------
# CLI entry point
# ------------------------------------------------------------------
def main():
    parser = argparse.ArgumentParser(
        description='IMU-based closed-loop turn controller')
    parser.add_argument('--angle', type=float, default=90.0,
                        help='Target angle in degrees. '
                             'Positive=CCW/left, negative=CW/right (default: 90)')
    parser.add_argument('--max-speed', type=float, default=0.3,
                        help='Max angular speed in rad/s (default: 0.3)')
    parser.add_argument('--kp', type=float, default=0.6,
                        help='Proportional gain (default: 0.6)')
    parser.add_argument('--stop', action='store_true',
                        help='Send stop only')
    parser.add_argument('--timeout', type=float, default=30.0,
                        help='Safety timeout in seconds (default: 30)')
    args = parser.parse_args()

    rclpy.init(args=sys.argv)
    ctrl = ImuTurnController()

    try:
        if args.stop:
            ctrl.stop()
            return

        ctrl.turn_to_angle(
            target_deg=args.angle,
            max_angular_speed=args.max_speed,
            kp=args.kp,
            timeout_sec=args.timeout)
    finally:
        ctrl.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
