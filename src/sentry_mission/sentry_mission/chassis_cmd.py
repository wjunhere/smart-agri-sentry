#!/usr/bin/env python3
"""Chassis command-line tool for manual motion testing.

Supports forward, backward, turn-left, turn-right with encoder-based
distance / angle tracking and automatic stop.

Examples:
  # Forward 0.3 m/s for 2 meters
  python chassis_cmd.py --forward 0.3 --dist 2.0

  # Backward 0.3 m/s for 1.5 meters
  python chassis_cmd.py --backward 0.3 --dist 1.5

  # Left turn (pure rotation) at 0.5 rad/s for 90 degrees
  python chassis_cmd.py --turn-left 0.5 --angle 90

  # Right turn at 0.3 rad/s for 45 degrees
  python chassis_cmd.py --turn-right 0.3 --angle 45

  # Arc turn: forward 0.3 m/s while turning left at 0.3 rad/s for 2 m arc
  python chassis_cmd.py --forward 0.3 --angular 0.3 --dist 2.0

  # Stop immediately
  python chassis_cmd.py --stop
"""

import argparse
import math
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import Twist
from sentry_interfaces.msg import ChassisStatus


# Chassis constants
PULSES_PER_METER = 11035.0
WHEEL_BASE = 0.23   # meters between tracks
ENCODER_TIMEOUT_SEC = 2.0


def _rad_to_deg(rad: float) -> float:
    return rad * 180.0 / math.pi


def _deg_to_rad(deg: float) -> float:
    return deg * math.pi / 180.0


class ChassisController:
    """Publish /cmd_vel and track encoder feedback for automatic stop."""

    def __init__(self):
        self.node = Node('chassis_cmd')
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.pub = self.node.create_publisher(Twist, '/sentry/cmd_vel', 10)
        self.last_status = None
        self.sub = self.node.create_subscription(
            ChassisStatus, '/sentry/chassis/status', self._on_status, qos)
        # Let first message arrive
        self._spin_until_status(timeout=3.0)

    def _on_status(self, msg: ChassisStatus):
        self.last_status = msg

    def _spin_until_status(self, timeout: float):
        """Block until first chassis status message arrives."""
        t0 = time.time()
        while self.last_status is None and time.time() - t0 < timeout:
            rclpy.spin_once(self.node, timeout_sec=0.05)
        if self.last_status is None:
            self.node.get_logger().error(
                'No chassis status received — is uart_bridge_node running?')

    def _spin(self, sec: float):
        for _ in range(int(sec * 100)):
            rclpy.spin_once(self.node, timeout_sec=0.01)

    def stop(self):
        """Publish zero velocity and spin once to flush."""
        self.pub.publish(Twist())
        self._spin(0.2)
        self.node.get_logger().info('Motors stopped')

    def _read_encoders(self):
        """Return (left_pulse, right_pulse, timestamp_ms) or (None, None, None)."""
        s = self.last_status
        if s is None or s.comm_timeout:
            return None, None, None
        return s.left_pulse, s.right_pulse, s.encoder_timestamp

    # ---- Straight-line motion ----

    def move_straight(self, speed_m_s: float, distance_m: float,
                      timeout_sec: float = 30.0) -> bool:
        """Drive straight until target distance reached.

        Args:
            speed_m_s: Positive = forward, negative = backward.
            distance_m: Target distance in meters (> 0).
            timeout_sec: Safety timeout.

        Returns:
            True if target distance reached, False on timeout.
        """
        direction = 'FORWARD' if speed_m_s > 0 else 'BACKWARD'
        target = abs(distance_m)

        # Read start
        self._spin(0.3)
        l0, r0, _ = self._read_encoders()
        if l0 is None:
            self.node.get_logger().error('Cannot read encoders — abort')
            return False

        msg = Twist()
        msg.linear.x = speed_m_s
        msg.angular.z = 0.0

        self.node.get_logger().info(
            f'{direction} {abs(speed_m_s):.2f} m/s × {target:.2f} m')
        self.node.get_logger().info(
            f'  Start pulses: L={l0} R={r0}')

        t0 = time.time()
        last_print = t0
        while time.time() - t0 < timeout_sec:
            self.pub.publish(msg)
            self._spin(0.01)

            l, r, _ = self._read_encoders()
            if l is None:
                continue

            dist_l = abs((l - l0) / PULSES_PER_METER)
            dist_r = abs((r - r0) / PULSES_PER_METER)
            avg_dist = (dist_l + dist_r) / 2.0

            elapsed = time.time() - t0
            if elapsed - last_print >= 0.5:
                spd_l = dist_l / elapsed if elapsed > 0 else 0
                spd_r = dist_r / elapsed if elapsed > 0 else 0
                self.node.get_logger().info(
                    f'  {elapsed:.1f}s: L={dist_l:.3f}m ({spd_l:.2f}m/s)  '
                    f'R={dist_r:.3f}m ({spd_r:.2f}m/s)  '
                    f'avg={avg_dist:.3f}m / {target:.2f}m')
                last_print = elapsed

            if avg_dist >= target:
                break

        self.stop()

        # Final read
        l, r, _ = self._read_encoders()
        dist_l = abs((l - l0) / PULSES_PER_METER) if l is not None else 0
        dist_r = abs((r - r0) / PULSES_PER_METER) if r is not None else 0
        avg_dist = (dist_l + dist_r) / 2.0
        elapsed = time.time() - t0
        err_pct = (avg_dist - target) / target * 100.0 if target > 0 else 0

        self.node.get_logger().info(
            f'{direction} DONE: {elapsed:.2f}s  '
            f'L={dist_l:.3f}m  R={dist_r:.3f}m  '
            f'avg={avg_dist:.3f}m vs {target:.2f}m ({err_pct:+.1f}%)')
        return True

    # ---- Pure rotation (turn in place) ----

    def turn_in_place(self, angular_rad_s: float, angle_deg: float,
                      timeout_sec: float = 30.0) -> bool:
        """Rotate in place until target angle reached.

        Args:
            angular_rad_s: Positive = CCW (left turn), negative = CW (right turn).
            angle_deg: Target angle in degrees (> 0, always positive).
            timeout_sec: Safety timeout.

        Returns:
            True if target angle reached, False on timeout.
        """
        direction = 'LEFT (CCW)' if angular_rad_s > 0 else 'RIGHT (CW)'
        target_rad = _deg_to_rad(angle_deg)

        self._spin(0.3)
        l0, r0, _ = self._read_encoders()
        if l0 is None:
            self.node.get_logger().error('Cannot read encoders — abort')
            return False

        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = angular_rad_s

        self.node.get_logger().info(
            f'TURN {direction}  {abs(angular_rad_s):.2f} rad/s × {angle_deg:.0f}°')
        self.node.get_logger().info(
            f'  Start pulses: L={l0} R={r0}')

        t0 = time.time()
        last_print = t0
        while time.time() - t0 < timeout_sec:
            self.pub.publish(msg)
            self._spin(0.01)

            l, r, _ = self._read_encoders()
            if l is None:
                continue

            # Angle = (right_dist - left_dist) / wheel_base
            dist_l = (l - l0) / PULSES_PER_METER
            dist_r = (r - r0) / PULSES_PER_METER
            angle_rad = (dist_r - dist_l) / WHEEL_BASE
            angle_deg_cur = _rad_to_deg(abs(angle_rad))

            elapsed = time.time() - t0
            if elapsed - last_print >= 0.5:
                self.node.get_logger().info(
                    f'  {elapsed:.1f}s:  ΔL={dist_l:+.3f}m  ΔR={dist_r:+.3f}m  '
                    f'angle={_rad_to_deg(angle_rad):+.1f}° / {angle_deg:.0f}°')
                last_print = elapsed

            if abs(angle_rad) >= target_rad:
                break

        self.stop()

        # Final read
        l, r, _ = self._read_encoders()
        dist_l = (l - l0) / PULSES_PER_METER if l is not None else 0
        dist_r = (r - r0) / PULSES_PER_METER if r is not None else 0
        angle_rad = (dist_r - dist_l) / WHEEL_BASE
        angle_final = _rad_to_deg(abs(angle_rad))
        elapsed = time.time() - t0
        err_pct = (angle_final - angle_deg) / angle_deg * 100.0 if angle_deg > 0 else 0

        self.node.get_logger().info(
            f'TURN {direction} DONE: {elapsed:.2f}s  '
            f'ΔL={dist_l:+.3f}m  ΔR={dist_r:+.3f}m  '
            f'angle={_rad_to_deg(angle_rad):+.1f}° vs {angle_deg:.0f}° ({err_pct:+.1f}%)')

        # Also show raw encoder deltas for debugging
        self.node.get_logger().info(
            f'  Raw pulses: L: {l0}→{l} ({l-l0:+d})  '
            f'R: {r0}→{r} ({r-r0:+d})')
        return True

    # ---- Arc turn (forward + rotation) ----

    def arc_turn(self, speed_m_s: float, angular_rad_s: float,
                 distance_m: float, timeout_sec: float = 30.0) -> bool:
        """Drive in an arc with both linear and angular velocity.

        Args:
            speed_m_s: Linear speed (> 0).
            angular_rad_s: Angular speed (positive = left, negative = right).
            distance_m: Arc length to travel.
            timeout_sec: Safety timeout.
        """
        self._spin(0.3)
        l0, r0, _ = self._read_encoders()
        if l0 is None:
            self.node.get_logger().error('Cannot read encoders — abort')
            return False

        msg = Twist()
        msg.linear.x = speed_m_s
        msg.angular.z = angular_rad_s

        turn_dir = 'LEFT' if angular_rad_s > 0 else 'RIGHT'
        self.node.get_logger().info(
            f'ARC {turn_dir}  v={speed_m_s:.2f} m/s  '
            f'ω={abs(angular_rad_s):.2f} rad/s  arc_len={distance_m:.2f} m')
        self.node.get_logger().info(
            f'  Start pulses: L={l0} R={r0}')

        t0 = time.time()
        last_print = t0
        while time.time() - t0 < timeout_sec:
            self.pub.publish(msg)
            self._spin(0.01)

            l, r, _ = self._read_encoders()
            if l is None:
                continue

            dist_l = (l - l0) / PULSES_PER_METER
            dist_r = (r - r0) / PULSES_PER_METER
            avg_dist = (abs(dist_l) + abs(dist_r)) / 2.0

            elapsed = time.time() - t0
            if elapsed - last_print >= 0.5:
                angle_rad = (dist_r - dist_l) / WHEEL_BASE
                self.node.get_logger().info(
                    f'  {elapsed:.1f}s:  dist={avg_dist:.3f}m / {distance_m:.2f}m  '
                    f'angle={_rad_to_deg(angle_rad):+.1f}°')
                last_print = elapsed

            if avg_dist >= distance_m:
                break

        self.stop()

        l, r, _ = self._read_encoders()
        dist_l = (l - l0) / PULSES_PER_METER if l is not None else 0
        dist_r = (r - r0) / PULSES_PER_METER if r is not None else 0
        avg_dist = (abs(dist_l) + abs(dist_r)) / 2.0
        angle_rad = (dist_r - dist_l) / WHEEL_BASE
        elapsed = time.time() - t0

        self.node.get_logger().info(
            f'ARC {turn_dir} DONE: {elapsed:.2f}s  '
            f'dist={avg_dist:.3f}m  angle={_rad_to_deg(angle_rad):+.1f}°')
        return True

    def destroy(self):
        self.node.destroy_node()


def main():
    parser = argparse.ArgumentParser(
        description='Chassis command-line tool for manual motion testing')
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument('--forward', type=float, metavar='M/S',
                       help='Forward speed in m/s')
    group.add_argument('--backward', type=float, metavar='M/S',
                       help='Backward speed in m/s')
    group.add_argument('--turn-left', type=float, metavar='RAD/S',
                       help='Left turn (CCW) angular speed in rad/s')
    group.add_argument('--turn-right', type=float, metavar='RAD/S',
                       help='Right turn (CW) angular speed in rad/s')
    group.add_argument('--stop', action='store_true',
                       help='Stop motors immediately')

    parser.add_argument('--dist', type=float, default=None,
                        help='Target distance in meters (for forward/backward)')
    parser.add_argument('--angle', type=float, default=90.0,
                        help='Target angle in degrees (for turns, default: 90)')
    parser.add_argument('--angular', type=float, default=None,
                        help='Angular velocity for arc turn (rad/s)')
    parser.add_argument('--timeout', type=float, default=30.0,
                        help='Safety timeout in seconds (default: 30)')

    args = parser.parse_args()

    rclpy.init(args=sys.argv)
    ctrl = ChassisController()

    try:
        if args.stop:
            ctrl.stop()
            print('STOP sent')
            return

        if args.forward is not None:
            speed = args.forward
            if args.angular is not None:
                # Arc turn
                if args.dist is None:
                    print('ERROR: --dist required for arc turn')
                    return
                ctrl.arc_turn(speed, args.angular, args.dist, args.timeout)
            else:
                if args.dist is None:
                    print('ERROR: --dist required for forward')
                    return
                ctrl.move_straight(speed, args.dist, args.timeout)

        elif args.backward is not None:
            speed = -abs(args.backward)
            if args.dist is None:
                print('ERROR: --dist required for backward')
                return
            ctrl.move_straight(speed, args.dist, args.timeout)

        elif args.turn_left is not None:
            ctrl.turn_in_place(abs(args.turn_left), args.angle, args.timeout)

        elif args.turn_right is not None:
            ctrl.turn_in_place(-abs(args.turn_right), args.angle, args.timeout)

    finally:
        ctrl.destroy()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
