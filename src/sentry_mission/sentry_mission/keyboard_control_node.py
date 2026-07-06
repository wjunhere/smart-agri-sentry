#!/usr/bin/env python3
"""Keyboard control node for chassis movement.

Arrow keys: UP/DOWN linear velocity, LEFT/RIGHT angular velocity.
SPACE: emergency stop.  Q/Ctrl+C: quit.

Reuses the MANUAL mode pattern from web_remote_node: calls /set_auto_mode
to switch to MANUAL, then publishes Twist to /cmd_vel at 20Hz with a 0.5s
safety timeout.
"""

import os
import select
import sys
import termios
import threading
import time
import tty

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_srvs.srv import SetBool


# ---------------------------------------------------------------------------
# keyboard primitives (from servo_keyboard_node.py)
# ---------------------------------------------------------------------------

def _set_raw_mode(fd):
    old = termios.tcgetattr(fd)
    tty.setcbreak(fd)
    return old


def _restore_mode(fd, old):
    termios.tcsetattr(fd, termios.TCSADRAIN, old)


def _getch(fd, timeout=None):
    esc_timeout = 0.5
    if not select.select([sys.stdin], [], [], timeout)[0]:
        return None
    ch = os.read(fd, 1).decode('utf-8', errors='replace')
    if ch != '\x1b':
        return ch
    if select.select([sys.stdin], [], [], esc_timeout)[0]:
        ch += os.read(fd, 1).decode('utf-8', errors='replace')
    if len(ch) == 2 and ch[1] == '[':
        if select.select([sys.stdin], [], [], esc_timeout)[0]:
            ch += os.read(fd, 1).decode('utf-8', errors='replace')
    return ch


def _clamp(value, lo, hi):
    return max(lo, min(hi, value))


# ---------------------------------------------------------------------------
# node
# ---------------------------------------------------------------------------

class KeyboardControlNode(Node):

    def __init__(self):
        super().__init__('keyboard_control_node')

        self.declare_parameter('max_linear', 0.5)
        self.declare_parameter('max_angular', 1.0)
        self.declare_parameter('step_linear', 0.05)
        self.declare_parameter('step_angular', 0.1)

        self.max_linear = self.get_parameter('max_linear').value
        self.max_angular = self.get_parameter('max_angular').value
        self.step_linear = self.get_parameter('step_linear').value
        self.step_angular = self.get_parameter('step_angular').value

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.mode_srv = self.create_client(SetBool, '/set_auto_mode')

        self.mode = 'AUTO'
        self.linear = 0.0
        self.angular = 0.0
        self.lock = threading.Lock()
        self.last_cmd_time = time.time()
        self.TIMEOUT = 0.5

        self.timer = self.create_timer(0.05, self.timer_cb)

    # -- timer ---------------------------------------------------------------

    def timer_cb(self):
        with self.lock:
            now = time.time()
            if self.mode == 'MANUAL' and (now - self.last_cmd_time) > self.TIMEOUT:
                self.linear = 0.0
                self.angular = 0.0
            if self.mode == 'MANUAL':
                twist = Twist()
                twist.linear.x = self.linear
                twist.angular.z = self.angular
                self.cmd_pub.publish(twist)

    # -- mode switching ------------------------------------------------------

    def switch_to_manual(self) -> bool:
        if not self.mode_srv.wait_for_service(timeout_sec=3.0):
            self.get_logger().error('/set_auto_mode service not available')
            return False
        req = SetBool.Request()
        req.data = False
        future = self.mode_srv.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
        if future.result() is None:
            self.get_logger().error('/set_auto_mode call timed out')
            return False
        if future.result().success:
            with self.lock:
                self.mode = 'MANUAL'
                self.linear = 0.0
                self.angular = 0.0
                self.last_cmd_time = time.time()
            self.get_logger().info('Switched to MANUAL mode')
            return True
        self.get_logger().warn(f'/set_auto_mode rejected: {future.result().message}')
        return False

    def switch_to_auto(self):
        if not self.mode_srv.wait_for_service(timeout_sec=3.0):
            self.get_logger().error('/set_auto_mode service not available')
            return
        req = SetBool.Request()
        req.data = True
        future = self.mode_srv.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
        if future.result() is not None and future.result().success:
            with self.lock:
                self.mode = 'AUTO'
            self.get_logger().info('Switched to AUTO mode')

    # -- keyboard loop -------------------------------------------------------

    def run(self):
        fd = sys.stdin.fileno()
        old_term = _set_raw_mode(fd)

        if not self.switch_to_manual():
            _restore_mode(fd, old_term)
            return

        self._print_help()
        try:
            while rclpy.ok():
                ch = _getch(fd, timeout=0.05)
                if ch is not None:
                    self._handle_key(ch)
                rclpy.spin_once(self, timeout_sec=0.001)
        finally:
            _restore_mode(fd, old_term)
            self.switch_to_auto()
            self.get_logger().info('Keyboard control exited')

    def _handle_key(self, ch: str):
        if ch.startswith('\x1b[') and len(ch) == 3:
            key = ch[2]
            if key == 'A':       # UP
                self._adjust_linear(+self.step_linear)
            elif key == 'B':     # DOWN
                self._adjust_linear(-self.step_linear)
            elif key == 'C':     # RIGHT
                self._adjust_angular(+self.step_angular)
            elif key == 'D':     # LEFT
                self._adjust_angular(-self.step_angular)
        elif ch == ' ':          # SPACE = emergency stop
            self._stop()
        elif ch in ('\r', '\n'):  # ENTER = stop (convenience)
            self._stop()
        elif ch.lower() == 'q' or ch == '\x03':  # Q or Ctrl+C
            raise KeyboardInterrupt
        else:
            return  # unrecognised key, no status line reprint

        self._print_status()

    def _adjust_linear(self, delta: float):
        with self.lock:
            self.linear = _clamp(self.linear + delta, -self.max_linear, self.max_linear)
            self.last_cmd_time = time.time()

    def _adjust_angular(self, delta: float):
        with self.lock:
            self.angular = _clamp(self.angular + delta, -self.max_angular, self.max_angular)
            self.last_cmd_time = time.time()

    def _stop(self):
        with self.lock:
            self.linear = 0.0
            self.angular = 0.0
            self.last_cmd_time = time.time()

    # -- display -------------------------------------------------------------

    def _print_help(self):
        self.get_logger().info(
            '  UP/DOWN: linear   LEFT/RIGHT: angular   '
            'SPACE: stop   Q: quit')

    def _print_status(self):
        bar = '+' if self.linear >= 0 else ''
        self.get_logger().info(
            f'  linear: {bar}{self.linear:+.2f} m/s  '
            f'angular: {self.angular:+.2f} rad/s')


# ---------------------------------------------------------------------------
# entry point
# ---------------------------------------------------------------------------

def main(args=None):
    rclpy.init(args=args)
    node = KeyboardControlNode()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
