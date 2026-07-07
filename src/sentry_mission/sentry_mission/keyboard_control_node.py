#!/usr/bin/env python3
"""Keyboard control node for chassis movement.

Arrow keys: UP/DOWN linear velocity, LEFT/RIGHT angular velocity.
SPACE: emergency stop.  Q/Ctrl+C: quit.

Publishes Twist directly to /cmd_vel.  uart_bridge_node is the sole
subscriber and converts Twist → differential drive → UART frame → STM32.
No dependency on mission_control_node; if it is running we try to switch
it to MANUAL as a best-effort to avoid conflicting /cmd_vel publishers.
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
        self.declare_parameter('step_angular', 0.05)

        self.max_linear = self.get_parameter('max_linear').value
        self.max_angular = self.get_parameter('max_angular').value
        self.step_linear = self.get_parameter('step_linear').value
        self.step_angular = self.get_parameter('step_angular').value

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.linear = 0.0
        self.angular = 0.0
        self.mode = 'MANUAL'  # start in MANUAL, publish directly
        self.lock = threading.Lock()
        self.last_cmd_time = time.time()
        self.TIMEOUT = 0.5
        self._had_mission_control = False

        self.timer = self.create_timer(0.05, self.timer_cb)

    # -- timer ---------------------------------------------------------------

    def timer_cb(self):
        """Publish Twist only in MANUAL mode to avoid conflicting with Nav2."""
        with self.lock:
            now = time.time()
            if (now - self.last_cmd_time) > self.TIMEOUT:
                self.linear = 0.0
                self.angular = 0.0
            if self.mode == 'MANUAL':
                twist = Twist()
                twist.linear.x = self.linear
                twist.angular.z = self.angular
                self.cmd_pub.publish(twist)

    # -- optional mode switching ---------------------------------------------

    def _try_disable_mission_control(self):
        """Best-effort: if mission_control_node is running, switch to MANUAL
        so it stops publishing its own /cmd_vel messages."""
        client = self.create_client(SetBool, '/set_auto_mode')
        if not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(
                '/set_auto_mode not available — publishing /cmd_vel directly')
            return
        req = SetBool.Request()
        req.data = False
        future = client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=1.0)
        if future.result() is not None and future.result().success:
            self._had_mission_control = True
            self.get_logger().info('mission_control_node switched to MANUAL')
        else:
            self.get_logger().info(
                'mission_control_node not running — publishing /cmd_vel directly')
        self.destroy_client(client)

    def _try_restore_mission_control(self):
        if not self._had_mission_control:
            return
        client = self.create_client(SetBool, '/set_auto_mode')
        if not client.wait_for_service(timeout_sec=1.0):
            return
        req = SetBool.Request()
        req.data = True
        future = client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=1.0)
        self.destroy_client(client)

    # -- keyboard loop -------------------------------------------------------

    def run(self):
        fd = sys.stdin.fileno()
        old_term = _set_raw_mode(fd)

        self._try_disable_mission_control()

        self._print_help()
        try:
            while rclpy.ok():
                ch = _getch(fd, timeout=0.05)
                if ch is not None:
                    self._handle_key(ch)
                rclpy.spin_once(self, timeout_sec=0.001)
        finally:
            _restore_mode(fd, old_term)
            self._try_restore_mission_control()
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
            return

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
