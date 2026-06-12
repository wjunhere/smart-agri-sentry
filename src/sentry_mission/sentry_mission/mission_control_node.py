#!/usr/bin/env python3
"""Mission control node with Nav2 waypoint navigation + visual servoing.

States:
  PATROL      - Nav2 waypoint cruising
  APPROACHING - Visual servoing toward detected plant
  STOPPED     - Brief pause before analysis
  ANALYZING   - Wait for fusion diagnosis result
  ACTION      - Record the diagnosis result
  RESUME      - Brief pause before resuming patrol
  MANUAL      - Web remote control mode, Nav2 paused
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist, Quaternion
from nav_msgs.msg import Odometry
from nav2_simple_commander.robot_navigator import BasicNavigator
from sentry_interfaces.msg import (
    PlantDetection, FusionResult, MissionStatus)
from std_msgs.msg import Bool
from std_srvs.srv import SetBool
import yaml
import math


STATE_PATROL = 'PATROL'
STATE_APPROACHING = 'APPROACHING'
STATE_STOPPED = 'STOPPED'
STATE_ANALYZING = 'ANALYZING'
STATE_ACTION = 'ACTION'
STATE_RESUME = 'RESUME'
STATE_MANUAL = 'MANUAL'


class MissionControlNode(Node):
    def __init__(self):
        super().__init__('mission_control_node')

        # -- Parameters --
        self.declare_parameter('cruise_speed', 0.3)
        self.declare_parameter('approach_speed', 0.15)
        self.declare_parameter('detection_confidence_threshold', 0.6)
        self.declare_parameter('min_area_ratio', 0.1)
        self.declare_parameter('stop_distance_tolerance', 0.05)
        self.declare_parameter('analyze_timeout_sec', 5.0)
        self.declare_parameter('resume_delay_sec', 2.0)
        self.declare_parameter('bbox_center_tolerance', 0.15)
        self.declare_parameter('waypoints_file', '')
        self.declare_parameter('wheel_base', 0.4)
        self.declare_parameter('pulses_per_meter', 1000)

        self.cruise_speed = self.get_parameter('cruise_speed').value
        self.approach_speed = self.get_parameter('approach_speed').value
        self.det_conf_th = self.get_parameter(
            'detection_confidence_threshold').value
        self.min_area_ratio = self.get_parameter('min_area_ratio').value
        self.stop_tol = self.get_parameter('stop_distance_tolerance').value
        self.analyze_timeout = self.get_parameter('analyze_timeout_sec').value
        self.resume_delay = self.get_parameter('resume_delay_sec').value
        self.bbox_center_tol = self.get_parameter('bbox_center_tolerance').value
        self.wheel_base = self.get_parameter('wheel_base').value

        # -- Waypoints --
        wp_file = self.get_parameter('waypoints_file').value
        self.waypoints = []
        if wp_file:
            try:
                with open(wp_file, 'r') as f:
                    data = yaml.safe_load(f)
                    self.waypoints = data.get('waypoints', [])
                self.get_logger().info(
                    f'Loaded {len(self.waypoints)} waypoints from {wp_file}')
            except Exception as e:
                self.get_logger().error(f'Failed to load waypoints: {e}')

        self.current_wp_idx = 0
        self.saved_wp_idx = 0

        # -- Nav2 --
        self.navigator = BasicNavigator()
        self.get_logger().info('Waiting for Nav2 to become active...')
        self.navigator.waitUntilNav2Active()
        self.get_logger().info('Nav2 is active')

        # -- Subscribers --
        self.sub_plant = self.create_subscription(
            PlantDetection, '/vision/plant_detected',
            self.on_plant_detected, 10)
        self.sub_fusion = self.create_subscription(
            FusionResult, '/fusion/diagnosis', self.on_fusion, 10)
        self.sub_resume = self.create_subscription(
            Bool, '/resume_navigation', self.on_resume, 10)

        # -- Publishers --
        self.pub_cmd = self.create_publisher(Twist, '/cmd_vel', 10)
        self.pub_status = self.create_publisher(
            MissionStatus, '/mission/status', 10)

        # -- Service --
        self.srv = self.create_service(
            SetBool, '/set_auto_mode', self.set_auto_mode_cb)

        # -- State --
        self.state = STATE_PATROL
        self.state_enter_time = 0.0
        self.plants_detected = 0
        self.plants_analyzed = 0
        self.last_plant = None
        self.last_fusion = None
        self.sending_goal = False

        # -- Timer --
        self.timer = self.create_timer(0.1, self.tick)

        # Start first waypoint
        self._send_next_waypoint()
        self.get_logger().info('Mission control node ready')

    # ---- Waypoint helpers ----

    def _yaw_to_quaternion(self, yaw: float) -> Quaternion:
        q = Quaternion()
        q.z = math.sin(yaw / 2.0)
        q.w = math.cos(yaw / 2.0)
        return q

    def _send_next_waypoint(self):
        """Send current waypoint to Nav2."""
        if self.state == STATE_MANUAL:
            return
        if self.current_wp_idx >= len(self.waypoints):
            self.get_logger().info('All waypoints reached')
            self.sending_goal = False
            return

        wp = self.waypoints[self.current_wp_idx]
        yaw = wp.get('yaw', 0.0)

        goal = PoseStamped()
        goal.header.frame_id = 'odom'
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = wp['x']
        goal.pose.position.y = wp['y']
        goal.pose.orientation = self._yaw_to_quaternion(yaw)

        self.navigator.goToPose(goal)
        self.sending_goal = True
        self.get_logger().info(
            f'Sent waypoint {self.current_wp_idx}: '
            f'x={wp["x"]}, y={wp["y"]}, yaw={yaw:.3f}')

    # ---- Callbacks ----

    def on_plant_detected(self, msg: PlantDetection):
        self.last_plant = msg
        if msg.detected and msg.confidence >= self.det_conf_th:
            if msg.area_ratio >= self.min_area_ratio:
                self.plants_detected += 1

    def on_fusion(self, msg: FusionResult):
        self.last_fusion = msg

    def on_resume(self, msg: Bool):
        if msg.data and self.state == STATE_MANUAL:
            self._transition(STATE_PATROL)
            self.current_wp_idx = self.saved_wp_idx
            self.navigator.cancelTask()
            self._send_next_waypoint()

    def set_auto_mode_cb(self, request, response):
        if request.data:
            # Switch to AUTO
            if self.state == STATE_MANUAL:
                self._transition(STATE_PATROL)
                self.current_wp_idx = self.saved_wp_idx
                self.navigator.cancelTask()
                self._send_next_waypoint()
            response.success = True
            response.message = 'Switched to AUTO mode'
        else:
            # Switch to MANUAL
            if self.state != STATE_MANUAL:
                self.saved_wp_idx = self.current_wp_idx
                self.navigator.cancelTask()
                self._transition(STATE_MANUAL)
            response.success = True
            response.message = 'Switched to MANUAL mode'
        return response

    # ---- State machine ----

    def tick(self):
        now = self.get_clock().now().nanoseconds / 1e9
        if self.state_enter_time == 0.0:
            self.state_enter_time = now

        cmd = Twist()
        status = MissionStatus()
        status.header.stamp = self.get_clock().now().to_msg()
        status.plants_detected = self.plants_detected
        status.plants_analyzed = self.plants_analyzed

        if self.state == STATE_PATROL:
            status.state = STATE_PATROL
            status.current_action = 'patrolling waypoints'

            # Check Nav2 goal completion
            if self.sending_goal and self.navigator.isTaskComplete():
                self.current_wp_idx += 1
                self.sending_goal = False
                self.get_logger().info(
                    f'Reached waypoint {self.current_wp_idx - 1}')
                if self.current_wp_idx < len(self.waypoints):
                    self._send_next_waypoint()
                else:
                    self.get_logger().info('All waypoints completed')

            # Check for plant detection trigger
            if (self.last_plant is not None
                    and self.last_plant.detected
                    and self.last_plant.confidence >= self.det_conf_th
                    and self.last_plant.area_ratio >= self.min_area_ratio):
                self.saved_wp_idx = self.current_wp_idx
                self.navigator.cancelTask()
                self.sending_goal = False
                self._transition(STATE_APPROACHING, now)

        elif self.state == STATE_APPROACHING:
            status.state = STATE_APPROACHING
            status.current_action = 'approaching plant'

            if self.last_plant is None or not self.last_plant.detected:
                # Lost plant, resume patrol
                self._transition(STATE_RESUME, now)
            else:
                bbox = self.last_plant.bbox  # [xmin, ymin, xmax, ymax]
                cx = (bbox[0] + bbox[2]) / 2.0
                cy = (bbox[1] + bbox[3]) / 2.0
                area = self.last_plant.area_ratio

                # Center the plant
                if abs(cx - 0.5) > self.bbox_center_tol:
                    cmd.angular.z = -1.0 * (cx - 0.5) * 2.0
                if abs(cy - 0.5) > self.bbox_center_tol:
                    cmd.linear.x = self.approach_speed * (0.5 - cy)
                else:
                    cmd.linear.x = self.approach_speed

                # Stop when close enough
                if area > (self.min_area_ratio + 0.15):
                    self._transition(STATE_STOPPED, now)

        elif self.state == STATE_STOPPED:
            status.state = STATE_STOPPED
            status.current_action = 'stopped for analysis'
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            self._transition(STATE_ANALYZING, now)

        elif self.state == STATE_ANALYZING:
            status.state = STATE_ANALYZING
            status.current_action = 'analyzing plant health'
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0

            elapsed = now - self.state_enter_time
            if self.last_fusion is not None:
                self.plants_analyzed += 1
                self._transition(STATE_ACTION, now)
            elif elapsed > self.analyze_timeout:
                self.get_logger().warn(
                    'Analysis timeout, proceeding without fusion data')
                self.plants_analyzed += 1
                self._transition(STATE_ACTION, now)

        elif self.state == STATE_ACTION:
            status.state = STATE_ACTION
            status.current_action = 'recording result'
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0

            if self.last_fusion is not None:
                self.get_logger().info(
                    f'Plant #{self.plants_analyzed}: '
                    f'risk={self.last_fusion.risk_score:.2f} '
                    f'alert={self.last_fusion.alert_level} '
                    f'mode={self.last_fusion.mode}')

            self._transition(STATE_RESUME, now)

        elif self.state == STATE_RESUME:
            status.state = STATE_RESUME
            status.current_action = 'resuming patrol'
            elapsed = now - self.state_enter_time
            if elapsed < self.resume_delay:
                cmd.linear.x = 0.0  # brief pause
            else:
                cmd.linear.x = self.cruise_speed
                self._transition(STATE_PATROL, now)
                self.current_wp_idx = self.saved_wp_idx
                self._send_next_waypoint()

        elif self.state == STATE_MANUAL:
            status.state = STATE_MANUAL
            status.current_action = 'manual control'
            # Do not publish cmd_vel in MANUAL mode;
            # web remote handles /cmd_vel directly.

        status.progress = self._compute_progress()
        self.pub_status.publish(status)
        # Only publish cmd_vel when not in MANUAL mode
        if self.state != STATE_MANUAL:
            self.pub_cmd.publish(cmd)

    def _transition(self, new_state: str, now: float = None):
        if now is None:
            now = self.get_clock().now().nanoseconds / 1e9
        if self.state != new_state:
            self.get_logger().info(f'State: {self.state} -> {new_state}')
        self.state = new_state
        self.state_enter_time = now

    def _compute_progress(self) -> float:
        if self.plants_detected == 0:
            return 0.0
        return min(1.0, self.plants_analyzed / max(1, self.plants_detected))


def main(args=None):
    rclpy.init(args=args)
    node = MissionControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
