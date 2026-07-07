#!/usr/bin/env python3
"""Mission control node with Nav2 waypoint navigation + vision pipeline.

States:
  PATROL      - Nav2 waypoint cruising, YOLO real-time detection
  STOPPED     - Stop and trigger vision pipeline scan
  SCANNING    - Wait for vision pipeline to complete (gimbal + two-stage inference)
  ANALYZING   - Wait for fusion diagnosis result
  ACTION      - Record the diagnosis result
  RESUME      - Brief pause before resuming patrol
  MANUAL      - Web remote control mode, Nav2 paused
"""

import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist, Quaternion
from nav_msgs.msg import Odometry
from nav2_simple_commander.robot_navigator import BasicNavigator
from sentry_interfaces.msg import PlantDetection, FusionResult, MissionStatus, Diagnosis
from sentry_interfaces.srv import PipelineTrigger, SetCropType
from std_msgs.msg import Bool
from std_srvs.srv import SetBool
import yaml


STATE_PATROL = 'PATROL'
STATE_STOPPED = 'STOPPED'
STATE_SCANNING = 'SCANNING'
STATE_ANALYZING = 'ANALYZING'
STATE_ACTION = 'ACTION'
STATE_RESUME = 'RESUME'
STATE_MANUAL = 'MANUAL'

_CMDV_OWNER_STATES = {
    STATE_STOPPED, STATE_SCANNING, STATE_ANALYZING, STATE_ACTION, STATE_RESUME
}


class MissionControlNode(Node):
    def __init__(self):
        super().__init__('mission_control_node')

        self.declare_parameter('cruise_speed', 0.3)
        self.declare_parameter('detection_confidence_threshold', 0.5)
        self.declare_parameter('min_area_ratio', 0.05)
        self.declare_parameter('analyze_timeout_sec', 5.0)
        self.declare_parameter('resume_delay_sec', 2.0)
        self.declare_parameter('waypoints_file', '')
        self.declare_parameter('wheel_base', 0.23)
        self.declare_parameter('pulses_per_meter', 11035)
        self.declare_parameter('min_resume_distance', 0.5)
        self.declare_parameter('crop_type', 'tomato')
        self.declare_parameter('max_scan_shots', 3)

        self.cruise_speed = self.get_parameter('cruise_speed').value
        self.det_conf_th = self.get_parameter(
            'detection_confidence_threshold').value
        self.min_area_ratio = self.get_parameter('min_area_ratio').value
        self.analyze_timeout = self.get_parameter('analyze_timeout_sec').value
        self.resume_delay = self.get_parameter('resume_delay_sec').value
        self.min_resume_distance = self.get_parameter('min_resume_distance').value
        self.crop_type = self.get_parameter('crop_type').value
        self.max_scan_shots = self.get_parameter('max_scan_shots').value

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
        self.waypoint_labels = [
            f'WP{i}: ({wp["x"]:.1f}, {wp["y"]:.1f})'
            for i, wp in enumerate(self.waypoints)
        ]

        # -- Nav2 --
        self.navigator = BasicNavigator()
        self._nav2_ready = False
        # Don't block - Nav2 readiness checked in tick()

        # -- Subscribers --
        self.sub_plant = self.create_subscription(
            PlantDetection, '/vision/plant_detected',
            self.on_plant_detected, 10)
        self.sub_fusion = self.create_subscription(
            FusionResult, '/fusion/diagnosis', self.on_fusion, 10)
        self.sub_resume = self.create_subscription(
            Bool, '/resume_navigation', self.on_resume, 10)
        self.sub_odom = self.create_subscription(
            Odometry, '/odom', self.on_odom, 10)

        # -- Publishers --
        self.pub_cmd = self.create_publisher(Twist, '/cmd_vel', 10)
        self.pub_status = self.create_publisher(
            MissionStatus, '/mission/status', 10)
        self.pub_diag = self.create_publisher(Diagnosis, '/vision/diagnosis', 10)

        # -- Service --
        self.srv = self.create_service(
            SetBool, '/set_auto_mode', self.set_auto_mode_cb)

        self.crop_type_srv = self.create_service(
            SetCropType, '/set_crop_type', self.set_crop_type_cb)

        # -- Pipeline client --
        self.pipeline_client = self.create_client(
            PipelineTrigger, '/vision/pipeline/trigger')

        # -- Plant detector pause client --
        self.pause_detector_client = self.create_client(
            SetBool, '/vision/plant_detector/pause')

        # -- State -- start in MANUAL so car stays still until frontend triggers AUTO
        self.state = STATE_MANUAL
        self.state_enter_time = 0.0
        self.plants_detected = 0
        self.plants_analyzed = 0
        self.last_plant = None
        self.last_fusion = None
        self.sending_goal = False

        # -- De-duplication --
        self.reference_x = 0.0
        self.reference_y = 0.0
        self.odom_x = 0.0
        self.odom_y = 0.0

        # -- Async action tracking --
        self._pending_future = None
        self._pending_action = ''  # 'pause', 'pipeline', 'resume'

        # -- Timer --
        self.timer = self.create_timer(0.1, self.tick)

        self._send_next_waypoint()
        self.get_logger().info('Mission control node ready')

    # ---- Waypoint helpers ----

    def _yaw_to_quaternion(self, yaw: float) -> Quaternion:
        q = Quaternion()
        q.z = math.sin(yaw / 2.0)
        q.w = math.cos(yaw / 2.0)
        return q

    def _send_next_waypoint(self):
        if not self._nav2_ready:
            return
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

    def on_odom(self, msg: Odometry):
        self.odom_x = msg.pose.pose.position.x
        self.odom_y = msg.pose.pose.position.y

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
            if self.state == STATE_MANUAL:
                self._transition(STATE_PATROL)
                self.current_wp_idx = self.saved_wp_idx
                self.navigator.cancelTask()
                self._send_next_waypoint()
            response.success = True
            response.message = 'Switched to AUTO mode'
        else:
            if self.state != STATE_MANUAL:
                self.saved_wp_idx = self.current_wp_idx
                self.navigator.cancelTask()
                self._transition(STATE_MANUAL)
            response.success = True
            response.message = 'Switched to MANUAL mode'
        return response

    def set_crop_type_cb(self, request, response):
        valid = {'tomato', 'wheat', 'strawberry'}
        if request.crop_type not in valid:
            response.success = False
            response.message = f'Invalid crop type: {request.crop_type}. Valid: {valid}'
            return response

        self.crop_type = request.crop_type
        self.get_logger().info(f'Crop type switched to {request.crop_type}')

        response.success = True
        response.message = f'Crop type set to {request.crop_type}'
        return response

    # ---- De-duplication ----

    def _distance_from_reference(self) -> float:
        dx = self.odom_x - self.reference_x
        dy = self.odom_y - self.reference_y
        return math.sqrt(dx * dx + dy * dy)

    def _should_trigger_scan(self) -> bool:
        """Check if we're far enough from the last scan position."""
        if self._distance_from_reference() >= self.min_resume_distance:
            return True
        self.get_logger().debug(
            f'Suppressing trigger: distance={self._distance_from_reference():.2f} '
            f'< {self.min_resume_distance}')
        return False

    # ---- Pipeline helpers ----

    def _pause_detector_async(self):
        req = SetBool.Request()
        req.data = True
        return self.pause_detector_client.call_async(req)

    def _resume_detector_async(self):
        req = SetBool.Request()
        req.data = False
        return self.pause_detector_client.call_async(req)

    def _call_pipeline_async(self):
        req = PipelineTrigger.Request()
        req.crop_type = self.crop_type
        req.max_shots = self.max_scan_shots
        return self.pipeline_client.call_async(req)

    # ---- State machine ----

    def tick(self):
        now = self.get_clock().now().nanoseconds / 1e9

        # Background Nav2 readiness check (non-blocking)
        if not self._nav2_ready and self.navigator.nav2_bringup_ready():
            self._nav2_ready = True
            self.get_logger().info('Nav2 is active')

        if self.state_enter_time == 0.0:
            self.state_enter_time = now

        cmd = Twist()
        status = MissionStatus()
        status.header.stamp = self.get_clock().now().to_msg()
        status.plants_detected = self.plants_detected
        status.plants_analyzed = self.plants_analyzed
        status.current_wp_idx = self.current_wp_idx
        status.total_wps = len(self.waypoints)
        status.waypoint_labels = self.waypoint_labels

        if self.state == STATE_PATROL:
            status.state = STATE_PATROL
            status.current_action = 'patrolling waypoints'

            if self.sending_goal and self.navigator.isTaskComplete():
                self.current_wp_idx += 1
                self.sending_goal = False
                self.get_logger().info(
                    f'Reached waypoint {self.current_wp_idx - 1}')
                if self.current_wp_idx < len(self.waypoints):
                    self._send_next_waypoint()
                else:
                    self.get_logger().info('All waypoints completed')

            # Check for plant detection trigger (with de-duplication)
            if (self.last_plant is not None
                    and self.last_plant.detected
                    and self.last_plant.confidence >= self.det_conf_th
                    and self.last_plant.area_ratio >= self.min_area_ratio
                    and self._should_trigger_scan()):
                self.saved_wp_idx = self.current_wp_idx
                self.navigator.cancelTask()
                self.sending_goal = False
                self._transition(STATE_STOPPED, now)

        elif self.state == STATE_STOPPED:
            status.state = STATE_STOPPED
            status.current_action = 'stopping for scan'
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0

            if self._pending_action == '':
                if self.pause_detector_client.wait_for_service(timeout_sec=1.0):
                    self._pending_future = self._pause_detector_async()
                    self._pending_action = 'pause'
                else:
                    self.get_logger().error('Detector pause unavailable, resuming')
                    self._transition(STATE_RESUME, now)

            elif self._pending_action == 'pause':
                if self._pending_future.done():
                    if self._pending_future.result() is not None and self._pending_future.result().success:
                        if self.pipeline_client.wait_for_service(timeout_sec=1.0):
                            self._pending_future = self._call_pipeline_async()
                            self._pending_action = 'pipeline'
                        else:
                            self.get_logger().error('Pipeline service unavailable')
                            self._resume_detector_async()
                            self._pending_action = ''
                            self._transition(STATE_RESUME, now)
                    else:
                        self.get_logger().error('Failed to pause detector')
                        self._transition(STATE_RESUME, now)

            elif self._pending_action == 'pipeline':
                if self._pending_future.done():
                    result = self._pending_future.result()
                    self._pending_action = ''
                    if result is not None and result.success:
                        self.pub_diag.publish(result.result)
                        self.get_logger().info(
                            f'Pipeline result: {result.result.disease_class} '
                            f'conf={result.result.confidence:.3f}')
                        self.reference_x = self.odom_x
                        self.reference_y = self.odom_y
                    else:
                        self.get_logger().warn('Pipeline failed or timed out')

                    self._resume_detector_async()
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
                cmd.linear.x = 0.0
            else:
                cmd.linear.x = self.cruise_speed
                self._transition(STATE_PATROL, now)
                self.current_wp_idx = self.saved_wp_idx
                self._send_next_waypoint()

        elif self.state == STATE_MANUAL:
            status.state = STATE_MANUAL
            status.current_action = 'manual control'

        status.progress = self._compute_progress()
        self.pub_status.publish(status)
        if self.state in _CMDV_OWNER_STATES:
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
