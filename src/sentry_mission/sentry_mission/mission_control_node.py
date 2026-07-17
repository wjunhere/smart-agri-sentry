#!/usr/bin/env python3
"""Mission control node with Nav2 waypoint navigation + vision pipeline.

States:
  PATROL      - Nav2 waypoint cruising, YOLO real-time detection
  STOPPED     - Stop and trigger vision pipeline scan
  SCANNING    - Wait for vision pipeline to complete (fixed-camera inference)
  ANALYZING   - Wait for fusion diagnosis result
  ACTION      - Record the diagnosis result
  RESUME      - Brief pause before resuming patrol
  MANUAL      - Web remote control mode, Nav2 paused
"""

import math
import threading

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist, Quaternion
from nav_msgs.msg import Odometry
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from robot_localization.srv import SetPose
from sensor_msgs.msg import Imu, LaserScan
from sentry_interfaces.msg import (
    PlantDetection, FusionResult, MissionStatus, Diagnosis, ObstacleInfo)
from sentry_interfaces.srv import PipelineTrigger, SetCropType
from sentry_mission.autonomous_cruise import should_send_patrol_goal
from std_msgs.msg import Bool
from std_srvs.srv import SetBool, Trigger
import yaml


STATE_PATROL = 'PATROL'
STATE_STOPPED = 'STOPPED'
STATE_SCANNING = 'SCANNING'
STATE_ANALYZING = 'ANALYZING'
STATE_ACTION = 'ACTION'
STATE_RESUME = 'RESUME'
STATE_MANUAL = 'MANUAL'
STATE_OBSTACLE_STOP = 'OBSTACLE_STOP'
STATE_OBSTACLE_BACKUP = 'OBSTACLE_BACKUP'
STATE_OBSTACLE_TURN = 'OBSTACLE_TURN'
STATE_OBSTACLE_ARC_DRIVE = 'OBSTACLE_ARC_DRIVE'
STATE_OBSTACLE_TURN_BACK = 'OBSTACLE_TURN_BACK'
STATE_OBSTACLE_REJOIN_FORWARD = 'OBSTACLE_REJOIN_FORWARD'
STATE_AVOIDING = 'AVOIDING'

DEFAULT_FIXED_POINT_RADIUS = 0.20
FIXED_POINT_DIAGNOSIS_CLASS_ID = 254
TOMATO_DISEASE_CLASSES = (
    'late_blight',
    'healthy',
    'early_blight',
    'bacterial_spot',
    'leaf_mold',
    'septoria_leaf_spot',
    'tomato_yellow_leaf_curl_virus',
)

_CMDV_OWNER_STATES = {
    STATE_STOPPED, STATE_SCANNING, STATE_ANALYZING, STATE_ACTION, STATE_RESUME,
    STATE_OBSTACLE_STOP, STATE_OBSTACLE_BACKUP, STATE_OBSTACLE_TURN,
    STATE_OBSTACLE_ARC_DRIVE, STATE_OBSTACLE_TURN_BACK,
    STATE_OBSTACLE_REJOIN_FORWARD
}


class MissionControlNode(Node):
    def __init__(self):
        super().__init__('mission_control_node')
        self.service_callback_group = ReentrantCallbackGroup()
        self.sensor_callback_group = ReentrantCallbackGroup()
        self.velocity_callback_group = ReentrantCallbackGroup()

        self.declare_parameter('cruise_speed', 0.3)
        self.declare_parameter('detection_confidence_threshold', 0.5)
        self.declare_parameter('min_area_ratio', 0.05)
        self.declare_parameter('analyze_timeout_sec', 5.0)
        self.declare_parameter('resume_delay_sec', 2.0)
        self.declare_parameter('waypoints_file', '')
        self.declare_parameter('wheel_base', 0.23)
        self.declare_parameter('pulses_per_meter', 11552)
        self.declare_parameter('min_resume_distance', 0.5)
        self.declare_parameter('crop_type', 'tomato')
        self.declare_parameter('max_scan_shots', 3)
        self.declare_parameter('mission_params_file', '')
        self.declare_parameter('odom_topic', '/odometry/filtered')
        self.declare_parameter('imu_topic', '/sensor/imu/data')
        self.declare_parameter('nav_cmd_topic', '/cmd_vel_nav_smoothed')
        self.declare_parameter('enable_obstacle_avoidance', True)
        self.declare_parameter('obstacle_stop_distance', 0.50)
        self.declare_parameter('obstacle_goal_margin', 0.15)
        self.declare_parameter('obstacle_resume_delay_sec', 0.5)
        self.declare_parameter('avoidance_backup_distance', 0.18)
        self.declare_parameter('avoidance_backup_speed', -0.08)
        self.declare_parameter('avoidance_backup_timeout_sec', 2.5)
        self.declare_parameter('avoidance_turn_angle', 0.60)
        self.declare_parameter('avoidance_turn_speed', 0.30)
        self.declare_parameter('avoidance_drive_distance', 0.55)
        self.declare_parameter('avoidance_rejoin_distance', 0.30)
        self.declare_parameter('avoidance_drive_speed', 0.08)
        self.declare_parameter('avoidance_drive_timeout_sec', 10.0)
        self.declare_parameter('avoidance_rejoin_timeout_sec', 5.0)
        self.declare_parameter('avoidance_internal_hard_stop', 0.20)
        self.declare_parameter('avoidance_internal_side_hard_stop', 0.05)
        self.declare_parameter('avoidance_retrigger_suppression_sec', 2.5)
        self.declare_parameter('avoidance_front_hard_stop', 0.20)
        self.declare_parameter('avoidance_side_soft_min', 0.20)
        self.declare_parameter('avoidance_side_target', 0.28)
        self.declare_parameter('avoidance_correction_angular_speed', 0.18)
        self.declare_parameter('avoidance_lateral_offset', 0.35)
        self.declare_parameter('avoidance_forward_distance', 1.0)
        self.declare_parameter('avoidance_entry_forward_distance', 0.35)
        self.declare_parameter('avoidance_side_clearance_min', 0.28)
        self.declare_parameter('lidar_base_x', 0.0804)
        self.declare_parameter('lidar_base_y', 0.0)
        self.declare_parameter('lidar_base_yaw', -1.57079)

        self.cruise_speed = self.get_parameter('cruise_speed').value
        self.det_conf_th = self.get_parameter(
            'detection_confidence_threshold').value
        self.min_area_ratio = self.get_parameter('min_area_ratio').value
        self.analyze_timeout = self.get_parameter('analyze_timeout_sec').value
        self.resume_delay = self.get_parameter('resume_delay_sec').value
        self.min_resume_distance = self.get_parameter('min_resume_distance').value
        self.crop_type = self.get_parameter('crop_type').value
        self.max_scan_shots = self.get_parameter('max_scan_shots').value
        self.mission_params_file = self.get_parameter(
            'mission_params_file').value
        self.enable_obstacle_avoidance = self.get_parameter(
            'enable_obstacle_avoidance').value
        self.odom_topic = self.get_parameter('odom_topic').value
        self.imu_topic = self.get_parameter('imu_topic').value
        self.nav_cmd_topic = self.get_parameter('nav_cmd_topic').value
        self.obstacle_stop_distance = self.get_parameter(
            'obstacle_stop_distance').value
        self.obstacle_goal_margin = self.get_parameter(
            'obstacle_goal_margin').value
        self.obstacle_resume_delay = self.get_parameter(
            'obstacle_resume_delay_sec').value
        self.avoidance_backup_distance = self.get_parameter(
            'avoidance_backup_distance').value
        self.avoidance_backup_speed = self.get_parameter(
            'avoidance_backup_speed').value
        self.avoidance_backup_timeout = self.get_parameter(
            'avoidance_backup_timeout_sec').value
        self.avoidance_turn_angle = self.get_parameter(
            'avoidance_turn_angle').value
        self.avoidance_turn_speed = self.get_parameter(
            'avoidance_turn_speed').value
        self.avoidance_drive_distance = self.get_parameter(
            'avoidance_drive_distance').value
        self.avoidance_rejoin_distance = self.get_parameter(
            'avoidance_rejoin_distance').value
        self.avoidance_drive_speed = self.get_parameter(
            'avoidance_drive_speed').value
        self.avoidance_drive_timeout = self.get_parameter(
            'avoidance_drive_timeout_sec').value
        self.avoidance_rejoin_timeout = self.get_parameter(
            'avoidance_rejoin_timeout_sec').value
        self.avoidance_internal_hard_stop = self.get_parameter(
            'avoidance_internal_hard_stop').value
        self.avoidance_internal_side_hard_stop = self.get_parameter(
            'avoidance_internal_side_hard_stop').value
        self.avoidance_retrigger_suppression = self.get_parameter(
            'avoidance_retrigger_suppression_sec').value
        self.avoidance_front_hard_stop = self.get_parameter(
            'avoidance_front_hard_stop').value
        self.avoidance_side_soft_min = self.get_parameter(
            'avoidance_side_soft_min').value
        self.avoidance_side_target = self.get_parameter(
            'avoidance_side_target').value
        self.avoidance_correction_angular_speed = self.get_parameter(
            'avoidance_correction_angular_speed').value
        self.avoidance_lateral_offset = self.get_parameter(
            'avoidance_lateral_offset').value
        self.avoidance_forward_distance = self.get_parameter(
            'avoidance_forward_distance').value
        self.avoidance_entry_forward_distance = self.get_parameter(
            'avoidance_entry_forward_distance').value
        self.avoidance_side_clearance_min = self.get_parameter(
            'avoidance_side_clearance_min').value
        self.lidar_base_x = self.get_parameter('lidar_base_x').value
        self.lidar_base_y = self.get_parameter('lidar_base_y').value
        self.lidar_base_yaw = self.get_parameter('lidar_base_yaw').value

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
        self.fixed_point_stops = self._load_fixed_point_stops(
            self.mission_params_file)

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
            Odometry, self.odom_topic, self.on_odom, 10,
            callback_group=self.sensor_callback_group)
        self.sub_imu = self.create_subscription(
            Imu, self.imu_topic, self.on_imu, 10,
            callback_group=self.sensor_callback_group)
        self.sub_obstacle = self.create_subscription(
            ObstacleInfo, '/lidar/obstacle_info', self.on_obstacle_info, 10,
            callback_group=self.sensor_callback_group)
        self.sub_scan = self.create_subscription(
            LaserScan, '/scan', self.on_scan, 10,
            callback_group=self.sensor_callback_group)
        self.sub_nav_cmd = self.create_subscription(
            Twist, self.nav_cmd_topic, self.on_nav_cmd, 10,
            callback_group=self.velocity_callback_group)

        # -- Publishers --
        self.pub_cmd = self.create_publisher(Twist, '/cmd_vel', 10)
        self.pub_status = self.create_publisher(
            MissionStatus, '/mission/status', 10)
        self.pub_diag = self.create_publisher(Diagnosis, '/vision/diagnosis', 10)

        # -- Service --
        self.srv = self.create_service(
            SetBool, '/set_auto_mode', self.set_auto_mode_cb,
            callback_group=self.service_callback_group)

        self.crop_type_srv = self.create_service(
            SetCropType, '/set_crop_type', self.set_crop_type_cb)

        # -- Pipeline client --
        self.pipeline_client = self.create_client(
            PipelineTrigger, '/vision/pipeline/trigger')

        # -- Plant detector pause client --
        self.pause_detector_client = self.create_client(
            SetBool, '/vision/plant_detector/pause')
        self.reset_encoder_client = self.create_client(
            Trigger, '/sentry/reset_encoder')
        self.reset_wheel_odom_client = self.create_client(
            Trigger, '/sentry/reset_wheel_odom')
        self.set_pose_client = self.create_client(SetPose, '/set_pose')

        # -- State -- start in MANUAL so car stays still until frontend triggers AUTO
        self.state = STATE_MANUAL
        self.state_enter_time = 0.0
        self.plants_detected = 0
        self.plants_analyzed = 0
        self.last_plant = None
        self.last_fusion = None
        self.active_fixed_point_disease = None
        self.handled_fixed_point_stops = set()
        self._diagnosis_published_at_ns = 0
        self.sending_goal = False
        self.last_goal_sent_time = 0.0
        self._cancel_in_progress = False
        self._nav_goal_lock = threading.Lock()
        self._state_lock = threading.Lock()
        self._next_goal_time = 0.0
        self.last_obstacle = None
        self.last_scan = None
        self.last_nav_cmd = None
        self.last_nav_cmd_time = 0.0
        self.odom_yaw = 0.0
        self.imu_yaw = 0.0
        self.last_imu_time = None
        self.avoidance_goals = []
        self.avoidance_goal_idx = 0
        self.avoidance_return_wp_idx = 0
        self.avoidance_backup_start_x = 0.0
        self.avoidance_backup_start_y = 0.0
        self.avoidance_side = 1
        self.avoidance_turn_start_yaw = 0.0
        self.avoidance_turn_yaw_source = 'odom'
        self.avoidance_drive_start_x = 0.0
        self.avoidance_drive_start_y = 0.0
        self.avoidance_suppress_until = 0.0

        # -- De-duplication --
        self.reference_x = 0.0
        self.reference_y = 0.0
        self.has_scan_reference = False
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
            self.last_goal_sent_time = 0.0
            return

        wp = self.waypoints[self.current_wp_idx]
        yaw = wp.get('yaw', 0.0)

        goal = PoseStamped()
        goal.header.frame_id = 'odom'
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = wp['x']
        goal.pose.position.y = wp['y']
        goal.pose.orientation = self._yaw_to_quaternion(yaw)

        self.sending_goal = True
        self.last_goal_sent_time = self.get_clock().now().nanoseconds / 1e9
        self._go_to_pose_async(goal, f'waypoint {self.current_wp_idx}')
        self.get_logger().info(
            f'Sent waypoint {self.current_wp_idx}: '
            f'x={wp["x"]}, y={wp["y"]}, yaw={yaw:.3f}')

    def _send_pose_goal(self, x: float, y: float, yaw: float, label: str):
        goal = PoseStamped()
        goal.header.frame_id = 'odom'
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = x
        goal.pose.position.y = y
        goal.pose.orientation = self._yaw_to_quaternion(yaw)

        self.sending_goal = True
        self.last_goal_sent_time = self.get_clock().now().nanoseconds / 1e9
        self._go_to_pose_async(goal, label)
        self.get_logger().info(
            f'Sent {label}: x={x:.3f}, y={y:.3f}, yaw={yaw:.3f}')

    def _go_to_pose_async(self, goal: PoseStamped, label: str):
        def _send():
            with self._nav_goal_lock:
                try:
                    self.navigator.goToPose(goal)
                except Exception as exc:
                    self.get_logger().warn(f'Failed to send {label}: {exc}')
                    self.sending_goal = False
                    self.last_goal_sent_time = 0.0

        threading.Thread(target=_send, daemon=True).start()

    def _load_fixed_point_stops(self, params_file: str) -> list:
        if not params_file:
            return []
        try:
            with open(params_file, 'r') as f:
                data = yaml.safe_load(f) or {}
        except (OSError, yaml.YAMLError) as exc:
            self.get_logger().error(
                f'Failed to load fixed-point stops from {params_file}: {exc}')
            return []

        stops = data.get('fixed_point_stops', [])
        if not isinstance(stops, list):
            self.get_logger().error('fixed_point_stops must be a list')
            return []

        clean = []
        for index, stop in enumerate(stops):
            try:
                x = float(stop['x'])
                y = float(stop['y'])
                radius = float(stop.get('radius', DEFAULT_FIXED_POINT_RADIUS))
                disease_class = str(stop['disease_class'])
            except (KeyError, TypeError, ValueError) as exc:
                self.get_logger().error(
                    f'Ignoring fixed-point stop {index}: {exc}')
                continue
            if not all(math.isfinite(value) for value in (x, y, radius)) or radius <= 0.0:
                self.get_logger().error(
                    f'Ignoring fixed-point stop {index}: invalid coordinates or radius')
                continue
            clean.append({
                'x': x,
                'y': y,
                'radius': radius,
                'disease_class': disease_class,
            })
        self.get_logger().info(f'Loaded {len(clean)} fixed-point stops')
        return clean



    # ---- Callbacks ----

    def on_odom(self, msg: Odometry):
        self.odom_x = msg.pose.pose.position.x
        self.odom_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self.odom_yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z))

    def on_imu(self, msg: Imu):
        q = msg.orientation
        self.imu_yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.last_imu_time = self.get_clock().now().nanoseconds / 1e9

    def on_obstacle_info(self, msg: ObstacleInfo):
        self.last_obstacle = msg
        if self.state == STATE_PATROL and self._front_obstacle_too_close():
            now = self.get_clock().now().nanoseconds / 1e9
            self._trigger_obstacle_avoidance(now)

    def on_scan(self, msg: LaserScan):
        self.last_scan = msg

    def on_nav_cmd(self, msg: Twist):
        self.last_nav_cmd = msg
        self.last_nav_cmd_time = self.get_clock().now().nanoseconds / 1e9

    def on_plant_detected(self, msg: PlantDetection):
        if self.state != STATE_PATROL:
            return
        self.last_plant = msg

    def on_fusion(self, msg: FusionResult):
        stamp = msg.header.stamp
        fusion_stamp_ns = stamp.sec * 1_000_000_000 + stamp.nanosec
        if fusion_stamp_ns < self._diagnosis_published_at_ns:
            self.get_logger().debug('Ignoring fusion result older than current scan')
            return
        self.last_fusion = msg

    def on_resume(self, msg: Bool):
        if msg.data and self.state == STATE_MANUAL:
            self._prepare_autonomous_start()
            self._transition(STATE_PATROL)
            self.current_wp_idx = self._saved_patrol_index()

    def set_auto_mode_cb(self, request, response):
        if request.data:
            if self.state == STATE_MANUAL:
                self._prepare_autonomous_start()
                self._transition(STATE_PATROL)
                self.current_wp_idx = self._saved_patrol_index()
            response.success = True
            response.message = 'Switched to AUTO mode'
        else:
            if self.state != STATE_MANUAL:
                self.saved_wp_idx = self.current_wp_idx
                self.sending_goal = False
                self.last_goal_sent_time = 0.0
                self._transition(STATE_MANUAL)
                self._publish_stop()
                self._cancel_nav2_task_async()
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
        if not self.has_scan_reference:
            return True
        if self._distance_from_reference() >= self.min_resume_distance:
            return True
        self.get_logger().debug(
            f'Suppressing trigger: distance={self._distance_from_reference():.2f} '
            f'< {self.min_resume_distance}')
        return False

    def _find_unhandled_fixed_point_stop(self):
        if self.state != STATE_PATROL:
            return None
        for index, stop in enumerate(self.fixed_point_stops):
            if index in self.handled_fixed_point_stops:
                continue
            distance = math.hypot(
                self.odom_x - stop['x'], self.odom_y - stop['y'])
            if distance <= stop['radius']:
                return index, stop
        return None

    def _accept_fixed_point_stop(self, index: int, stop: dict, now: float):
        self.handled_fixed_point_stops.add(index)
        self.active_fixed_point_disease = stop['disease_class']
        self.saved_wp_idx = self.current_wp_idx
        self.last_fusion = None
        self._diagnosis_published_at_ns = 0
        self.plants_detected += 1
        self.get_logger().info(
            'Fixed-point stop trigger accepted: '
            f'index={index}, x={stop["x"]:.3f}, y={stop["y"]:.3f}, '
            f'radius={stop["radius"]:.3f}, disease={stop["disease_class"]}')
        self._cancel_nav2_task_async()
        self.sending_goal = False
        self.last_goal_sent_time = 0.0
        self._transition(STATE_STOPPED, now)

    # ---- Obstacle avoidance helpers ----

    def _front_obstacle_too_close(self) -> bool:
        if not self.enable_obstacle_avoidance or self.last_obstacle is None:
            return False
        now = self.get_clock().now().nanoseconds / 1e9
        if now < self.avoidance_suppress_until:
            return False
        dist = self.last_obstacle.front_min_distance
        if not (
            math.isfinite(dist)
            and dist > 0.0
            and dist <= self.obstacle_stop_distance
        ):
            return False
        goal_dist = self._distance_to_current_waypoint()
        return math.isfinite(goal_dist) and dist < goal_dist - self.obstacle_goal_margin

    def _distance_to_current_waypoint(self) -> float:
        if self.current_wp_idx >= len(self.waypoints):
            return float('inf')
        wp = self.waypoints[self.current_wp_idx]
        return math.hypot(wp['x'] - self.odom_x, wp['y'] - self.odom_y)

    def _scan_points_in_base(self):
        if self.last_scan is None:
            return []
        points = []
        c = math.cos(self.lidar_base_yaw)
        s = math.sin(self.lidar_base_yaw)
        for i, distance in enumerate(self.last_scan.ranges):
            if (not math.isfinite(distance)
                    or distance < self.last_scan.range_min
                    or distance > self.last_scan.range_max):
                continue
            angle = self.last_scan.angle_min + i * self.last_scan.angle_increment
            lx = distance * math.cos(angle)
            ly = distance * math.sin(angle)
            bx = self.lidar_base_x + c * lx - s * ly
            by = self.lidar_base_y + s * lx + c * ly
            points.append((bx, by))
        return points

    def _side_clearance(self, side: int) -> float:
        min_clearance = float('inf')
        for bx, by in self._scan_points_in_base():
            if bx < -0.1 or bx > self.avoidance_forward_distance:
                continue
            if side > 0 and by <= 0.1:
                continue
            if side < 0 and by >= -0.1:
                continue
            lateral_distance = abs(by)
            if lateral_distance < min_clearance:
                min_clearance = lateral_distance
        return min_clearance

    def _choose_avoidance_side(self) -> int:
        left_clearance = self._side_clearance(1)
        right_clearance = self._side_clearance(-1)
        side = 1 if left_clearance >= right_clearance else -1
        chosen = left_clearance if side > 0 else right_clearance
        if chosen < self.avoidance_side_clearance_min:
            self.get_logger().warn(
                'Avoidance side clearance is tight: '
                f'left={left_clearance:.2f}m right={right_clearance:.2f}m')
        return side

    def _robot_relative_to_odom(self, dx: float, dy: float):
        c = math.cos(self.odom_yaw)
        s = math.sin(self.odom_yaw)
        return (
            self.odom_x + c * dx - s * dy,
            self.odom_y + s * dx + c * dy,
        )

    def _trigger_obstacle_avoidance(self, now: float) -> bool:
        with self._state_lock:
            if self.state != STATE_PATROL:
                return False
            # Do not carry a plant frame into the bypass maneuver. A new
            # patrol frame is required after the vehicle has rejoined.
            self.last_plant = None
            self.saved_wp_idx = self.current_wp_idx
            self._cancel_nav2_task_async()
            self.sending_goal = False
            self.last_goal_sent_time = 0.0
            self._publish_stop()
            self._prepare_avoidance_maneuver()
            self._transition(STATE_OBSTACLE_STOP, now)
            return True



    def _angle_diff(self, target: float, source: float) -> float:
        return math.atan2(math.sin(target - source), math.cos(target - source))

    def _prepare_avoidance_maneuver(self):
        self.avoidance_side = self._choose_avoidance_side()
        self.avoidance_return_wp_idx = self.current_wp_idx
        self.avoidance_goal_idx = 0
        self.avoidance_goals = []
        self.get_logger().warn(
            'Front obstacle detected; stopping and preparing '
            f'{"left" if self.avoidance_side > 0 else "right"} '
            f'maneuver back to waypoint {self.avoidance_return_wp_idx}')

    def _front_distance(self) -> float:
        if self.last_obstacle is None:
            return float('inf')
        dist = self.last_obstacle.front_min_distance
        return dist if math.isfinite(dist) and dist > 0.0 else float('inf')

    def _avoidance_internal_obstacle_too_close(self) -> bool:
        if self._front_distance() <= self.avoidance_internal_hard_stop:
            return True
        return (
            min(self._side_clearance(1), self._side_clearance(-1))
            <= self.avoidance_internal_side_hard_stop
        )

    def _select_avoidance_turn_yaw(self, now: float):
        if (self.last_imu_time is not None
                and now - self.last_imu_time <= 0.5):
            return self.imu_yaw, 'imu'
        return self.odom_yaw, 'odom'

    def _begin_avoidance_turn(self, state: str, now: float):
        (self.avoidance_turn_start_yaw,
         self.avoidance_turn_yaw_source) = self._select_avoidance_turn_yaw(now)
        self._transition(state, now)

    def _current_avoidance_turn_yaw(self, now: float):
        if self.avoidance_turn_yaw_source != 'imu':
            return self.odom_yaw
        if (self.last_imu_time is None
                or now - self.last_imu_time > 0.5):
            return None
        return self.imu_yaw

    def _start_avoidance_turn_back(self, now: float):
        self._begin_avoidance_turn(STATE_OBSTACLE_TURN_BACK, now)

    def _finish_avoidance_maneuver(self, now: float):
        self.current_wp_idx = self.avoidance_return_wp_idx
        self.avoidance_suppress_until = now + self.avoidance_retrigger_suppression
        self._transition(STATE_PATROL, now)
        self._send_next_waypoint()

    def _avoidance_drive_correction(self) -> float:
        side_clearance = self._side_clearance(self.avoidance_side)
        other_clearance = self._side_clearance(-self.avoidance_side)
        if side_clearance < self.avoidance_side_soft_min:
            return -self.avoidance_side * self.avoidance_correction_angular_speed
        if other_clearance < self.avoidance_side_soft_min:
            return self.avoidance_side * self.avoidance_correction_angular_speed
        if side_clearance < self.avoidance_side_target:
            return -self.avoidance_side * 0.5 * self.avoidance_correction_angular_speed
        return 0.0
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

    def _apply_fixed_point_diagnosis_override(self, diagnosis: Diagnosis) -> Diagnosis:
        if self.active_fixed_point_disease is None:
            return diagnosis

        phase = (self.get_clock().now().nanoseconds // 100_000_000) % 11
        confidence = 0.80 + phase / 100.0
        diagnosis.disease_class = self.active_fixed_point_disease
        diagnosis.class_id = FIXED_POINT_DIAGNOSIS_CLASS_ID
        diagnosis.confidence = confidence
        diagnosis.probabilities = [
            confidence if label == self.active_fixed_point_disease
            else (1.0 - confidence) / (len(TOMATO_DISEASE_CLASSES) - 1)
            for label in TOMATO_DISEASE_CLASSES
        ]
        return diagnosis

    # ---- State machine ----

    def tick(self):
        now = self.get_clock().now().nanoseconds / 1e9

        # Background Nav2 readiness via NavigateToPose action server check.
        if not self._nav2_ready:
            if not hasattr(self, '_nav2_tick_count'):
                self._nav2_tick_count = 0
            self._nav2_tick_count += 1
            ready = False
            nav_client = getattr(self.navigator, 'nav_to_pose_client', None)
            if nav_client is not None:
                try:
                    ready = nav_client.wait_for_server(timeout_sec=0.0)
                except Exception:
                    pass
            if ready:
                self._nav2_ready = True
                self.get_logger().info(
                    f'Nav2 ready (ticks={self._nav2_tick_count}, '
                    f'server={ready})')
                if should_send_patrol_goal(
                        self.state,
                        self._nav2_ready,
                        self.sending_goal,
                        self.current_wp_idx,
                        len(self.waypoints)) and now >= self._next_goal_time:
                    self._send_next_waypoint()
            elif self._nav2_tick_count % 100 == 0:
                self.get_logger().warn(
                    f'Waiting for Nav2 action server (ticks={self._nav2_tick_count})')

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

            if self._front_obstacle_too_close():
                self._trigger_obstacle_avoidance(now)

            fixed_point_stop = self._find_unhandled_fixed_point_stop()
            if fixed_point_stop is not None:
                self._accept_fixed_point_stop(*fixed_point_stop, now)

            if should_send_patrol_goal(
                    self.state,
                    self._nav2_ready,
                    self.sending_goal,
                    self.current_wp_idx,
                    len(self.waypoints)
            ) and now >= self._next_goal_time:
                self._send_next_waypoint()

            if (self.sending_goal
                    and now - self.last_goal_sent_time >= 0.5
                    and self.navigator.isTaskComplete()):
                self.sending_goal = False
                self.last_goal_sent_time = 0.0
                result = self.navigator.getResult()
                if result == TaskResult.SUCCEEDED:
                    self.current_wp_idx += 1
                    self.get_logger().info(
                        f'Reached waypoint {self.current_wp_idx - 1}')
                    if self.current_wp_idx < len(self.waypoints):
                        self._send_next_waypoint()
                    else:
                        self.get_logger().info('All waypoints completed')
                else:
                    self.get_logger().warn(
                        f'Nav2 task failed ({result}), '
                        f'retrying waypoint {self.current_wp_idx} after delay')
                    self._next_goal_time = now + 2.0

            # Check for plant detection trigger (with de-duplication)
            if (self.state == STATE_PATROL
                    and self.last_plant is not None
                    and self.last_plant.detected
                    and self.last_plant.confidence >= self.det_conf_th
                    and self.last_plant.area_ratio >= self.min_area_ratio
                    and self._should_trigger_scan()):
                self.saved_wp_idx = self.current_wp_idx
                self.last_fusion = None
                self._diagnosis_published_at_ns = 0
                self.plants_detected += 1
                self.get_logger().info(
                    'Plant stop trigger accepted: '
                    f'confidence={self.last_plant.confidence:.3f}, '
                    f'area_ratio={self.last_plant.area_ratio:.3f}, '
                    f'distance={self._distance_from_reference():.3f}m')
                self._cancel_nav2_task_async()
                self.sending_goal = False
                self.last_goal_sent_time = 0.0
                self._transition(STATE_STOPPED, now)

        elif self.state == STATE_OBSTACLE_STOP:
            status.state = STATE_OBSTACLE_STOP
            status.current_action = 'stopping for obstacle'
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            elapsed = now - self.state_enter_time
            if elapsed >= self.obstacle_resume_delay:
                self.avoidance_backup_start_x = self.odom_x
                self.avoidance_backup_start_y = self.odom_y
                self._transition(STATE_OBSTACLE_BACKUP, now)

        elif self.state == STATE_OBSTACLE_BACKUP:
            status.state = STATE_OBSTACLE_BACKUP
            status.current_action = 'backing up before avoidance'
            elapsed = now - self.state_enter_time
            backed = math.hypot(
                self.odom_x - self.avoidance_backup_start_x,
                self.odom_y - self.avoidance_backup_start_y)
            if backed < self.avoidance_backup_distance and elapsed < self.avoidance_backup_timeout:
                cmd.linear.x = self.avoidance_backup_speed
                cmd.angular.z = 0.0
            else:
                cmd.linear.x = 0.0
                cmd.angular.z = 0.0
                self._begin_avoidance_turn(STATE_OBSTACLE_TURN, now)

        elif self.state == STATE_OBSTACLE_TURN:
            status.state = STATE_OBSTACLE_TURN
            status.current_action = 'turning around obstacle'
            turn_yaw = self._current_avoidance_turn_yaw(now)
            turned = (abs(self._angle_diff(
                turn_yaw, self.avoidance_turn_start_yaw))
                if turn_yaw is not None else 0.0)
            if turn_yaw is None:
                cmd.linear.x = 0.0
                cmd.angular.z = 0.0
            elif turned < self.avoidance_turn_angle:
                cmd.linear.x = 0.0
                cmd.angular.z = self.avoidance_side * self.avoidance_turn_speed
            else:
                cmd.linear.x = 0.0
                cmd.angular.z = 0.0
                self.avoidance_drive_start_x = self.odom_x
                self.avoidance_drive_start_y = self.odom_y
                self._transition(STATE_OBSTACLE_ARC_DRIVE, now)

        elif self.state == STATE_OBSTACLE_ARC_DRIVE:
            status.state = STATE_OBSTACLE_ARC_DRIVE
            status.current_action = 'driving around obstacle'
            elapsed = now - self.state_enter_time
            driven = math.hypot(
                self.odom_x - self.avoidance_drive_start_x,
                self.odom_y - self.avoidance_drive_start_y)
            if (driven >= self.avoidance_drive_distance
                    or elapsed >= self.avoidance_drive_timeout
                    or self._avoidance_internal_obstacle_too_close()):
                cmd.linear.x = 0.0
                cmd.angular.z = 0.0
                self._start_avoidance_turn_back(now)
            else:
                cmd.linear.x = self.avoidance_drive_speed
                cmd.angular.z = 0.0

        elif self.state == STATE_OBSTACLE_TURN_BACK:
            status.state = STATE_OBSTACLE_TURN_BACK
            status.current_action = 'turning back to original heading'
            turn_yaw = self._current_avoidance_turn_yaw(now)
            turned = (abs(self._angle_diff(
                turn_yaw, self.avoidance_turn_start_yaw))
                if turn_yaw is not None else 0.0)
            if turn_yaw is None:
                cmd.linear.x = 0.0
                cmd.angular.z = 0.0
            elif turned < self.avoidance_turn_angle:
                cmd.linear.x = 0.0
                cmd.angular.z = -self.avoidance_side * self.avoidance_turn_speed
            else:
                cmd.linear.x = 0.0
                cmd.angular.z = 0.0
                self.avoidance_drive_start_x = self.odom_x
                self.avoidance_drive_start_y = self.odom_y
                self._transition(STATE_OBSTACLE_REJOIN_FORWARD, now)

        elif self.state == STATE_OBSTACLE_REJOIN_FORWARD:
            status.state = STATE_OBSTACLE_REJOIN_FORWARD
            status.current_action = 'rejoining patrol route'
            elapsed = now - self.state_enter_time
            driven = math.hypot(
                self.odom_x - self.avoidance_drive_start_x,
                self.odom_y - self.avoidance_drive_start_y)
            if driven >= self.avoidance_rejoin_distance or elapsed >= self.avoidance_rejoin_timeout:
                cmd.linear.x = 0.0
                cmd.angular.z = 0.0
                self._finish_avoidance_maneuver(now)
            else:
                cmd.linear.x = self.avoidance_drive_speed
                cmd.angular.z = 0.0

        elif self.state == STATE_AVOIDING:
            status.state = STATE_AVOIDING
            status.current_action = 'avoiding obstacle'
            self.current_wp_idx = self.avoidance_return_wp_idx
            self._transition(STATE_PATROL, now)
            self._send_next_waypoint()
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
                        self._diagnosis_published_at_ns = (
                            self.get_clock().now().nanoseconds)
                        self.last_fusion = None
                        diagnosis = self._apply_fixed_point_diagnosis_override(
                            result.result)
                        self.pub_diag.publish(diagnosis)
                        self.get_logger().info(
                            f'Pipeline result: {diagnosis.disease_class} '
                            f'conf={diagnosis.confidence:.3f}')
                        self.reference_x = self.odom_x
                        self.reference_y = self.odom_y
                        self.has_scan_reference = True
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
            self.active_fixed_point_disease = None

        elif self.state == STATE_RESUME:
            status.state = STATE_RESUME
            status.current_action = 'resuming patrol'
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            elapsed = now - self.state_enter_time
            if elapsed >= self.resume_delay:
                self._transition(STATE_PATROL, now)
                self.current_wp_idx = self._saved_patrol_index()
                self._send_next_waypoint()

        elif self.state == STATE_MANUAL:
            status.state = STATE_MANUAL
            status.current_action = 'manual control'

        status.progress = self._compute_progress()
        self.pub_status.publish(status)
        if self.state == STATE_PATROL:
            self._publish_nav_cmd(now)
        elif self.state in _CMDV_OWNER_STATES:
            self.pub_cmd.publish(cmd)

    def _publish_nav_cmd(self, now: float):
        if self.last_nav_cmd is None:
            return
        if now - self.last_nav_cmd_time > 0.5:
            self._publish_stop()
            return
        if self.last_nav_cmd.linear.x < -1e-3:
            self.get_logger().warn(
                'Blocked Nav2 reverse cmd in PATROL; mission owns avoidance')
            self._publish_stop()
            return
        if (abs(self.last_nav_cmd.angular.z) > 0.6
                and abs(self.last_nav_cmd.linear.x) < 0.02):
            self.get_logger().warn(
                'Blocked Nav2 recovery spin cmd in PATROL; mission owns avoidance')
            self._publish_stop()
            return
        self.pub_cmd.publish(self.last_nav_cmd)
    def _transition(self, new_state: str, now: float = None):
        if now is None:
            now = self.get_clock().now().nanoseconds / 1e9
        if self.state != new_state:
            self.get_logger().info(f'State: {self.state} -> {new_state}')
        self.state = new_state
        self.state_enter_time = now

    def _saved_patrol_index(self) -> int:
        if self.saved_wp_idx < 0:
            return 0
        if self.waypoints and self.saved_wp_idx >= len(self.waypoints):
            return 0
        return self.saved_wp_idx

    def _publish_stop(self):
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        self.pub_cmd.publish(cmd)

    def _prepare_autonomous_start(self):
        self._publish_stop()
        self.sending_goal = False
        self.last_goal_sent_time = 0.0
        self.last_plant = None
        self.active_fixed_point_disease = None
        self.handled_fixed_point_stops.clear()
        self.has_scan_reference = False
        self._next_goal_time = self.get_clock().now().nanoseconds / 1e9 + 0.8
        self._call_trigger_service_async(
            self.reset_wheel_odom_client, 'wheel odometry reset')
        self._call_trigger_service_async(
            self.reset_encoder_client, 'STM32 encoder reset')
        self._reset_ekf_pose_async()

    def _call_trigger_service_async(self, client, label: str):
        if not client.wait_for_service(timeout_sec=0.05):
            self.get_logger().warn(f'{label} service unavailable')
            return
        future = client.call_async(Trigger.Request())

        def _log_result(done_future):
            try:
                result = done_future.result()
            except Exception as exc:
                self.get_logger().warn(f'{label} failed: {exc}')
                return
            if result is None or not result.success:
                message = '' if result is None else result.message
                self.get_logger().warn(f'{label} rejected: {message}')
            else:
                self.get_logger().info(f'{label}: {result.message}')

        future.add_done_callback(_log_result)

    def _reset_ekf_pose_async(self):
        if not self.set_pose_client.wait_for_service(timeout_sec=0.05):
            self.get_logger().warn('EKF set_pose service unavailable')
            return

        req = SetPose.Request()
        req.pose.header.stamp = self.get_clock().now().to_msg()
        req.pose.header.frame_id = 'odom'
        req.pose.pose.pose.position.x = 0.0
        req.pose.pose.pose.position.y = 0.0
        req.pose.pose.pose.position.z = 0.0
        req.pose.pose.pose.orientation.w = 1.0
        req.pose.pose.covariance[0] = 0.01
        req.pose.pose.covariance[7] = 0.01
        req.pose.pose.covariance[35] = 0.01
        future = self.set_pose_client.call_async(req)

        def _log_result(done_future):
            try:
                done_future.result()
            except Exception as exc:
                self.get_logger().warn(f'EKF pose reset failed: {exc}')
                return
            self.get_logger().info('EKF pose reset to odom origin')

        future.add_done_callback(_log_result)

    def _cancel_nav2_task_async(self):
        if self._cancel_in_progress:
            return
        self._cancel_in_progress = True

        def _cancel():
            try:
                with self._nav_goal_lock:
                    self.navigator.cancelTask()
            except Exception as exc:
                self.get_logger().warn(f'Nav2 cancel failed: {exc}')
            finally:
                self._cancel_in_progress = False

        threading.Thread(target=_cancel, daemon=True).start()

    def _compute_progress(self) -> float:
        if self.plants_detected == 0:
            return 0.0
        return min(1.0, self.plants_analyzed / max(1, self.plants_detected))


def main(args=None):
    rclpy.init(args=args)
    node = MissionControlNode()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
