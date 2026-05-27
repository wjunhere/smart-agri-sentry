import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sentry_interfaces.msg import (
    PlantDetection, FusionResult, MissionStatus, ChassisStatus)


STATE_CRUISING = 'CRUISING'
STATE_APPROACHING = 'APPROACHING'
STATE_STOPPED = 'STOPPED'
STATE_ANALYZING = 'ANALYZING'
STATE_ACTION = 'ACTION'
STATE_RESUME = 'RESUME'


class MissionControlNode(Node):
    def __init__(self):
        super().__init__('mission_control_node')
        self.declare_parameter('cruise_speed', 0.3)
        self.declare_parameter('approach_speed', 0.15)
        self.declare_parameter('detection_confidence_threshold', 0.6)
        self.declare_parameter('min_area_ratio', 0.1)
        self.declare_parameter('stop_distance_tolerance', 0.05)
        self.declare_parameter('analyze_timeout_sec', 5.0)
        self.declare_parameter('resume_delay_sec', 2.0)
        self.declare_parameter('bbox_center_tolerance', 0.15)

        self.cruise_speed = self.get_parameter('cruise_speed').value
        self.approach_speed = self.get_parameter('approach_speed').value
        self.det_conf_th = self.get_parameter(
            'detection_confidence_threshold').value
        self.min_area_ratio = self.get_parameter('min_area_ratio').value
        self.stop_tol = self.get_parameter('stop_distance_tolerance').value
        self.analyze_timeout = self.get_parameter('analyze_timeout_sec').value
        self.resume_delay = self.get_parameter('resume_delay_sec').value
        self.bbox_center_tol = self.get_parameter(
            'bbox_center_tolerance').value

        self.state = STATE_CRUISING
        self.state_enter_time = 0.0
        self.plants_detected = 0
        self.plants_analyzed = 0
        self.last_plant = None
        self.last_fusion = None
        self.last_chassis = None

        self.sub_plant = self.create_subscription(
            PlantDetection, '/vision/plant_detected',
            self.on_plant_detected, 10)
        self.sub_fusion = self.create_subscription(
            FusionResult, '/fusion/diagnosis', self.on_fusion, 10)
        self.sub_chassis = self.create_subscription(
            ChassisStatus, '/sentry/chassis/status', self.on_chassis, 10)

        self.pub_cmd = self.create_publisher(Twist, '/sentry/cmd_vel', 10)
        self.pub_status = self.create_publisher(
            MissionStatus, '/mission/status', 10)

        self.timer = self.create_timer(0.1, self.tick)
        self.get_logger().info('Mission control node ready')

    def on_plant_detected(self, msg: PlantDetection):
        self.last_plant = msg
        if msg.detected and msg.confidence >= self.det_conf_th:
            if msg.area_ratio >= self.min_area_ratio:
                self.plants_detected += 1

    def on_fusion(self, msg: FusionResult):
        self.last_fusion = msg

    def on_chassis(self, msg: ChassisStatus):
        self.last_chassis = msg

    def tick(self):
        now = self.get_clock().now().nanoseconds / 1e9
        if self.state_enter_time == 0.0:
            self.state_enter_time = now

        cmd = Twist()
        status = MissionStatus()
        status.header.stamp = self.get_clock().now().to_msg()
        status.plants_detected = self.plants_detected
        status.plants_analyzed = self.plants_analyzed

        if self.state == STATE_CRUISING:
            status.state = STATE_CRUISING
            status.current_action = 'cruising'
            cmd.linear.x = self.cruise_speed

            # Check for plant trigger
            if (self.last_plant is not None
                    and self.last_plant.detected
                    and self.last_plant.confidence >= self.det_conf_th
                    and self.last_plant.area_ratio >= self.min_area_ratio):
                self._transition(STATE_APPROACHING, now)

        elif self.state == STATE_APPROACHING:
            status.state = STATE_APPROACHING
            status.current_action = 'approaching plant'

            if self.last_plant is None or not self.last_plant.detected:
                # Lost plant, resume
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

                # Stop when close enough (large area_ratio)
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

            # Record the fusion result (placeholder for data logging)
            if self.last_fusion is not None:
                self.get_logger().info(
                    f'Plant #{self.plants_analyzed}: risk={self.last_fusion.risk_score:.2f} '
                    f'alert={self.last_fusion.alert_level} mode={self.last_fusion.mode}')

            self._transition(STATE_RESUME, now)

        elif self.state == STATE_RESUME:
            status.state = STATE_RESUME
            status.current_action = 'resuming cruise'
            elapsed = now - self.state_enter_time
            if elapsed < self.resume_delay:
                cmd.linear.x = 0.0  # brief pause
            else:
                cmd.linear.x = self.cruise_speed
                self._transition(STATE_CRUISING, now)

        status.progress = self._compute_progress()
        self.pub_cmd.publish(cmd)
        self.pub_status.publish(status)

    def _transition(self, new_state: str, now: float):
        if self.state != new_state:
            self.get_logger().info(
                f'State: {self.state} -> {new_state}')
            self.state = new_state
            self.state_enter_time = now

    def _compute_progress(self) -> float:
        # Simple progress metric based on analyzed vs detected
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
