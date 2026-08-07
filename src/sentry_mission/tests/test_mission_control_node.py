"""Tests for mission_control_node state machine."""

import pytest
import rclpy
from unittest.mock import patch, MagicMock

from sentry_mission.mission_control_node import MissionControlNode
from nav2_simple_commander.robot_navigator import TaskResult
from sentry_interfaces.msg import Diagnosis, FusionResult, ObstacleInfo, PlantDetection


@pytest.fixture(scope='module')
def ros_context():
    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.fixture
def node(ros_context):
    with patch('sentry_mission.mission_control_node.BasicNavigator'):
        n = MissionControlNode()
        yield n
        n.destroy_node()


def test_plant_subscription_uses_sensor_callback_group(node):
    """Plant callbacks must run independently of the patrol timer."""
    assert node.sub_plant.callback_group is node.sensor_callback_group

def test_resume_does_not_publish_cruise_speed(node):
    """RESUME state should not publish non-zero cmd_vel even after delay expires."""
    node.state = 'RESUME'
    node.state_enter_time = 0.0
    node.resume_delay = 0.0  # Immediately expired

    with patch.object(node.pub_cmd, 'publish') as mock_cmd, \
         patch.object(node, '_transition') as mock_transition, \
         patch.object(node, '_send_next_waypoint') as mock_send:
        node.tick()

        assert mock_cmd.called
        cmd = mock_cmd.call_args[0][0]
        assert cmd.linear.x == 0.0
        assert cmd.angular.z == 0.0


def test_nav2_task_failed_retries_same_waypoint(node):
    """PATROL state: Nav2 failure keeps waypoint and schedules a delayed retry."""
    node.state = 'PATROL'
    node._nav2_ready = True
    node.current_wp_idx = 1
    node.sending_goal = True
    node.waypoints = [
        {'x': 0.0, 'y': 0.0, 'yaw': 0.0},
        {'x': 1.0, 'y': 0.0, 'yaw': 0.0},
        {'x': 2.0, 'y': 0.0, 'yaw': 0.0},
    ]

    with patch.object(node.navigator, 'isTaskComplete', return_value=True), \
         patch.object(node.navigator, 'getResult', return_value=TaskResult.FAILED), \
         patch.object(node, '_send_next_waypoint') as mock_send:
        node.tick()

        assert node.current_wp_idx == 1
        assert node.sending_goal == False
        assert node._next_goal_time > 0.0
        mock_send.assert_not_called()


def test_nav2_task_succeeded_advances_waypoint(node):
    """PATROL state: successful Nav2 task advances to next waypoint."""
    node.state = 'PATROL'
    node._nav2_ready = True
    node.current_wp_idx = 1
    node.sending_goal = True
    node.waypoints = [
        {'x': 0.0, 'y': 0.0, 'yaw': 0.0},
        {'x': 1.0, 'y': 0.0, 'yaw': 0.0},
        {'x': 2.0, 'y': 0.0, 'yaw': 0.0},
    ]

    with patch.object(node.navigator, 'isTaskComplete', return_value=True), \
         patch.object(node.navigator, 'getResult', return_value=TaskResult.SUCCEEDED):
        node.tick()

    assert node.current_wp_idx == 2
    assert node.sending_goal == True  # Re-set by _send_next_waypoint


def test_auto_mode_prepares_autonomous_start(node):
    """AUTO transition should reset odometry/encoders before sending a goal."""
    from std_srvs.srv import SetBool

    node.state = 'MANUAL'
    node.saved_wp_idx = 1
    node.current_wp_idx = 0
    node.sending_goal = True

    with patch.object(node, '_prepare_autonomous_start') as mock_prepare:
        request = SetBool.Request()
        request.data = True
        response = node.set_auto_mode_cb(request, SetBool.Response())

    assert response.success is True
    assert node.state == 'PATROL'
    assert node.current_wp_idx == 1
    mock_prepare.assert_called_once()


def test_prepare_autonomous_start_calls_reset_services(node):
    """Verify autonomous start asynchronously requests odom, encoder, and EKF resets."""
    reset_odom = MagicMock()
    reset_encoder = MagicMock()
    set_pose = MagicMock()
    reset_odom.wait_for_service.return_value = True
    reset_encoder.wait_for_service.return_value = True
    set_pose.wait_for_service.return_value = True
    reset_odom.call_async.return_value.add_done_callback = MagicMock()
    reset_encoder.call_async.return_value.add_done_callback = MagicMock()
    set_pose.call_async.return_value.add_done_callback = MagicMock()
    node.reset_wheel_odom_client = reset_odom
    node.reset_encoder_client = reset_encoder
    node.set_pose_client = set_pose
    node.sending_goal = True
    node.has_scan_reference = True

    with patch.object(node.pub_cmd, 'publish') as mock_stop:
        node._prepare_autonomous_start()

    assert node.sending_goal is False
    assert node.has_scan_reference is False
    assert node._next_goal_time > 0.0
    assert mock_stop.called
    reset_odom.wait_for_service.assert_called_once()
    reset_encoder.wait_for_service.assert_called_once()
    reset_odom.call_async.assert_called_once()
    reset_encoder.call_async.assert_called_once()
    set_pose.wait_for_service.assert_called_once()
    set_pose.call_async.assert_called_once()
    pose_req = set_pose.call_async.call_args[0][0]
    assert pose_req.pose.header.frame_id == 'odom'
    assert pose_req.pose.pose.pose.position.x == 0.0
    assert pose_req.pose.pose.pose.position.y == 0.0
    assert pose_req.pose.pose.pose.orientation.w == 1.0


def test_patrol_close_obstacle_stops_and_builds_bypass(node):
    """PATROL state: a close front obstacle pauses the main route and prepares bypass goals."""
    node.state = 'PATROL'
    node._nav2_ready = True
    node.current_wp_idx = 0
    node.sending_goal = True
    node.odom_x = 0.0
    node.odom_y = 0.0
    node.odom_yaw = 0.0
    node.waypoints = [{'x': 2.5, 'y': 0.0, 'yaw': 1.5708}]
    node.last_obstacle = ObstacleInfo()
    node.last_obstacle.front_min_distance = 0.45

    with patch.object(node, '_cancel_nav2_task_async') as mock_cancel, \
         patch.object(node.pub_cmd, 'publish') as mock_stop, \
         patch.object(node.navigator, 'isTaskComplete', return_value=False):
        node.tick()

    assert node.state == 'OBSTACLE_STOP'
    assert node.sending_goal is False
    assert node.avoidance_return_wp_idx == 0
    assert node.avoidance_goals == []
    assert mock_cancel.called
    assert mock_stop.called


def test_patrol_ignores_obstacle_beyond_current_waypoint(node):
    """PATROL state: an obstacle beyond the active waypoint should not trigger bypass."""
    node.state = 'PATROL'
    node._nav2_ready = True
    node.current_wp_idx = 0
    node.sending_goal = True
    node.odom_x = 0.0
    node.odom_y = 0.0
    node.odom_yaw = 0.0
    node.waypoints = [{'x': 0.30, 'y': 0.0, 'yaw': 0.0}]
    node.last_obstacle = ObstacleInfo()
    node.last_obstacle.front_min_distance = 0.45

    with patch.object(node, '_cancel_nav2_task_async') as mock_cancel, \
         patch.object(node.navigator, 'isTaskComplete', return_value=False):
        node.tick()

    assert node.state == 'PATROL'
    assert node.sending_goal is True
    assert node.avoidance_goals == []
    mock_cancel.assert_not_called()

def test_obstacle_stop_enters_backup_after_delay(node):
    """OBSTACLE_STOP state: after the stop delay, mission backs away before bypass."""
    node.state = 'OBSTACLE_STOP'
    node.state_enter_time = 0.0
    node.obstacle_resume_delay = 0.0
    node.odom_x = 0.7
    node.odom_y = -0.2

    with patch.object(node, '_send_pose_goal') as mock_send:
        node.tick()

    assert node.state == 'OBSTACLE_BACKUP'
    assert node.avoidance_backup_start_x == 0.7
    assert node.avoidance_backup_start_y == -0.2
    mock_send.assert_not_called()


def test_obstacle_backup_enters_turn_after_clearance(node):
    """OBSTACLE_BACKUP state: after backing up enough, mission starts a direct turn."""
    node.state = 'OBSTACLE_BACKUP'
    node.state_enter_time = 0.0
    node.avoidance_backup_distance = 0.18
    node.avoidance_backup_timeout = 2.5
    node.avoidance_backup_start_x = 0.0
    node.avoidance_backup_start_y = 0.0
    node.odom_x = -0.19
    node.odom_y = 0.0
    node.odom_yaw = 0.25

    node.tick()

    assert node.state == 'OBSTACLE_TURN'
    assert node.avoidance_turn_start_yaw == 0.25


def test_obstacle_turn_enters_arc_drive_after_angle(node):
    """OBSTACLE_TURN state: after the target angle, mission starts direct arc driving."""
    node.state = 'OBSTACLE_TURN'
    node.state_enter_time = 0.0
    node.avoidance_side = 1
    node.avoidance_turn_angle = 0.45
    node.avoidance_turn_start_yaw = 0.0
    node.odom_yaw = 0.48
    node.odom_x = 1.0
    node.odom_y = -0.2

    node.tick()

    assert node.state == 'OBSTACLE_ARC_DRIVE'
    assert node.avoidance_drive_start_x == 1.0
    assert node.avoidance_drive_start_y == -0.2


def test_obstacle_turn_uses_fresh_imu_yaw_when_odom_yaw_is_stale(node):
    """An IMU heading completes the turn when encoder odometry has frozen."""
    now = node.get_clock().now().nanoseconds / 1e9
    node.state = 'OBSTACLE_TURN'
    node.state_enter_time = now
    node.avoidance_side = 1
    node.avoidance_turn_angle = 0.45
    node.avoidance_turn_start_yaw = 0.0
    node.avoidance_turn_yaw_source = 'imu'
    node.avoidance_turn_timeout = 5.0
    node.odom_yaw = 0.0
    node.imu_yaw = 0.48
    node.last_imu_time = now
    node.odom_x = 1.0
    node.odom_y = -0.2

    node.tick()

    assert node.state == 'OBSTACLE_ARC_DRIVE'


def test_obstacle_turn_waits_for_heading_not_elapsed_time(node):
    """A turn only advances after the selected yaw source reaches its target."""
    now = node.get_clock().now().nanoseconds / 1e9
    node.state = 'OBSTACLE_TURN'
    node.state_enter_time = now - 10.0
    node.avoidance_side = 1
    node.avoidance_turn_angle = 0.45
    node.avoidance_turn_start_yaw = 0.0
    node.avoidance_turn_yaw_source = 'imu'
    node.odom_yaw = 0.0
    node.imu_yaw = 0.20
    node.last_imu_time = now

    with patch.object(node.pub_cmd, 'publish') as mock_cmd:
        node.tick()

    cmd = mock_cmd.call_args[0][0]
    assert node.state == 'OBSTACLE_TURN'
    assert cmd.linear.x == 0.0
    assert cmd.angular.z == node.avoidance_turn_speed


def test_obstacle_arc_drive_enters_turn_back_after_distance(node):
    """OBSTACLE_ARC_DRIVE state: after the bypass distance, mission turns back before resuming Nav2."""
    node.state = 'OBSTACLE_ARC_DRIVE'
    node._nav2_ready = True
    node.state_enter_time = 0.0
    node.avoidance_return_wp_idx = 1
    node.avoidance_drive_distance = 0.65
    node.avoidance_drive_timeout = 10.0
    node.avoidance_drive_start_x = 0.0
    node.avoidance_drive_start_y = 0.0
    node.odom_x = 0.66
    node.odom_y = 0.0
    node.odom_yaw = 0.25

    with patch.object(node, '_send_next_waypoint') as mock_send:
        node.tick()

    assert node.state == 'OBSTACLE_TURN_BACK'
    assert node.avoidance_turn_start_yaw == 0.25
    mock_send.assert_not_called()


def test_obstacle_arc_drive_enters_turn_back_from_hard_front_obstacle(node):
    """OBSTACLE_ARC_DRIVE state: a very close front obstacle ends bypass forward and turns back next."""
    node.state = 'OBSTACLE_ARC_DRIVE'
    node.state_enter_time = 0.0
    node.avoidance_side = -1
    node.avoidance_internal_hard_stop = 0.20
    node.avoidance_drive_distance = 0.65
    node.avoidance_drive_timeout = 10.0
    node.odom_yaw = -0.5
    node.last_obstacle = ObstacleInfo()
    node.last_obstacle.front_min_distance = 0.18

    with patch.object(node.pub_cmd, 'publish') as mock_cmd:
        node.tick()

    cmd = mock_cmd.call_args[0][0]
    assert node.state == 'OBSTACLE_TURN_BACK'
    assert node.avoidance_turn_start_yaw == -0.5
    assert cmd.linear.x == 0.0
    assert cmd.angular.z == 0.0



def test_bypass_forward_uses_internal_hard_stop_not_patrol_threshold(node):
    """The bypass forward leg should ignore old patrol/soft thresholds above the internal hard stop."""
    node.state = 'OBSTACLE_ARC_DRIVE'
    node.state_enter_time = 0.0
    node.avoidance_side = 1
    node.avoidance_drive_speed = 0.08
    node.avoidance_drive_distance = 0.65
    node.avoidance_drive_timeout = 10.0
    node.avoidance_drive_start_x = 0.0
    node.avoidance_drive_start_y = 0.0
    node.avoidance_front_hard_stop = 0.50
    node.avoidance_internal_hard_stop = 0.20
    node.odom_x = 0.2
    node.odom_y = 0.0
    node.last_obstacle = ObstacleInfo()
    node.last_obstacle.front_min_distance = 0.30

    with patch.object(node, '_side_clearance', return_value=float('inf')), \
         patch.object(node.pub_cmd, 'publish') as mock_cmd:
        node.tick()

    cmd = mock_cmd.call_args[0][0]
    assert node.state == 'OBSTACLE_ARC_DRIVE'
    assert cmd.linear.x == 0.08
    assert cmd.angular.z == 0.0


def test_bypass_forward_enters_turn_back_on_internal_front_or_side_hard_stop(node):
    """During the bypass forward leg, <=0.20m front/side clearance ends the leg instead of retriggering avoidance."""
    node.state = 'OBSTACLE_ARC_DRIVE'
    node.state_enter_time = 0.0
    node.avoidance_side = 1
    node.avoidance_drive_distance = 0.65
    node.avoidance_drive_timeout = 10.0
    node.avoidance_drive_start_x = 0.0
    node.avoidance_drive_start_y = 0.0
    node.avoidance_internal_hard_stop = 0.20
    node.odom_x = 0.2
    node.odom_y = 0.0
    node.odom_yaw = 0.42
    node.last_obstacle = ObstacleInfo()
    node.last_obstacle.front_min_distance = 0.19

    with patch.object(node, '_side_clearance', return_value=float('inf')), \
         patch.object(node.pub_cmd, 'publish') as mock_cmd:
        node.tick()

    cmd = mock_cmd.call_args[0][0]
    assert node.state == 'OBSTACLE_TURN_BACK'
    assert node.avoidance_turn_start_yaw == 0.42
    assert cmd.linear.x == 0.0
    assert cmd.angular.z == 0.0



def test_bypass_forward_ignores_side_clearance_above_side_hard_stop(node):
    """Side plants above the dedicated side hard stop should not end the bypass forward leg."""
    node.state = 'OBSTACLE_ARC_DRIVE'
    node.state_enter_time = 0.0
    node.avoidance_side = 1
    node.avoidance_drive_speed = 0.08
    node.avoidance_drive_distance = 0.65
    node.avoidance_drive_timeout = 10.0
    node.avoidance_drive_start_x = 0.0
    node.avoidance_drive_start_y = 0.0
    node.avoidance_internal_hard_stop = 0.20
    node.avoidance_internal_side_hard_stop = 0.05
    node.odom_x = 0.2
    node.odom_y = 0.0
    node.last_obstacle = ObstacleInfo()
    node.last_obstacle.front_min_distance = 0.80

    with patch.object(node, '_side_clearance', side_effect=[0.15, 0.42]), \
         patch.object(node.pub_cmd, 'publish') as mock_cmd:
        node.tick()

    cmd = mock_cmd.call_args[0][0]
    assert node.state == 'OBSTACLE_ARC_DRIVE'
    assert cmd.linear.x == 0.08
    assert cmd.angular.z == 0.0


def test_bypass_forward_enters_turn_back_on_side_hard_stop(node):
    """Side clearance at or below 0.05m still ends the bypass forward leg."""
    node.state = 'OBSTACLE_ARC_DRIVE'
    node.state_enter_time = 0.0
    node.avoidance_side = 1
    node.avoidance_drive_distance = 0.65
    node.avoidance_drive_timeout = 10.0
    node.avoidance_drive_start_x = 0.0
    node.avoidance_drive_start_y = 0.0
    node.avoidance_internal_hard_stop = 0.20
    node.avoidance_internal_side_hard_stop = 0.05
    node.odom_x = 0.2
    node.odom_y = 0.0
    node.odom_yaw = 0.42
    node.last_obstacle = ObstacleInfo()
    node.last_obstacle.front_min_distance = 0.80

    with patch.object(node, '_side_clearance', side_effect=[0.04, 0.42]), \
         patch.object(node.pub_cmd, 'publish') as mock_cmd:
        node.tick()

    cmd = mock_cmd.call_args[0][0]
    assert node.state == 'OBSTACLE_TURN_BACK'
    assert node.avoidance_turn_start_yaw == 0.42
    assert cmd.linear.x == 0.0
    assert cmd.angular.z == 0.0


def test_bypass_forward_enters_turn_back_after_distance(node):
    """After the 0.65m bypass leg, mission turns back before rejoining Nav2."""
    node.state = 'OBSTACLE_ARC_DRIVE'
    node._nav2_ready = True
    node.state_enter_time = 0.0
    node.avoidance_return_wp_idx = 1
    node.avoidance_drive_distance = 0.65
    node.avoidance_drive_timeout = 10.0
    node.avoidance_drive_start_x = 0.0
    node.avoidance_drive_start_y = 0.0
    node.odom_x = 0.66
    node.odom_y = 0.0
    node.odom_yaw = 0.3

    with patch.object(node, '_send_next_waypoint') as mock_send:
        node.tick()

    assert node.state == 'OBSTACLE_TURN_BACK'
    assert node.avoidance_turn_start_yaw == 0.3
    mock_send.assert_not_called()


def test_turn_back_enters_rejoin_forward_after_angle(node):
    """The turn-back leg rotates opposite the first turn before the short rejoin drive."""
    node.state = 'OBSTACLE_TURN_BACK'
    node.state_enter_time = 0.0
    node.avoidance_side = 1
    node.avoidance_turn_angle = 0.60
    node.avoidance_turn_start_yaw = 1.0
    node.odom_yaw = 0.39
    node.odom_x = 1.2
    node.odom_y = -0.4

    node.tick()

    assert node.state == 'OBSTACLE_REJOIN_FORWARD'
    assert node.avoidance_drive_start_x == 1.2
    assert node.avoidance_drive_start_y == -0.4


def test_rejoin_forward_returns_to_original_waypoint_after_distance(node):
    """After the short rejoin leg, mission resumes the saved waypoint under Nav2."""
    node.state = 'OBSTACLE_REJOIN_FORWARD'
    node._nav2_ready = True
    node.state_enter_time = 0.0
    node.avoidance_return_wp_idx = 1
    node.avoidance_rejoin_distance = 0.30
    node.avoidance_drive_start_x = 0.0
    node.avoidance_drive_start_y = 0.0
    node.odom_x = 0.31
    node.odom_y = 0.0
    node.waypoints = [
        {'x': 0.0, 'y': 0.0, 'yaw': 0.0},
        {'x': 2.5, 'y': 0.0, 'yaw': 1.5708},
    ]

    with patch.object(node, '_send_next_waypoint') as mock_send:
        node.tick()

    assert node.state == 'PATROL'
    assert node.current_wp_idx == 1
    mock_send.assert_called_once()


def test_avoidance_success_returns_to_original_waypoint(node):
    """Legacy AVOIDING state falls back to the saved main waypoint."""
    node.state = 'AVOIDING'
    node._nav2_ready = True
    node.current_wp_idx = 0
    node.avoidance_return_wp_idx = 1
    node.waypoints = [
        {'x': 0.0, 'y': 0.0, 'yaw': 0.0},
        {'x': 2.5, 'y': 0.0, 'yaw': 1.5708},
    ]

    with patch.object(node, '_send_next_waypoint') as mock_send:
        node.tick()

    assert node.state == 'PATROL'
    assert node.current_wp_idx == 1
    mock_send.assert_called_once()

def test_nav_cmd_subscription_uses_velocity_callback_group(node):
    """Nav velocity callbacks must not share the timer's default callback group."""
    assert node.sub_nav_cmd.callback_group is node.velocity_callback_group

def test_auto_mode_after_completed_route_restarts_from_first_waypoint(node):
    """Starting AUTO after a completed route should begin at WP0 again."""
    from std_srvs.srv import SetBool

    node.state = 'MANUAL'
    node.waypoints = [
        {'x': 0.0, 'y': 0.0, 'yaw': 0.0},
        {'x': 1.0, 'y': 0.0, 'yaw': 0.0},
        {'x': 2.0, 'y': 0.0, 'yaw': 0.0},
    ]
    node.saved_wp_idx = len(node.waypoints)
    node.current_wp_idx = len(node.waypoints)

    request = SetBool.Request()
    request.data = True
    response = node.set_auto_mode_cb(request, SetBool.Response())

    assert response.success is True
    assert node.state == 'PATROL'
    assert node.current_wp_idx == 0


def test_plant_trigger_clears_stale_fusion_before_analysis(node):
    """A new stopped scan must wait for fusion from this diagnosis, not an old one."""
    node.state = 'PATROL'
    node._nav2_ready = True
    node.current_wp_idx = 0
    node.sending_goal = True
    node.last_fusion = FusionResult()
    detection = PlantDetection()
    detection.detected = True
    detection.confidence = 0.95
    detection.area_ratio = 0.20
    node.on_plant_detected(detection)
    node.reference_x = 0.0
    node.reference_y = 0.0
    node.odom_x = 1.0
    node.odom_y = 0.0

    with patch.object(node, '_cancel_nav2_task_async'):
        node.on_plant_detected(detection)

    assert node.state == 'STOPPED'
    assert node.last_fusion is None


def test_first_plant_trigger_stops_before_dedup_distance(node):
    """The first real plant must stop patrol even close to the start point."""
    node.state = 'PATROL'
    node._nav2_ready = True
    node.current_wp_idx = 0
    node.sending_goal = True
    node.reference_x = 0.0
    node.reference_y = 0.0
    node.odom_x = 0.10
    node.odom_y = 0.0
    detection = PlantDetection()
    detection.detected = True
    detection.confidence = 0.60
    detection.area_ratio = 0.35
    node.on_plant_detected(detection)

    assert node.state == 'STOPPED'


def test_voted_plant_detection_survives_following_negative_frame(node):
    """A confirmed plant must not be lost when the next frame is negative."""
    node.state = 'PATROL'
    node._nav2_ready = True
    node.current_wp_idx = 0
    node.sending_goal = True

    positive = PlantDetection()
    positive.detected = True
    positive.confidence = 0.60
    positive.area_ratio = 0.20
    negative = PlantDetection()
    negative.detected = False

    node.on_plant_detected(positive)
    node.on_plant_detected(negative)

    assert node.state == 'STOPPED'

def test_fusion_before_current_diagnosis_is_ignored(node):
    """Only fusion generated after this scan's diagnosis may finish analysis."""
    stale = FusionResult()
    stale.header.stamp.sec = 10
    stale.header.stamp.nanosec = 0
    node._diagnosis_published_at_ns = 10_000_000_001

    node.on_fusion(stale)

    assert node.last_fusion is None

    fresh = FusionResult()
    fresh.header.stamp.sec = 10
    fresh.header.stamp.nanosec = 2
    node.on_fusion(fresh)

    assert node.last_fusion is fresh


def test_plant_is_counted_only_when_patrol_accepts_stop_trigger(node):
    """Repeated detector frames for one stopped plant must not inflate totals."""
    node.state = 'PATROL'
    node._nav2_ready = True
    node.current_wp_idx = 0
    node.sending_goal = True
    node.reference_x = 0.0
    node.reference_y = 0.0
    node.odom_x = 1.0
    node.odom_y = 0.0
    detection = PlantDetection()
    detection.detected = True
    detection.confidence = 0.95
    detection.area_ratio = 0.20

    node.on_plant_detected(detection)
    node.on_plant_detected(detection)

    assert node.state == 'STOPPED'
    assert node.plants_detected == 1


def test_plant_detection_during_obstacle_turn_is_not_latched(node):
    """Avoidance frames must not cause a delayed plant stop after rejoin."""
    node.state = 'OBSTACLE_TURN'
    node.last_plant = None
    detection = PlantDetection()
    detection.detected = True
    detection.confidence = 0.95
    detection.area_ratio = 0.20

    node.on_plant_detected(detection)

    assert node.last_plant is None


def test_obstacle_entry_clears_pending_plant_detection(node):
    """An obstacle must take priority over a plant frame from the prior patrol tick."""
    node.state = 'PATROL'
    node._nav2_ready = True
    node.current_wp_idx = 0
    node.sending_goal = True
    node.odom_x = 0.0
    node.odom_y = 0.0
    node.odom_yaw = 0.0
    node.waypoints = [{'x': 2.5, 'y': 0.0, 'yaw': 0.0}]
    node.last_obstacle = ObstacleInfo()
    node.last_obstacle.front_min_distance = 0.45
    node.last_plant = PlantDetection()
    node.last_plant.detected = True
    node.last_plant.confidence = 0.95
    node.last_plant.area_ratio = 0.20

    with patch.object(node, '_cancel_nav2_task_async'), \
         patch.object(node.pub_cmd, 'publish'), \
         patch.object(node.navigator, 'isTaskComplete', return_value=False):
        node.tick()

    assert node.state == 'OBSTACLE_STOP'
    assert node.last_plant is None


def test_patrol_enters_stop_state_at_configured_fixed_point(node):
    node.state = 'PATROL'
    node._nav2_ready = True
    node.current_wp_idx = 0
    node.sending_goal = True
    node.fixed_point_stops = [{
        'x': 1.0,
        'y': -0.5,
        'radius': 0.2,
        'disease_class': 'late_blight',
    }]
    node.handled_fixed_point_stops = set()
    node.odom_x = 1.1
    node.odom_y = -0.5

    with patch.object(node, '_cancel_nav2_task_async'), \
         patch.object(node.navigator, 'isTaskComplete', return_value=False):
        node.tick()

    assert node.state == 'STOPPED'
    assert node.active_fixed_point_disease == 'late_blight'
    assert node.handled_fixed_point_stops == {0}


def test_fixed_point_does_not_trigger_outside_configured_radius(node):
    node.fixed_point_stops = [{
        'x': 1.0,
        'y': 0.0,
        'radius': 0.2,
        'disease_class': 'healthy',
    }]
    node.handled_fixed_point_stops = set()
    node.odom_x = 1.21
    node.odom_y = 0.0

    assert node._find_unhandled_fixed_point_stop() is None


def test_fixed_point_does_not_trigger_again_in_same_patrol(node):
    node.fixed_point_stops = [{
        'x': 1.0,
        'y': 0.0,
        'radius': 0.2,
        'disease_class': 'healthy',
    }]
    node.handled_fixed_point_stops = {0}
    node.odom_x = 1.0
    node.odom_y = 0.0

    assert node._find_unhandled_fixed_point_stop() is None


def test_fixed_point_does_not_trigger_during_obstacle_turn(node):
    node.state = 'OBSTACLE_TURN'
    node.fixed_point_stops = [{
        'x': 0.0,
        'y': 0.0,
        'radius': 0.2,
        'disease_class': 'early_blight',
    }]
    node.handled_fixed_point_stops = set()
    node.odom_x = 0.0
    node.odom_y = 0.0

    assert node._find_unhandled_fixed_point_stop() is None


def test_fixed_point_diagnosis_uses_selected_disease_with_bounded_confidence(node):
    node.active_fixed_point_disease = 'early_blight'
    diagnosis = Diagnosis()
    diagnosis.disease_class = 'healthy'
    diagnosis.confidence = 0.35

    overridden = node._apply_fixed_point_diagnosis_override(diagnosis)

    assert overridden.disease_class == 'early_blight'
    assert 0.80 <= overridden.confidence <= 0.90


def test_normal_diagnosis_is_unchanged_without_active_fixed_point(node):
    node.active_fixed_point_disease = None
    diagnosis = Diagnosis()
    diagnosis.disease_class = 'healthy'
    diagnosis.confidence = 0.72

    result = node._apply_fixed_point_diagnosis_override(diagnosis)

    assert result is diagnosis
    assert result.disease_class == 'healthy'
    assert result.confidence == 0.72


# ---- Avoidance suppression for already-scanned plants ----

def _arm_obstacle(node):
    """Set up a front obstacle that would normally trigger avoidance."""
    node.enable_obstacle_avoidance = True
    node.state = 'PATROL'
    node.odom_x = 0.0
    node.odom_y = 0.0
    node.waypoints = [{'x': 10.0, 'y': 10.0, 'yaw': 0.0}]
    node.current_wp_idx = 0
    node.avoidance_suppress_until = 0.0
    obstacle = ObstacleInfo()
    obstacle.front_min_distance = 0.3
    node.last_obstacle = obstacle


def test_scanned_plant_suppresses_obstacle_avoidance(node):
    """Within avoidance_scanned_radius of a scanned plant: no avoidance."""
    _arm_obstacle(node)
    node._scanned_plant_positions = [(0.2, 0.1)]
    assert node._front_obstacle_too_close() is False


def test_unscanned_obstacle_still_triggers_avoidance(node):
    """Away from any scanned plant: avoidance triggers as before."""
    _arm_obstacle(node)
    node._scanned_plant_positions = [(5.0, 5.0)]
    assert node._front_obstacle_too_close() is True


def test_scanned_positions_cleared_on_mission_start(node):
    """New cruise resets odometry, so stale scan positions must be dropped."""
    node._scanned_plant_positions = [(1.0, 2.0)]
    with patch.object(node, 'pub_cmd'), \
         patch.object(node, 'reset_wheel_odom_client') as c1, \
         patch.object(node, 'reset_encoder_client') as c2, \
         patch.object(node, '_reset_ekf_pose_async'):
        c1.wait_for_service.return_value = False
        c2.wait_for_service.return_value = False
        node._prepare_autonomous_start()
    assert node._scanned_plant_positions == []


def test_patrol_start_resumes_detector(node):
    """Entering AUTO must unpause the plant detector when it is available."""
    with patch.object(node, 'pub_cmd'), \
         patch.object(node, 'reset_wheel_odom_client') as c1, \
         patch.object(node, 'reset_encoder_client') as c2, \
         patch.object(node, '_reset_ekf_pose_async'), \
         patch.object(node, 'pause_detector_client') as pc:
        c1.wait_for_service.return_value = False
        c2.wait_for_service.return_value = False
        pc.service_is_ready.return_value = True
        node._prepare_autonomous_start()
    pc.call_async.assert_called_once()
    assert pc.call_async.call_args[0][0].data is False


def test_patrol_start_warns_when_detector_missing(node):
    """Detector process dead: no resume call, but start must not crash."""
    with patch.object(node, 'pub_cmd'), \
         patch.object(node, 'reset_wheel_odom_client') as c1, \
         patch.object(node, 'reset_encoder_client') as c2, \
         patch.object(node, '_reset_ekf_pose_async'), \
         patch.object(node, 'pause_detector_client') as pc:
        c1.wait_for_service.return_value = False
        c2.wait_for_service.return_value = False
        pc.service_is_ready.return_value = False
        node._prepare_autonomous_start()
    pc.call_async.assert_not_called()


def test_patrol_start_reloads_waypoints(node, tmp_path):
    """Waypoint edits made while the stack is resident apply to the next cruise."""
    import yaml as _yaml
    wp_file = tmp_path / 'waypoints.yaml'
    wp_file.write_text(_yaml.safe_dump({'waypoints': [
        {'x': 5.0, 'y': 5.0, 'yaw': 0.0},
        {'x': 6.0, 'y': 5.0, 'yaw': 0.0},
    ]}))
    node.waypoints_file = str(wp_file)
    node.waypoints = [{'x': 1.0, 'y': 1.0, 'yaw': 0.0}]
    with patch.object(node, 'pub_cmd'), \
         patch.object(node, 'reset_wheel_odom_client') as c1, \
         patch.object(node, 'reset_encoder_client') as c2, \
         patch.object(node, '_reset_ekf_pose_async'), \
         patch.object(node, 'pause_detector_client') as pc:
        c1.wait_for_service.return_value = False
        c2.wait_for_service.return_value = False
        pc.service_is_ready.return_value = False
        node._prepare_autonomous_start()
    assert len(node.waypoints) == 2
    assert node.waypoints[0]['x'] == 5.0
    assert len(node.waypoint_labels) == 2


def test_load_waypoints_keeps_old_list_on_error(node):
    """A broken/missing waypoints file must not wipe the in-memory route."""
    node.waypoints_file = '/nonexistent/waypoints.yaml'
    node.waypoints = [{'x': 1.0, 'y': 1.0, 'yaw': 0.0}]
    node._load_waypoints()
    assert len(node.waypoints) == 1


def test_no_crop_scan_is_discarded(node):
    """An empty scan (no_crop_detected) must not count or publish a diagnosis,
    but still sets the dedup reference to avoid a stop loop."""
    node.state = 'STOPPED'
    node._pending_action = 'pipeline'
    node.plants_detected = 1
    node.odom_x = 0.5
    node.odom_y = 0.0
    node.has_scan_reference = False
    node.waypoints = [{'x': 1.0, 'y': 0.0, 'yaw': 0.0}]
    node.current_wp_idx = 0

    diag = Diagnosis()
    diag.disease_class = 'no_crop_detected'
    service_result = MagicMock()
    service_result.success = True
    service_result.result = diag
    future = MagicMock()
    future.done.return_value = True
    future.result.return_value = service_result
    node._pending_future = future

    with patch.object(node, '_resume_detector_async'), \
         patch.object(node, 'pub_diag') as mock_diag:
        node.tick()

    assert node.plants_detected == 0
    mock_diag.publish.assert_not_called()
    assert node.state == 'RESUME'
    assert node.has_scan_reference is True
    assert node.reference_x == 0.5
