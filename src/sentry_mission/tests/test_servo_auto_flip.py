"""Tests for servo auto-flip on row switch."""

import math
import pytest
import rclpy
from unittest.mock import patch

from sentry_mission.mission_control_node import MissionControlNode


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


def _serpentine():
    """Layout-(b) two-row serpentine, robot starts at origin."""
    return [
        {'x': 2.5, 'y': 0.0, 'yaw': 1.5708},
        {'x': 2.5, 'y': 1.0, 'yaw': 3.1416},
        {'x': 0.0, 'y': 1.0, 'yaw': 3.1416},
    ]


def _arm(node, idx):
    node.enable_servo_auto_flip = True
    node.waypoints = _serpentine()
    node.min_row_segment_length = node._derive_min_segment_length()
    node.current_wp_idx = idx


# ---- Task 1: params, init members, derivation ----

def test_default_params_and_init_members(node):
    assert node.enable_servo_auto_flip is False
    assert node.servo_yaw_right == 0
    assert node.servo_yaw_left == 180
    assert node.servo_pitch_hold == 0
    assert node.flip_heading_threshold == pytest.approx(2.09)
    assert node.servo_flip_cooldown_sec == pytest.approx(8.0)
    assert node.servo_flip_cooldown_distance == pytest.approx(0.8)
    assert node._servo_side == 'right'
    assert node._servo_flip_time is None
    assert node._servo_flip_position is None
    assert node._mission_start_pose == (0.0, 0.0)


def test_auto_derive_min_segment_length(node):
    node.waypoints = _serpentine()
    # segments: 1.0 (corner), 2.5 (row) -> (1.0 + 2.5) / 2
    assert node._derive_min_segment_length() == pytest.approx(1.75)


def test_auto_derive_disabled_with_fewer_than_two_waypoints(node):
    node.waypoints = [{'x': 1.0, 'y': 0.0, 'yaw': 0.0}]
    assert node._derive_min_segment_length() is None


def test_manual_min_segment_length_overrides_auto(node):
    node._min_seg_len_manual = 1.2
    node.waypoints = _serpentine()
    assert node._derive_min_segment_length() == pytest.approx(1.2)


# ---- Task 2: geometry flip ----

def test_flip_at_first_row_end(node):
    """Default waypoints: arrival at wp0 (first row end) flips right->left."""
    _arm(node, 1)
    node._mission_start_pose = (0.0, 0.0)
    with patch.object(node.pub_servo_cmd, 'publish') as mock_pub:
        node._maybe_flip_servo(now=100.0)
    assert mock_pub.called
    msg = mock_pub.call_args[0][0]
    assert msg.yaw == 180
    assert node._servo_side == 'left'
    assert node._servo_flip_time == 100.0
    assert node.has_scan_reference is True


def test_no_flip_at_corner_end(node):
    """Arrival at wp1 (corner end, short segment) must not flip."""
    _arm(node, 2)
    with patch.object(node.pub_servo_cmd, 'publish') as mock_pub:
        node._maybe_flip_servo(now=100.0)
    mock_pub.assert_not_called()
    assert node._servo_side == 'right'


def test_no_flip_at_final_waypoint(node):
    """Arrival at last waypoint: no following segment, no flip."""
    _arm(node, 3)
    with patch.object(node.pub_servo_cmd, 'publish') as mock_pub:
        node._maybe_flip_servo(now=100.0)
    mock_pub.assert_not_called()


def test_toggle_back_on_second_row_switch(node):
    """Two consecutive row switches toggle side back to right."""
    node.enable_servo_auto_flip = True
    node.waypoints = [
        {'x': 2.5, 'y': 0.0, 'yaw': 0.0},
        {'x': 2.5, 'y': 1.0, 'yaw': 0.0},
        {'x': 0.0, 'y': 1.0, 'yaw': 0.0},
        {'x': 0.0, 'y': 2.0, 'yaw': 0.0},
        {'x': 2.5, 'y': 2.0, 'yaw': 0.0},
    ]
    node.min_row_segment_length = node._derive_min_segment_length()
    node._mission_start_pose = (0.0, 0.0)
    with patch.object(node.pub_servo_cmd, 'publish') as mock_pub:
        node.current_wp_idx = 1
        node._maybe_flip_servo(now=1.0)
        node.current_wp_idx = 3
        node._maybe_flip_servo(now=2.0)
    assert mock_pub.call_count == 2
    assert mock_pub.call_args_list[0][0][0].yaw == 180
    assert mock_pub.call_args_list[1][0][0].yaw == 0
    assert node._servo_side == 'right'


def test_no_flip_on_l_turn(node):
    """L-shaped path (delta ~= 90 deg) is not a row switch."""
    node.enable_servo_auto_flip = True
    node.waypoints = _serpentine()
    node.min_row_segment_length = node._derive_min_segment_length()
    # Start pose makes first segment head +y instead of +x
    node._mission_start_pose = (2.5, -2.5)
    node.current_wp_idx = 1
    with patch.object(node.pub_servo_cmd, 'publish') as mock_pub:
        node._maybe_flip_servo(now=1.0)
    mock_pub.assert_not_called()


def test_no_flip_on_straight_midpoint(node):
    """Collinear waypoint mid-row (delta ~= 0) is not a row switch."""
    node.enable_servo_auto_flip = True
    node.waypoints = [
        {'x': 2.5, 'y': 0.0, 'yaw': 0.0},
        {'x': 5.0, 'y': 0.0, 'yaw': 0.0},
        {'x': 5.0, 'y': 1.0, 'yaw': 0.0},
        {'x': 0.0, 'y': 1.0, 'yaw': 0.0},
    ]
    node.min_row_segment_length = 1.0  # manual: rows 2.5/5.0, corner 1.0
    node._mission_start_pose = (0.0, 0.0)
    node.current_wp_idx = 1
    with patch.object(node.pub_servo_cmd, 'publish') as mock_pub:
        node._maybe_flip_servo(now=1.0)
    mock_pub.assert_not_called()


def test_no_flip_when_disabled(node):
    _arm(node, 1)
    node.enable_servo_auto_flip = False
    node._mission_start_pose = (0.0, 0.0)
    with patch.object(node.pub_servo_cmd, 'publish') as mock_pub:
        node._maybe_flip_servo(now=1.0)
    mock_pub.assert_not_called()


def test_no_flip_when_segment_length_underivable(node):
    _arm(node, 1)
    node.min_row_segment_length = None
    node._mission_start_pose = (0.0, 0.0)
    with patch.object(node.pub_servo_cmd, 'publish') as mock_pub:
        node._maybe_flip_servo(now=1.0)
    mock_pub.assert_not_called()


def test_mission_start_pose_defaults_to_odom_origin(node):
    """Mission start pose is the odom origin (EKF reset on AUTO start)."""
    assert node._mission_start_pose == (0.0, 0.0)


def test_mission_rerun_still_flips(node):
    """After mission rerun (idx reset to 0), wp0 arrival flips again."""
    _arm(node, 1)
    node._mission_start_pose = (0.0, 0.0)
    node._servo_side = 'left'  # side from previous run
    with patch.object(node.pub_servo_cmd, 'publish') as mock_pub:
        node._maybe_flip_servo(now=1.0)
    assert mock_pub.called
    assert mock_pub.call_args[0][0].yaw == 0  # toggled back to right


def test_tick_calls_maybe_flip_servo_on_waypoint_reached(node):
    """Waypoint completion in tick() triggers the flip check."""
    from nav2_simple_commander.robot_navigator import TaskResult
    node.state = 'PATROL'
    node._nav2_ready = True
    node.current_wp_idx = 0
    node.sending_goal = True
    node.last_goal_sent_time = 0.0
    node.waypoints = _serpentine()
    with patch.object(node.navigator, 'isTaskComplete', return_value=True), \
         patch.object(node.navigator, 'getResult',
                      return_value=TaskResult.SUCCEEDED), \
         patch.object(node, '_maybe_flip_servo') as mock_flip:
        node.tick()
    mock_flip.assert_called_once()
    assert node.current_wp_idx == 1


# ---- Task 3: cooldown ----

def test_cooldown_suppresses_scan_trigger(node):
    """Within cooldown window (time AND distance), scan trigger is blocked."""
    node._servo_flip_time = node.get_clock().now().nanoseconds / 1e9
    node._servo_flip_position = (0.0, 0.0)
    node.odom_x = 0.1
    node.odom_y = 0.0
    node.has_scan_reference = True
    node.reference_x = 0.0
    node.reference_y = 0.0
    assert node._should_trigger_scan() is False


def test_cooldown_expires_after_distance(node):
    """Driving past the cooldown distance ends the window."""
    node._servo_flip_time = node.get_clock().now().nanoseconds / 1e9
    node._servo_flip_position = (0.0, 0.0)
    node.odom_x = 1.0  # > servo_flip_cooldown_distance (0.8)
    node.odom_y = 0.0
    node.has_scan_reference = True
    node.reference_x = 0.0
    node.reference_y = 0.0
    assert node._should_trigger_scan() is True
    assert node._servo_flip_time is None


def test_cooldown_expires_after_time(node):
    """Cooldown ends after servo_flip_cooldown_sec even without moving."""
    node._servo_flip_time = (
        node.get_clock().now().nanoseconds / 1e9
        - node.servo_flip_cooldown_sec - 1.0)
    node._servo_flip_position = (0.0, 0.0)
    node.odom_x = 0.0
    node.odom_y = 0.0
    node.has_scan_reference = True
    node.reference_x = 0.0
    node.reference_y = 0.0
    # min_resume_distance not met (0 < 0.5), so the distance check still
    # returns False; what we verify is the cooldown window was cleared
    assert node._should_trigger_scan() is False
    assert node._servo_flip_time is None


# ---- Task 4: launch wiring + servo config ----

def test_launch_wires_servo_auto_flip_params():
    import pathlib
    launch = pathlib.Path(__file__).parents[2].joinpath(
        'sentry_bringup', 'launch', 'sentry_v2.launch.py')
    text = launch.resolve().read_text(encoding='utf-8')
    for key in ('enable_servo_auto_flip', 'servo_yaw_right', 'servo_yaw_left',
                'servo_pitch_hold', 'flip_heading_threshold',
                'min_row_segment_length', 'servo_flip_cooldown_sec',
                'servo_flip_cooldown_distance'):
        assert key in text, f'{key} not wired in sentry_v2.launch.py'


def test_servo_config_initial_angle_is_physical_right():
    import pathlib
    import yaml
    cfg = pathlib.Path(__file__).parents[2].joinpath(
        'sentry_servo', 'config', 'servo_config.yaml')
    data = yaml.safe_load(cfg.resolve().read_text(encoding='utf-8'))
    # 180 deg = physical right on the current servo mount
    assert data['servos']['yaw']['initial_angle'] == 180


# ---- Servo restore-to-home after patrol ----

def test_restore_servo_home_after_left_flip(node):
    """Patrol ending with servo on the left side restores yaw to home."""
    node.enable_servo_auto_flip = True
    node._servo_side = 'left'
    node._servo_flip_time = 100.0
    node._servo_flip_position = (1.0, 2.0)
    with patch.object(node.pub_servo_cmd, 'publish') as mock_pub:
        node._restore_servo_home()
    mock_pub.assert_called_once()
    msg = mock_pub.call_args[0][0]
    assert msg.yaw == node.servo_yaw_right
    assert msg.pitch == int(node.servo_pitch_hold)
    assert node._servo_side == 'right'
    assert node._servo_flip_time is None
    assert node._servo_flip_position is None


def test_restore_servo_home_noop_when_already_home(node):
    """Servo already on the right (home) side: no command sent."""
    node.enable_servo_auto_flip = True
    node._servo_side = 'right'
    with patch.object(node.pub_servo_cmd, 'publish') as mock_pub:
        node._restore_servo_home()
    mock_pub.assert_not_called()


def test_restore_servo_home_noop_when_auto_flip_disabled(node):
    """Auto-flip off means the node never moved the servo: don't touch it."""
    node.enable_servo_auto_flip = False
    node._servo_side = 'left'
    with patch.object(node.pub_servo_cmd, 'publish') as mock_pub:
        node._restore_servo_home()
    mock_pub.assert_not_called()


def test_manual_stop_restores_servo_home(node):
    """Manual cruise stop (set_auto_mode False) restores servo like patrol end."""
    from std_srvs.srv import SetBool
    node.state = 'PATROL'
    node.enable_servo_auto_flip = True
    node._servo_side = 'left'
    with patch.object(node, '_cancel_nav2_task_async'), \
         patch.object(node, 'pub_cmd'), \
         patch.object(node.pub_servo_cmd, 'publish') as mock_pub:
        request = SetBool.Request()
        request.data = False
        response = node.set_auto_mode_cb(request, SetBool.Response())
    assert response.success is True
    assert node.state == 'MANUAL'
    mock_pub.assert_called_once()
    assert mock_pub.call_args[0][0].yaw == node.servo_yaw_right
    assert node._servo_side == 'right'


# ---- Configurable servo start side ----

def test_restore_uses_start_side_left(node):
    """With start side 'left', restore targets servo_yaw_left."""
    node.enable_servo_auto_flip = True
    node.servo_start_side = 'left'
    node._servo_side = 'right'
    with patch.object(node.pub_servo_cmd, 'publish') as mock_pub:
        node._restore_servo_home()
    msg = mock_pub.call_args[0][0]
    assert msg.yaw == node.servo_yaw_left
    assert node._servo_side == 'left'


def test_runtime_start_side_change_commands_servo(node):
    """Setting servo_start_side at runtime re-homes the servo immediately."""
    from rclpy.parameter import Parameter as RosParameter
    node.enable_servo_auto_flip = True
    param = RosParameter('servo_start_side', value='left')
    with patch.object(node.pub_servo_cmd, 'publish') as mock_pub:
        result = node._on_param_change([param])
    assert result.successful is True
    assert node.servo_start_side == 'left'
    assert node._servo_side == 'left'
    assert mock_pub.call_args[0][0].yaw == node.servo_yaw_left


def test_runtime_start_side_rejects_bad_value(node):
    from rclpy.parameter import Parameter as RosParameter
    param = RosParameter('servo_start_side', value='up')
    result = node._on_param_change([param])
    assert result.successful is False
    assert node.servo_start_side == 'right'
