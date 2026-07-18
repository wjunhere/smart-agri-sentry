"""Off-board regression tests for autonomous cruise startup.

These tests avoid importing ROS2 so they can run on a normal developer
machine while still locking the logic that gates patrol startup.
"""

from pathlib import Path
import sys


MISSION_SRC = Path(__file__).resolve().parents[1]
REPO_ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(MISSION_SRC))


def test_patrol_goal_is_sent_after_nav2_becomes_ready():
    from sentry_mission.autonomous_cruise import should_send_patrol_goal

    assert should_send_patrol_goal(
        state="PATROL",
        nav2_ready=True,
        sending_goal=False,
        current_wp_idx=0,
        waypoint_count=2,
    )

    assert not should_send_patrol_goal("MANUAL", True, False, 0, 2)
    assert not should_send_patrol_goal("PATROL", False, False, 0, 2)
    assert not should_send_patrol_goal("PATROL", True, True, 0, 2)
    assert not should_send_patrol_goal("PATROL", True, False, 2, 2)


def test_mission_state_drives_frontend_mode():
    from sentry_mission.autonomous_cruise import mission_state_to_mode

    assert mission_state_to_mode("MANUAL") == "MANUAL"
    for state in ["PATROL", "STOPPED", "SCANNING", "ANALYZING", "ACTION", "RESUME"]:
        assert mission_state_to_mode(state) == "AUTO"


def test_frontend_defaults_manual_and_syncs_mode_from_mission_status():
    ros_js = REPO_ROOT / "src" / "sentry_mission" / "static_v2" / "ros.js"
    text = ros_js.read_text(encoding="utf-8")

    assert "mode: 'MANUAL'" in text
    assert "store.mode = missionStateToMode(msg.state);" in text


def test_launch_uses_explicit_mapless_navigation_stack():
    launch_file = (
        REPO_ROOT
        / "src"
        / "sentry_bringup"
        / "launch"
        / "sentry_v2.launch.py"
    )
    text = launch_file.read_text(encoding="utf-8")

    assert "navigation_launch.py" not in text
    assert "bringup_launch.py" not in text
    assert "'map': ''" not in text
    assert "DeclareLaunchArgument('cruise_speed'" in text
    assert "executable='controller_server'" in text
    assert "executable='planner_server'" in text
    assert "executable='bt_navigator'" in text
    assert "executable='velocity_smoother'" in text


def test_frontend_exposes_camera_start_and_cruise_speed_controls():
    top_bar = REPO_ROOT / 'src' / 'sentry_mission' / 'static_v2' / 'components' / 'top-bar.js'
    cruise_panel = REPO_ROOT / 'src' / 'sentry_mission' / 'static_v2' / 'components' / 'cruise-panel.js'
    ros_js = REPO_ROOT / 'src' / 'sentry_mission' / 'static_v2' / 'ros.js'

    assert '开启摄像头' in top_bar.read_text(encoding='utf-8')
    assert 'callVisionStart' in top_bar.read_text(encoding='utf-8')
    assert '拍摄' in top_bar.read_text(encoding='utf-8')
    assert 'callCaptureImage' in top_bar.read_text(encoding='utf-8')
    assert '巡航速度' in cruise_panel.read_text(encoding='utf-8')
    assert 'callSetCruiseSpeed' in cruise_panel.read_text(encoding='utf-8')
    assert "'/vision/start'" in ros_js.read_text(encoding='utf-8')
    assert "'/cruise-speed'" in ros_js.read_text(encoding='utf-8')
    assert "'/camera/capture'" in ros_js.read_text(encoding='utf-8')
    assert ':disabled="store.cruiseSpeedBusy"' in cruise_panel.read_text(encoding='utf-8')
    assert 'let cruiseSpeedLoaded = false;' in ros_js.read_text(encoding='utf-8')
    assert 'if (!cruiseSpeedLoaded && Number.isFinite(Number(data.cruise_speed)))' in ros_js.read_text(encoding='utf-8')


def test_start_script_loads_saved_cruise_speed_before_launch():
    start_script = REPO_ROOT / 'scripts' / 'rdk' / 'start_robot_stack.sh'
    text = start_script.read_text(encoding='utf-8')

    assert 'MISSION_PARAMS_FILE' in text
    assert 'cruise_speed:="${CRUISE_SPEED}"' in text


def test_bringup_declares_autonomous_cruise_runtime_dependencies():
    package_xml = REPO_ROOT / "src" / "sentry_bringup" / "package.xml"
    text = package_xml.read_text(encoding="utf-8")

    for dependency in [
        "nav2_bringup",
        "robot_localization",
        "robot_state_publisher",
        "tf2_ros",
        "sentry_lidar",
        "sentry_mission",
        "sentry_sensors",
    ]:
        assert f"<depend>{dependency}</depend>" in text


def test_mapless_nav2_costmaps_do_not_require_static_map():
    nav2_config = (
        REPO_ROOT
        / "src"
        / "sentry_mission"
        / "config"
        / "nav2_no_map.yaml"
    )
    text = nav2_config.read_text(encoding="utf-8")

    assert "static_layer" not in text
    assert "map_server:" not in text
    assert 'map: \'\'' not in text
    assert "global_costmap:\n  global_costmap:\n    ros__parameters:" in text
    assert "local_costmap:\n  local_costmap:\n    ros__parameters:" in text
    assert 'plugin: "nav2_waypoint_follower::WaitAtWaypoint"' in text


def test_dpad_only_publishes_cmd_vel_in_manual_mode():
    dpad = (
        REPO_ROOT
        / "src"
        / "sentry_mission"
        / "static_v2"
        / "components"
        / "dpad.js"
    )
    text = dpad.read_text(encoding="utf-8")

    assert "store.mode !== 'MANUAL'" in text
