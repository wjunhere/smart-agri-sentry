"""Regression checks for the calibrated in-place turn parameters."""

from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[3]
LAUNCH_FILE = REPO_ROOT / 'src' / 'sentry_bringup' / 'launch' / 'sentry_v2.launch.py'
NAV2_FILE = REPO_ROOT / 'src' / 'sentry_mission' / 'config' / 'nav2_no_map.yaml'


def test_turn_calibration_is_applied_consistently():
    """Command conversion and wheel odometry use the measured 0.244 m base."""
    launch_text = LAUNCH_FILE.read_text(encoding='utf-8')
    uart_text = (REPO_ROOT / 'src' / 'sentry_sensors' / 'sentry_sensors' /
                 'uart_bridge_node.py').read_text(encoding='utf-8')
    wheel_odom_text = (REPO_ROOT / 'src' / 'sentry_mission' / 'sentry_mission' /
                       'wheel_odom_node.py').read_text(encoding='utf-8')
    mission_text = (REPO_ROOT / 'src' / 'sentry_mission' / 'sentry_mission' /
                    'mission_control_node.py').read_text(encoding='utf-8')
    assert launch_text.count("'wheel_base': 0.244") == 2
    assert "declare_parameter('wheel_base', 0.244)" in uart_text
    assert "declare_parameter('wheel_base', 0.244)" in wheel_odom_text
    assert "declare_parameter('wheel_base', 0.244)" in mission_text


def test_heading_goal_tolerance_matches_turn_calibration():
    """Waypoint turns must settle within the requested 0.04 rad tolerance."""
    nav_text = NAV2_FILE.read_text(encoding='utf-8')
    assert 'yaw_goal_tolerance: 0.04' in nav_text