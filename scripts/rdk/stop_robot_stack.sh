#!/usr/bin/env bash
# Safely stop the RDK robot stack and clear known ROS leftovers.
#
# Full-clean by default. When SENTRY_PRESERVE_WEB=1, keep web_remote_node,
# rosbridge, and the miniprogram_bridge gateway alive so frontend control
# planes can call this script safely.

set -u

START_TS="$(date +%s)"
ROS_DISTRO_NAME="${ROS_DISTRO:-humble}"
WS_DIR="${SENTRY_WS:-/home/sunrise/dev_ws}"
PRESERVE_WEB="${SENTRY_PRESERVE_WEB:-0}"
STOP_INT_GRACE_SEC="${SENTRY_STOP_INT_GRACE_SEC:-1.5}"
STOP_TERM_GRACE_SEC="${SENTRY_STOP_TERM_GRACE_SEC:-1.0}"
REFRESH_DAEMON="${SENTRY_REFRESH_DAEMON:-1}"
STOPPED_ANY=0

elapsed() {
  local now
  now="$(date +%s)"
  printf '+%03ds' "$((now - START_TS))"
}

log() {
  printf '[stop_robot_stack][%s] %s\n' "$(elapsed)" "$*"
}

source_ros() {
  set +u
  # shellcheck disable=SC1090
  source "/opt/ros/${ROS_DISTRO_NAME}/setup.bash"
  if [ -f "${WS_DIR}/install/setup.bash" ]; then
    # shellcheck disable=SC1090
    source "${WS_DIR}/install/setup.bash"
  fi
  set -u
  export FASTDDS_BUILTIN_TRANSPORTS="${FASTDDS_BUILTIN_TRANSPORTS:-UDPv4}"
  export RMW_FASTRTPS_USE_SHM="${RMW_FASTRTPS_USE_SHM:-0}"
}

ros_available() {
  command -v ros2 >/dev/null 2>&1
}

call_manual_mode() {
  if ! ros_available; then
    return 0
  fi
  log "Switching mission_control to MANUAL if service is available..."
  timeout 4 ros2 service call /set_auto_mode std_srvs/srv/SetBool "{data: false}" >/dev/null 2>&1 || true
}

publish_zero_velocity() {
  if ! ros_available; then
    return 0
  fi
  log "Publishing zero /cmd_vel for 1 second..."
  timeout 1.2 ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist \
    "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" \
    >/dev/null 2>&1 || true
}

terminate_pattern() {
  local signal="$1"
  local pattern="$2"
  local matches
  matches="$(pgrep -af "$pattern" || true)"
  if [ -n "$matches" ]; then
    STOPPED_ANY=1
    log "Sending ${signal} to: ${pattern}"
    pkill "-${signal}" -f "$pattern" >/dev/null 2>&1 || true
  fi
}

stop_patterns() {
  local signal="$1"
  terminate_pattern "$signal" "ros2 bag record"
  terminate_pattern "$signal" "ros2 topic pub"
  terminate_pattern "$signal" "ros2 launch sentry_bringup sentry_v2.launch.py"
  terminate_pattern "$signal" "mission_control_node"
  if [ "$PRESERVE_WEB" != "1" ]; then
    terminate_pattern "$signal" "web_remote_node"
    terminate_pattern "$signal" "rosbridge_websocket"
    # miniprogram_bridge is the always-on gateway layer (systemd
    # sentry-bridge.service) that triggers this script via /stack/*;
    # killing it would sever the frontend control plane mid-request.
    terminate_pattern "$signal" "miniprogram_bridge"
  fi
  terminate_pattern "$signal" "keyboard_control_node"
  terminate_pattern "$signal" "uart_bridge_node"
  terminate_pattern "$signal" "lora_bridge_node"
  terminate_pattern "$signal" "wheel_odom_node"
  terminate_pattern "$signal" "controller_server"
  terminate_pattern "$signal" "planner_server"
  terminate_pattern "$signal" "smoother_server"
  terminate_pattern "$signal" "behavior_server"
  terminate_pattern "$signal" "bt_navigator"
  terminate_pattern "$signal" "velocity_smoother"
  terminate_pattern "$signal" "lifecycle_manager"
  terminate_pattern "$signal" "ekf_node"
  terminate_pattern "$signal" "robot_state_publisher"
  terminate_pattern "$signal" "static_transform_publisher"
  terminate_pattern "$signal" "sentry_lidar"
  terminate_pattern "$signal" "imu_node"
  terminate_pattern "$signal" "imu_filter_madgwick_node"
  terminate_pattern "$signal" "servo_driver_node"
  terminate_pattern "$signal" "mipi_camera_node"
  terminate_pattern "$signal" "hikrobot_camera_node"
  terminate_pattern "$signal" "image_republisher"
  terminate_pattern "$signal" "vision_diagnosis_node"
  terminate_pattern "$signal" "plant_detector_node"
  terminate_pattern "$signal" "vision_pipeline_node"
  terminate_pattern "$signal" "fusion_node"
  terminate_pattern "$signal" "forecast_node"
  terminate_pattern "$signal" "advisory_node"
  terminate_pattern "$signal" "data_logger_node"
}

sleep_if_stopped() {
  local seconds="$1"
  if [ "$STOPPED_ANY" = "1" ]; then
    sleep "$seconds"
  fi
}

report_leftovers() {
  log "Remaining project ROS processes, if any:"
  pgrep -af "/home/sunrise/dev_ws/install|sentry_v2.launch.py|ros2 bag record|ros2 topic pub" || true
}

main() {
  log "Host IPs: $(hostname -I 2>/dev/null || true)"
  if [ "$PRESERVE_WEB" = "1" ]; then
    log "Preserving web_remote_node and rosbridge control plane."
  fi
  source_ros

  call_manual_mode
  publish_zero_velocity

  log "Stopping launch, bag, and known ROS nodes..."
  STOPPED_ANY=0
  stop_patterns INT
  sleep_if_stopped "$STOP_INT_GRACE_SEC"

  publish_zero_velocity
  STOPPED_ANY=0
  stop_patterns TERM
  sleep_if_stopped "$STOP_TERM_GRACE_SEC"

  STOPPED_ANY=0
  stop_patterns KILL

  if ros_available && [ "$REFRESH_DAEMON" = "1" ]; then
    log "Refreshing ROS daemon graph cache..."
    ros2 daemon stop >/dev/null 2>&1 || true
    ros2 daemon start >/dev/null 2>&1 || true
  fi

  report_leftovers
  log "Stop/cleanup complete."
}

main "$@"
