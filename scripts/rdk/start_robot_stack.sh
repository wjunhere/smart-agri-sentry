#!/usr/bin/env bash
# Full-clean start for the RDK robot stack.
#
# Runs stop_robot_stack.sh first, starts the formal sentry_v2 launch, and
# verifies the runtime state used by field demonstrations.

set -u

START_TS="$(date +%s)"
ROS_DISTRO_NAME="${ROS_DISTRO:-humble}"
WS_DIR="${SENTRY_WS:-/home/sunrise/dev_ws}"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
LOG_DIR="${SENTRY_LOG_DIR:-/tmp}"
LAUNCH_LOG="${LOG_DIR}/sentry_v2_start_robot_stack.log"
PRESERVE_WEB="${SENTRY_PRESERVE_WEB:-0}"
NODE_WAIT_TIMEOUT="${SENTRY_NODE_WAIT_TIMEOUT:-35}"
PARAM_TIMEOUT="${SENTRY_PARAM_TIMEOUT:-4}"
TOPIC_TIMEOUT="${SENTRY_TOPIC_TIMEOUT:-3}"
CHASSIS_TIMEOUT="${SENTRY_CHASSIS_TIMEOUT:-8}"
REQUIRE_SCAN_SAMPLE="${SENTRY_REQUIRE_SCAN_SAMPLE:-0}"
CHECK_STABLE_PARAMS="${SENTRY_CHECK_STABLE_PARAMS:-0}"

CROP_TYPE="${CROP_TYPE:-tomato}"
ENABLE_VISION="${ENABLE_VISION:-true}"
ENABLE_ADVISORY="${ENABLE_ADVISORY:-true}"
ENABLE_WEB="${ENABLE_WEB:-true}"
CAMERA_BACKEND="${CAMERA_BACKEND:-mipi}"
if [ "$PRESERVE_WEB" = "1" ]; then
  ENABLE_WEB="false"
fi

elapsed() {
  local now
  now="$(date +%s)"
  printf '+%03ds' "$((now - START_TS))"
}

log() {
  printf '[start_robot_stack][%s] %s\n' "$(elapsed)" "$*"
}

warn() {
  printf '[start_robot_stack][%s][WARN] %s\n' "$(elapsed)" "$*" >&2
}

fail() {
  printf '[start_robot_stack][%s][ERROR] %s\n' "$(elapsed)" "$*" >&2
  exit 1
}

control_web_expected() {
  [ "$ENABLE_WEB" = "true" ] || [ "$PRESERVE_WEB" = "1" ]
}

source_ros() {
  set +u
  # shellcheck disable=SC1090
  source "/opt/ros/${ROS_DISTRO_NAME}/setup.bash"
  [ -f "${WS_DIR}/install/setup.bash" ] || fail "Missing ${WS_DIR}/install/setup.bash; build/source the workspace first."
  # shellcheck disable=SC1090
  source "${WS_DIR}/install/setup.bash"
  set -u
  export FASTDDS_BUILTIN_TRANSPORTS="${FASTDDS_BUILTIN_TRANSPORTS:-UDPv4}"
  export RMW_FASTRTPS_USE_SHM="${RMW_FASTRTPS_USE_SHM:-0}"
}

wait_for_nodes() {
  local timeout_sec="$1"
  shift
  local elapsed_sec=0
  local node node_list
  local -a pending=("$@")
  local -a next=()

  while [ "$elapsed_sec" -lt "$timeout_sec" ]; do
    node_list="$(ros2 node list 2>/dev/null || true)"
    next=()
    for node in "${pending[@]}"; do
      if printf '%s\n' "$node_list" | grep -qx "$node"; then
        log "Node ready: ${node}"
      else
        next+=("$node")
      fi
    done
    pending=("${next[@]}")
    [ "${#pending[@]}" -eq 0 ] && return 0
    sleep 1
    elapsed_sec=$((elapsed_sec + 1))
  done

  fail "Timed out waiting for nodes: ${pending[*]}. Check ${LAUNCH_LOG}"
}

check_param_contains() {
  local node="$1"
  local param="$2"
  local expected="$3"
  local value
  value="$(timeout "$PARAM_TIMEOUT" ros2 param get "$node" "$param" 2>&1 || true)"
  printf '[start_robot_stack][%s] %s %s -> %s\n' "$(elapsed)" "$node" "$param" "$value"
  if ! printf '%s' "$value" | grep -q "$expected"; then
    fail "Unexpected parameter ${node}.${param}; expected output containing '${expected}'"
  fi
}

check_topic_once_quiet() {
  local topic="$1"
  local timeout_sec="${2:-$TOPIC_TIMEOUT}"
  local required="${3:-required}"
  local safe_name
  local output_file
  safe_name="$(printf '%s' "$topic" | tr '/ ' '__')"
  output_file="${LOG_DIR}/start_robot_stack${safe_name}.sample"

  log "Checking ${topic} once..."
  timeout "$timeout_sec" ros2 topic echo "$topic" --once >"$output_file" 2>&1 || true
  if [ -s "$output_file" ] && grep -q "header:" "$output_file"; then
    log "${topic} sample received."
    return 0
  fi

  if [ "$required" = "required" ]; then
    sed -n '1,8p' "$output_file" 2>/dev/null || true
    fail "No sample from ${topic} within ${timeout_sec}s"
  fi
  warn "No sample from ${topic} within ${timeout_sec}s"
  return 0
}

check_chassis_status() {
  local status
  log "Checking /sentry/chassis/status once..."
  status="$(timeout "$CHASSIS_TIMEOUT" ros2 topic echo /sentry/chassis/status --once 2>&1)" || fail "No /sentry/chassis/status sample within ${CHASSIS_TIMEOUT}s"
  printf '%s\n' "$status" | grep -E 'left_speed|right_speed|battery_voltage|comm_timeout' || true
  if ! printf '%s' "$status" | grep -q "comm_timeout: false"; then
    fail "Chassis communication is not healthy"
  fi
  if printf '%s' "$status" | grep -qi "nan"; then
    fail "Chassis status contains NaN values"
  fi
}

check_obstacle_info() {
  local output_file="${LOG_DIR}/start_robot_stack_lidar_obstacle_info.sample"
  log "Checking /lidar/obstacle_info once..."
  if timeout "$TOPIC_TIMEOUT" ros2 topic echo /lidar/obstacle_info --once >"$output_file" 2>&1; then
    grep -E 'front_min_distance|obstacle_detected|danger_threshold|front_point_count' "$output_file" || true
    return 0
  fi
  warn "No sample from /lidar/obstacle_info within ${TOPIC_TIMEOUT}s"
}

check_no_duplicate_nodes() {
  local duplicates
  duplicates="$(ros2 node list 2>/dev/null | sort | uniq -d | grep -Ev '^/transform_listener_impl_' || true)"
  if [ -z "$duplicates" ]; then
    return 0
  fi

  warn "Duplicate ROS node names detected; refreshing graph and retrying..."
  ros2 daemon stop >/dev/null 2>&1 || true
  ros2 daemon start >/dev/null 2>&1 || true
  sleep 3
  duplicates="$(ros2 node list 2>/dev/null | sort | uniq -d | grep -Ev '^/transform_listener_impl_' || true)"
  if [ -z "$duplicates" ]; then
    return 0
  fi

  local remaining=""
  while IFS= read -r node; do
    [ -n "$node" ] || continue
    if [ "$node" = "/mission_control_node" ]; then
      local mission_count
      mission_count="$(pgrep -fc '/sentry_mission/mission_control_node' || true)"
      if [ "${mission_count:-0}" -le 1 ]; then
        warn "Ignoring stale ROS graph duplicate for /mission_control_node; actual process count=${mission_count:-0}"
        continue
      fi
    fi
    remaining="${remaining}${node}
"
  done <<EOF
$duplicates
EOF

  if [ -n "$remaining" ]; then
    printf '%b' "$remaining"
    fail "Duplicate ROS node names detected after full-clean start"
  fi
}

check_cmd_vel_route() {
  local info
  info="$(ros2 topic info /cmd_vel -v 2>&1 || true)"
  printf '%s\n' "$info" | grep -E 'Publisher count|Subscription count|Node name' || true
  if control_web_expected; then
    if ! printf '%s' "$info" | grep -q "Publisher count: 2"; then
      fail "/cmd_vel should have mission_control_node and web_remote_node publishers when web control is active"
    fi
    if ! printf '%s' "$info" | grep -q "web_remote_node"; then
      fail "/cmd_vel publisher should include web_remote_node when web control is active"
    fi
  else
    if ! printf '%s' "$info" | grep -q "Publisher count: 1"; then
      fail "/cmd_vel should have exactly one publisher when web is disabled"
    fi
  fi
  if ! printf '%s' "$info" | grep -q "mission_control_node"; then
    fail "/cmd_vel publisher should include mission_control_node"
  fi
  if ! printf '%s' "$info" | grep -q "uart_bridge_node"; then
    fail "/cmd_vel subscriber should include uart_bridge_node"
  fi
}

ensure_preserved_rosbridge() {
  [ "$PRESERVE_WEB" = "1" ] || return 0
  if ros2 node list 2>/dev/null | grep -qx "/rosbridge_websocket"; then
    log "Preserved rosbridge already running."
    return 0
  fi
  log "Starting rosbridge control plane for preserved frontend; log: ${LOG_DIR}/rosbridge_websocket_start_robot_stack.log"
  setsid ros2 run rosbridge_server rosbridge_websocket \
    >"${LOG_DIR}/rosbridge_websocket_start_robot_stack.log" 2>&1 &
  echo "$!" >/tmp/rosbridge_websocket.pid
}

check_web_frontend() {
  control_web_expected || return 0
  command -v curl >/dev/null 2>&1 || fail "curl is required to check the web frontend"
  local status
  log "Checking web frontend status endpoint..."
  status="$(curl -s --max-time 3 http://127.0.0.1:5000/status || true)"
  printf '[start_robot_stack][%s] web status -> %s\n' "$(elapsed)" "$status"
  if ! printf '%s' "$status" | grep -q '"service_ready":true'; then
    fail "web_remote_node cannot reach /set_auto_mode service"
  fi
  curl -fsS --max-time 3 http://127.0.0.1:5000/ >/dev/null || fail "web frontend index page is not reachable"
}

main() {
  log "Host IPs: $(hostname -I 2>/dev/null || true)"

  if [ ! -f "${SCRIPT_DIR}/stop_robot_stack.sh" ]; then
    fail "Missing ${SCRIPT_DIR}/stop_robot_stack.sh"
  fi

  log "Running full cleanup before start..."
  bash "${SCRIPT_DIR}/stop_robot_stack.sh" || fail "Full cleanup failed"

  source_ros
  mkdir -p "$LOG_DIR"

  log "Starting sentry_v2.launch.py; log: ${LAUNCH_LOG}"
  setsid ros2 launch sentry_bringup sentry_v2.launch.py \
    crop_type:="${CROP_TYPE}" \
    enable_vision:="${ENABLE_VISION}" \
    camera_backend:="${CAMERA_BACKEND}" \
    enable_advisory:="${ENABLE_ADVISORY}" \
    enable_web:="${ENABLE_WEB}" \
    >"${LAUNCH_LOG}" 2>&1 &
  echo "$!" >/tmp/sentry_v2.launch.pid

  local -a nodes=(
    /mission_control_node
    /uart_bridge_node
    /wheel_odom_node
    /controller_server
    /planner_server
    /bt_navigator
    /velocity_smoother
    /sentry_lidar
  )
  if control_web_expected; then
    ensure_preserved_rosbridge
    nodes+=(/web_remote_node /rosbridge_websocket)
  fi
  if [ "$ENABLE_VISION" = "true" ]; then
    if [ "$CAMERA_BACKEND" = "hikrobot" ]; then
      nodes+=(/hikrobot_camera_node)
    else
      nodes+=(/mipi_camera_node)
    fi
    nodes+=(/plant_detector_node /vision_pipeline_node)
  fi
  if [ "$ENABLE_ADVISORY" = "true" ]; then
    nodes+=(/fusion_node)
  fi
  wait_for_nodes "$NODE_WAIT_TIMEOUT" "${nodes[@]}"

  log "Checking duplicate ROS node names..."
  check_no_duplicate_nodes

  if [ "$CHECK_STABLE_PARAMS" = "1" ]; then
    log "Checking stable runtime parameters..."
    check_param_contains /uart_bridge_node right_speed_scale "1.0"
    check_param_contains /uart_bridge_node left_speed_scale "1.0"
    check_param_contains /wheel_odom_node pulses_per_meter "11552"
    check_param_contains /wheel_odom_node wheel_base "0.23"
    check_param_contains /mission_control_node obstacle_resume_delay_sec "0.5"
    check_param_contains /mission_control_node avoidance_retrigger_suppression_sec "2.5"
    check_param_contains /mission_control_node avoidance_drive_distance "0.55"
  else
    log "Skipping stable parameter checks; set SENTRY_CHECK_STABLE_PARAMS=1 for full validation."
  fi

  log "Checking /cmd_vel ownership..."
  check_cmd_vel_route

  log "Checking web frontend control path..."
  check_web_frontend

  log "Checking chassis, scan, and obstacle topics..."
  check_chassis_status
  if [ "$REQUIRE_SCAN_SAMPLE" = "1" ]; then
    check_topic_once_quiet /scan "$TOPIC_TIMEOUT" required
  else
    check_topic_once_quiet /scan "$TOPIC_TIMEOUT" optional
  fi
  check_obstacle_info

  log "Robot stack started and checked. Open http://<rdk-ip>:5000/ and use the cruise buttons."
}

main "$@"
