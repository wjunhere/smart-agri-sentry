#!/usr/bin/env bash

cd "$HOME/dev_ws" || exit 1
source /opt/ros/humble/setup.bash
source install/setup.bash

LOG=/tmp/sentry_auto_cruise_test.log
rm -f "$LOG"

setsid ros2 launch sentry_bringup sentry_v2.launch.py >"$LOG" 2>&1 &
pid=$!
echo "LAUNCH_PID=$pid"

cleanup() {
  echo "===FORCE_MANUAL==="
  timeout 6s ros2 service call /set_auto_mode std_srvs/srv/SetBool "{data: false}" || true
  timeout 6s ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
    "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" || true
  kill -INT -- "-$pid" 2>/dev/null || true
  sleep 5
  if kill -0 "$pid" 2>/dev/null; then
    kill -- "-$pid" 2>/dev/null || true
  fi
}
trap cleanup EXIT

sleep 22

echo "===PRE_STATUS==="
timeout 5s ros2 topic echo /mission/status --once || true

echo "===PRE_CHASSIS==="
chassis="$(timeout 5s ros2 topic echo /sentry/chassis/status --once || true)"
echo "$chassis"
if echo "$chassis" | grep -q "comm_timeout: true"; then
  echo "ABORT: chassis comm_timeout is true"
  exit 2
fi

echo "===PRE_SCAN_HZ==="
timeout 5s ros2 topic hz /scan || true

echo "===SET_AUTO_TRUE==="
timeout 8s ros2 service call /set_auto_mode std_srvs/srv/SetBool "{data: true}" || true

sleep 2

echo "===AUTO_STATUS==="
timeout 5s ros2 topic echo /mission/status --once || true

echo "===CMD_VEL_SAMPLE==="
timeout 5s ros2 topic echo /cmd_vel --once || true

echo "===CHASSIS_DURING_AUTO==="
timeout 5s ros2 topic echo /sentry/chassis/status --once || true

sleep 4

echo "===SET_AUTO_FALSE==="
timeout 8s ros2 service call /set_auto_mode std_srvs/srv/SetBool "{data: false}" || true

sleep 1

echo "===POST_STATUS==="
timeout 5s ros2 topic echo /mission/status --once || true

echo "===LOG_KEY_LINES==="
egrep -n "Mission control node ready|Nav2 ready|Switched to AUTO|State: MANUAL -> PATROL|Sent waypoint|Reached waypoint|Nav2 task failed|State: PATROL -> MANUAL|No chassis status|comm_timeout|static costmap|FATAL|process has died" "$LOG" | head -160 || true
