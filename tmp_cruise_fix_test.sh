#!/usr/bin/env bash
export PATH="$HOME/.local/bin:$PATH"
source /opt/ros/humble/setup.bash
source ~/dev_ws/install/setup.bash

echo "=== 启动 sentry_v2 ==="
setsid ros2 launch sentry_bringup sentry_v2.launch.py crop_type:=tomato > /tmp/cruise_fix.log 2>&1 &
LAUNCH_PID=$!

cleanup() {
  echo "=== 清理 ==="
  ros2 service call /set_auto_mode std_srvs/srv/SetBool "{data: false}" 2>/dev/null || true
  ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
    "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" 2>/dev/null || true
  kill -- -$LAUNCH_PID 2>/dev/null || true
}
trap cleanup EXIT

sleep 38
echo "系统就绪"

echo ""
echo "=== 1. Chassis基线 ==="
ros2 topic echo /sentry/chassis/status --once 2>/dev/null

echo ""
echo "=== 2. 激活AUTO ==="
ros2 service call /set_auto_mode std_srvs/srv/SetBool "{data: true}" 2>/dev/null

echo ""
echo "=== 3. 监控15秒 ==="
for i in $(seq 1 5); do
  sleep 3
  echo "--- t+$((i*3))s ---"
  ros2 topic echo /cmd_vel --once 2>/dev/null
  ros2 topic echo /mission/status --once 2>/dev/null | grep -E "state:|current_wp_idx:"
done

echo ""
echo "=== 4. 关闭AUTO ==="
ros2 service call /set_auto_mode std_srvs/srv/SetBool "{data: false}" 2>/dev/null

echo ""
echo "=== 5. 关键日志 ==="
grep -iE "State:|Sent waypoint|Reached|Nav2 ready|Failed|progress|Boost" /tmp/cruise_fix.log 2>/dev/null | head -30
