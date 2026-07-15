#!/usr/bin/env bash
source /opt/ros/humble/setup.bash
source ~/dev_ws/install/setup.bash

echo "=== 启动系统 ==="
setsid ros2 launch sentry_bringup sentry_v2.launch.py crop_type:=tomato > /tmp/nav2_fix.log 2>&1 &
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
echo "系统已初始化"

echo ""
echo "=== 1. Nav2生命周期 ==="
grep -iE "Activat|Configur|lifecycle_manager.*Creating|lifecycle_manager.*activat" /tmp/nav2_fix.log 2>/dev/null | head -15

echo ""
echo "=== 2. LiDAR近场检查 (扫描最近10个点) ==="
ros2 topic echo /scan --once 2>/dev/null | grep -A5 "ranges:" | head -20

echo ""
echo "=== 3. 激活AUTO模式 ==="
ros2 service call /set_auto_mode std_srvs/srv/SetBool "{data: true}" 2>/dev/null

echo "等待10秒..."
sleep 10

echo ""
echo "=== 4. cmd_vel采样 (连续3次) ==="
for i in 1 2 3; do
  echo "--- sample $i ---"
  ros2 topic echo /cmd_vel --once 2>/dev/null
  sleep 2
done

echo ""
echo "=== 5. 控制器状态 ==="
ros2 topic echo /mission/status --once 2>/dev/null

echo ""
echo "=== 6. 关闭AUTO ==="
ros2 service call /set_auto_mode std_srvs/srv/SetBool "{data: false}" 2>/dev/null

echo ""
echo "=== 7. Nav2关键日志 ==="
grep -iE "controller_server.*Failed|Nav2 ready|Sent waypoint|Reached|progress|FAIL|ERROR|activat" /tmp/nav2_fix.log 2>/dev/null | head -30
