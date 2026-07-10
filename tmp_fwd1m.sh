#!/usr/bin/env bash
source /opt/ros/humble/setup.bash
source ~/dev_ws/install/setup.bash

echo "=== 启动系统 ==="
setsid ros2 launch sentry_bringup sentry_v2.launch.py crop_type:=tomato > /tmp/fwd1m_launch.log 2>&1 &
LAUNCH_PID=$!

echo "等待初始化..."
sleep 35

echo "=== 前进1m测试 ==="
ros2 run sentry_mission chassis_cmd --forward 0.2 --dist 1.0 2>&1
RESULT=$?

echo "Exit: $RESULT"
echo "=== 清理 ==="
kill -- -$LAUNCH_PID 2>/dev/null
