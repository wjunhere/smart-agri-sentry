#!/usr/bin/env bash
# Minimum speed threshold test for Smart Agri Sentry
# Tests what cmd_vel values actually make the robot move

source /opt/ros/humble/setup.bash
source ~/dev_ws/install/setup.bash

# Start minimal environment
ros2 run sentry_sensors uart_bridge_node --ros-args \
  -p uart_port:=/dev/ttyS1 -p baudrate:=115200 -p forward_servo_cmd:=false &
UART_PID=$!
sleep 2
echo "UART bridge started (PID=$UART_PID)"

cleanup() {
  echo ""
  echo "=== Cleanup ==="
  ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
    "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" 2>/dev/null || true
  kill $UART_PID 2>/dev/null || true
}
trap cleanup EXIT

for SPEED in 0.05 0.08 0.10 0.12 0.15 0.20; do
  echo ""
  echo "========================================="
  echo "=== Testing cmd_vel = ${SPEED} m/s ==="
  echo "========================================="

  # Publish continuously for 2 seconds
  timeout 3 bash -c "source /opt/ros/humble/setup.bash && source ~/dev_ws/install/setup.bash && ros2 topic pub -r 20 /cmd_vel geometry_msgs/msg/Twist '{linear: {x: $SPEED, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'" &
  PUB_PID=$!

  sleep 1
  echo "Chassis status during ${SPEED} m/s:"
  timeout 2 ros2 topic echo /sentry/chassis/status --once 2>/dev/null || echo "(no data)"

  kill $PUB_PID 2>/dev/null
  wait $PUB_PID 2>/dev/null

  # Stop
  ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
    "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" 2>/dev/null
  sleep 2
done

echo ""
echo "=== All tests complete ==="
