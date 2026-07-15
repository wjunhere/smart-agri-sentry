#!/usr/bin/env bash
# Find minimum effective speed using chassis_cmd
source /opt/ros/humble/setup.bash
source ~/dev_ws/install/setup.bash

echo "=== chassis_cmd min speed test ==="

for SPEED in 0.05 0.08 0.10 0.12 0.15 0.20; do
  echo ""
  echo "--- chassis_cmd --forward ${SPEED} --dist 0.2 ---"
  timeout 20 ros2 run sentry_mission chassis_cmd --forward $SPEED --dist 0.2 2>&1
  echo "Exit code: $?"
  sleep 2
done

echo ""
echo "=== Done ==="
