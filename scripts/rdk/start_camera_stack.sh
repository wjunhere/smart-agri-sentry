#!/usr/bin/env bash
# Clean-restart the minimal camera stack for the web frontend.
#
# Kills any existing camera/republish/rosbridge processes first (avoids
# duplicate nodes fighting over the MIPI device and topics), then starts
# fresh: mipi_camera_node (undistorted) + compressed republish + rosbridge.
#
# Safe to call repeatedly. Web node and other gateway services are untouched.

set -u
ROS_DISTRO_NAME="${ROS_DISTRO:-humble}"
WS_DIR="${SENTRY_WS:-/home/sunrise/dev_ws}"
LOG_DIR="${SENTRY_LOG_DIR:-/tmp}"

log() { printf '[start_camera_stack] %s\n' "$*"; }
fail() { printf '[start_camera_stack][ERROR] %s\n' "$*" >&2; exit 1; }

set +u
# shellcheck disable=SC1090
source "/opt/ros/${ROS_DISTRO_NAME}/setup.bash"
[ -f "${WS_DIR}/install/setup.bash" ] || fail "Missing ${WS_DIR}/install/setup.bash"
# shellcheck disable=SC1090
source "${WS_DIR}/install/setup.bash"
set -u
export FASTDDS_BUILTIN_TRANSPORTS="${FASTDDS_BUILTIN_TRANSPORTS:-UDPv4}"
export RMW_FASTRTPS_USE_SHM="${RMW_FASTRTPS_USE_SHM:-0}"

log "Killing existing camera/republish processes..."
pkill -f 'sentry_bringup.*mipi_camera_node' 2>/dev/null || true
pkill -f 'image_transport.*republish' 2>/dev/null || true
sleep 2
pkill -9 -f 'sentry_bringup.*mipi_camera_node' 2>/dev/null || true
pkill -9 -f 'image_transport.*republish' 2>/dev/null || true
sleep 1

log "Starting mipi_camera_node (undistort enabled)..."
setsid ros2 run sentry_bringup mipi_camera_node --ros-args \
  -p device_id:=2 -p width:=640 -p height:=480 -p fps:=10.0 \
  -p sensor_width:=1920 -p sensor_height:=1080 -p yuv_format:=nv12 \
  -p enable_color_correction:=true -p blue_gain:=1.0 -p green_gain:=0.98 -p red_gain:=1.0 \
  -p enable_low_light_enhancement:=true -p denoise_h:=0.0 -p gamma:=1.10 \
  -p saturation_scale:=0.95 -p sharpen_amount:=0.15 \
  -p enable_undistort:=true \
  -p undistort_calib_file:="${WS_DIR}/config/imx477_640x480.yaml" \
  -p undistort_alpha:=0.0 \
  >"${LOG_DIR}/camera_stack_mipi.log" 2>&1 &
echo "$!" > /tmp/mipi_camera_node.pid

log "Starting compressed republisher..."
setsid ros2 run image_transport republish raw compressed --ros-args \
  -r in:=/sentry/camera/image_raw -r out:=/out \
  >"${LOG_DIR}/camera_stack_republish.log" 2>&1 &
echo "$!" > /tmp/image_republisher.pid

if ! ros2 node list 2>/dev/null | grep -qx '/rosbridge_websocket'; then
  log "Starting rosbridge websocket :9090..."
  setsid ros2 run rosbridge_server rosbridge_websocket --ros-args -p port:=9090 \
    >"${LOG_DIR}/camera_stack_rosbridge.log" 2>&1 &
  echo "$!" > /tmp/rosbridge_websocket.pid
else
  log "rosbridge already running; keeping it."
fi

log "Waiting for camera frames on /out/compressed ..."
for attempt in $(seq 1 20); do
  if timeout 3 ros2 topic echo /out/compressed --once >/dev/null 2>&1; then
    log "Camera stack is up (frames flowing)."
    exit 0
  fi
  sleep 1
done

tail -5 "${LOG_DIR}/camera_stack_mipi.log" >&2 || true
fail "No frames on /out/compressed within 20s"
