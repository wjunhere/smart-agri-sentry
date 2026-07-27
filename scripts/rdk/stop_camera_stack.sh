#!/usr/bin/env bash
# Stop the minimal camera stack started by start_camera_stack.sh.
#
# Kills mipi_camera_node and the compressed republisher. rosbridge is left
# running on purpose: it is shared with the frontend control plane and the
# inference stack. Safe to call repeatedly.

set -u

log() { printf '[stop_camera_stack] %s\n' "$*"; }

log "Stopping mipi_camera_node / image republisher..."
pkill -f 'sentry_bringup.*mipi_camera_node' 2>/dev/null || true
pkill -f 'image_transport.*republish' 2>/dev/null || true
sleep 2
pkill -9 -f 'sentry_bringup.*mipi_camera_node' 2>/dev/null || true
pkill -9 -f 'image_transport.*republish' 2>/dev/null || true

rm -f /tmp/mipi_camera_node.pid /tmp/image_republisher.pid 2>/dev/null || true
log "Camera stack stopped."
exit 0
