#!/usr/bin/env bash
# Stop the model inference stack started by start_inference_stack.sh.
#
# Kills plant_detector_node and vision_pipeline_node. The camera stack is
# untouched. Safe to call repeatedly.

set -u

log() { printf '[stop_inference_stack] %s\n' "$*"; }

log "Stopping plant_detector_node / vision_pipeline_node..."
pkill -f 'sentry_vision.*plant_detector_node' 2>/dev/null || true
pkill -f 'sentry_vision.*vision_pipeline_node' 2>/dev/null || true
sleep 2
pkill -9 -f 'sentry_vision.*plant_detector_node' 2>/dev/null || true
pkill -9 -f 'sentry_vision.*vision_pipeline_node' 2>/dev/null || true

rm -f /tmp/plant_detector_node.pid /tmp/vision_pipeline_node.pid 2>/dev/null || true
log "Inference stack stopped."
exit 0
