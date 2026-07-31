#!/usr/bin/env bash
# Clean-restart the model inference stack: YOLO plant detector + vision
# pipeline (YOLO crop + MobileNet disease classification).
#
# Kills existing inference nodes first (avoids duplicate nodes and double
# BPU model loading), then starts fresh with the fine-tuned plant model.
#
# Safe to call repeatedly. Camera stack is untouched (requires it running).

set -u
ROS_DISTRO_NAME="${ROS_DISTRO:-humble}"
WS_DIR="${SENTRY_WS:-/home/sunrise/dev_ws}"
LOG_DIR="${SENTRY_LOG_DIR:-/tmp}"
YOLO_MODEL="${SENTRY_YOLO_MODEL:-${WS_DIR}/models/best_plant_11s_bayese_640x640_nv12.bin}"
CONF_THRES="${SENTRY_YOLO_CONF:-0.35}"
MIN_AREA="${SENTRY_YOLO_MIN_AREA:-0.01}"

log() { printf '[start_inference_stack] %s\n' "$*"; }
fail() { printf '[start_inference_stack][ERROR] %s\n' "$*" >&2; exit 1; }

set +u
# shellcheck disable=SC1090
source "/opt/ros/${ROS_DISTRO_NAME}/setup.bash"
[ -f "${WS_DIR}/install/setup.bash" ] || fail "Missing ${WS_DIR}/install/setup.bash"
# shellcheck disable=SC1090
source "${WS_DIR}/install/setup.bash"
set -u
export FASTDDS_BUILTIN_TRANSPORTS="${FASTDDS_BUILTIN_TRANSPORTS:-UDPv4}"
export RMW_FASTRTPS_USE_SHM="${RMW_FASTRTPS_USE_SHM:-0}"

[ -f "$YOLO_MODEL" ] || fail "YOLO model not found: ${YOLO_MODEL}"

log "Killing existing inference nodes..."
pkill -f 'sentry_vision.*plant_detector_node' 2>/dev/null || true
pkill -f 'sentry_vision.*vision_pipeline_node' 2>/dev/null || true
pkill -f 'sentry_vision.*vision_diagnosis_node' 2>/dev/null || true
sleep 2
pkill -9 -f 'sentry_vision.*plant_detector_node' 2>/dev/null || true
pkill -9 -f 'sentry_vision.*vision_pipeline_node' 2>/dev/null || true
pkill -9 -f 'sentry_vision.*vision_diagnosis_node' 2>/dev/null || true
sleep 1

log "Starting plant_detector_node (model: ${YOLO_MODEL})..."
setsid ros2 run sentry_vision plant_detector_node --ros-args \
  -p confidence_threshold:="${CONF_THRES}" \
  -p min_area_ratio:="${MIN_AREA}" \
  -p model_path:="${YOLO_MODEL}" \
  >"${LOG_DIR}/inference_plant_detector.log" 2>&1 &
echo "$!" > /tmp/plant_detector_node.pid

log "Starting vision_pipeline_node (YOLO + MobileNet)..."
setsid ros2 run sentry_vision vision_pipeline_node --ros-args \
  -p yolo_model_path:="${YOLO_MODEL}" \
  >"${LOG_DIR}/inference_vision_pipeline.log" 2>&1 &
echo "$!" > /tmp/vision_pipeline_node.pid

# Streaming disease diagnosis for the frontend: classifies the plant_detector
# crop (letterbox) continuously and publishes /vision/diagnosis.
log "Starting vision_diagnosis_node (streaming, YOLO-crop input)..."
setsid ros2 run sentry_vision vision_diagnosis_node --ros-args \
  -p crop_type:="${CROP_TYPE:-tomato}" \
  -p healthy_threshold:=0.0 \
  >"${LOG_DIR}/inference_vision_diagnosis.log" 2>&1 &
echo "$!" > /tmp/vision_diagnosis_node.pid

log "Waiting for inference nodes in ROS graph..."
for attempt in $(seq 1 15); do
  nodes="$(ros2 node list 2>/dev/null || true)"
  det_ok=false; pipe_ok=false; diag_ok=false
  printf '%s\n' "$nodes" | grep -qx '/plant_detector_node' && det_ok=true
  printf '%s\n' "$nodes" | grep -qx '/vision_pipeline_node' && pipe_ok=true
  printf '%s\n' "$nodes" | grep -qx '/vision_diagnosis_node' && diag_ok=true
  if $det_ok && $pipe_ok && $diag_ok; then
    log "Inference stack is up (plant_detector + vision_pipeline + vision_diagnosis)."
    exit 0
  fi
  sleep 1
done

tail -5 "${LOG_DIR}/inference_plant_detector.log" >&2 || true
tail -5 "${LOG_DIR}/inference_vision_pipeline.log" >&2 || true
tail -5 "${LOG_DIR}/inference_vision_diagnosis.log" >&2 || true
fail "Inference nodes did not appear within 15s"
