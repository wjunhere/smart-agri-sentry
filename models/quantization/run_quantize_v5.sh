#!/bin/bash
# Quantize MobileNetV3 v5 (board-domain finetune) to RDK X5 BPU .bin (nv12 input)
set -e

WORK="E:/smart_agri_sentry/models/quantization"
ONNX="E:/smart_agri_sentry/models/tomato_mobilenetv3_v5.onnx"
CAL="D:/wjun/data/toamtos/models/mobilenetv3/bpu_workdir/calibration_v5_rgbchw"
IMAGE="openexplorer/ai_toolchain_ubuntu_20_x5_cpu:v1.2.8-py310"

MSYS_NO_PATHCONV=1 docker run --rm \
  -v "${WORK}:/data/work" \
  -v "${ONNX}:/data/model/tomato_mobilenetv3_v5.onnx:ro" \
  -v "${CAL}:/data/calibration:ro" \
  -w /data/work \
  "${IMAGE}" \
  hb_mapper makertbin --config /data/work/tomato_mobilenetv3_v5_config.yaml --model-type onnx

echo "Done."
