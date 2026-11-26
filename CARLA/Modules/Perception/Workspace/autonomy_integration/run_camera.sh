#!/usr/bin/env bash
set -eo pipefail
source "$(dirname -- "${BASH_SOURCE[0]}")/perception-env.sh"
export PYTHONPATH="$PERCEPTION/Dependencies/onnxruntime${PYTHONPATH:+:$PYTHONPATH}"
source "$(dirname -- "${BASH_SOURCE[0]}")/camera-inference-env.sh"

detector_overrides=()
if [[ -n "${ASURT_VISION_BACKEND:-}" ]]; then
    detector_overrides+=(-p "vision_backend:=$ASURT_VISION_BACKEND")
fi
model_path="${ASURT_YOLO_MODEL_PATH:-$PERCEPTION/Models/yolo26n_fsoco_e2e_opset17.onnx}"
detector_overrides+=(-p "yolo_model_path:=$model_path")

set -u
${ASURT_MODULE_RUNNER:-exec} ros2 run asurt_zed2i_sim zed2i_sim_node --ros-args \
    --params-file "$WORKSPACE/src/asurt_zed2i_sim/config/zed2i.yaml" \
    -p "host:=${ASURT_CARLA_HOST:-127.0.0.1}" -p "port:=${ASURT_CARLA_PORT:-2000}" \
    "${detector_overrides[@]}" "$@"
