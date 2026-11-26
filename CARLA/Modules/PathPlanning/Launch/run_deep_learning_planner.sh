#!/usr/bin/env bash
set -euo pipefail

ASURT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../../.." && pwd)"
PLANNING="$ASURT_ROOT/Simulation/Systems/PathPlanning"
UNDERLAY="$ASURT_ROOT/Simulation/Systems/Common/ROSUnderlays/fs-system-26-test-build/install/setup.bash"
PERCEPTION="$ASURT_ROOT/Simulation/Systems/Perception/Workspace/autonomy_integration/install/setup.bash"
SLAM="$ASURT_ROOT/Simulation/Systems/SLAM/Workspace/slam_integration/install/setup.bash"
PLANNING_OVERLAY="$PLANNING/Workspace/planning_integration/install/setup.bash"
MODEL="$ASURT_ROOT/Simulation/Systems/Legacy/fs-system-26-oldstack/Deeplearning/src/planning_deep_learning/planning_deep_learning/Completed_Models/best_model.onnx"
CONFIG="$PLANNING/Config/deep_learning.yaml"
ORT_DEPS="$ASURT_ROOT/Simulation/Systems/Perception/Dependencies/onnxruntime"
GPU_SITE="$PLANNING/Modules/DeepLearning/GPU/.venv/lib/python3.10/site-packages"
GPU_LIBS="$PLANNING/Modules/DeepLearning/GPU/.cuda_libs"

if [[ ! -f "$MODEL" ]]; then
    echo "Pretrained ASURT ONNX model missing: $MODEL" >&2
    exit 2
fi
if [[ ! -f "$PLANNING_OVERLAY" ]]; then
    echo "Planning workspace has not been built: $PLANNING_OVERLAY" >&2
    exit 2
fi
# This is a passive planner. All sensor and CARLA processes are external.
set +u
source /opt/ros/humble/setup.bash
for overlay in "$UNDERLAY" "$PERCEPTION" "$SLAM" "$PLANNING_OVERLAY"; do
    if [[ -f "$overlay" ]]; then
        source "$overlay"
    fi
done
set -u
if [[ -d "$GPU_SITE/onnxruntime" ]]; then
    # Isolated CUDA/cuDNN runtime; the ROS node still strictly requires the
    # requested CUDA provider, so failure never masquerades as GPU success.
    export LD_LIBRARY_PATH="$GPU_SITE/nvidia/curand/lib:$GPU_SITE/nvidia/cudnn/lib:$GPU_SITE/nvidia/cublas/lib:$GPU_SITE/nvidia/cuda_runtime/lib:${LD_LIBRARY_PATH:-}"
    if [[ -d "$GPU_LIBS" ]]; then
        export LD_LIBRARY_PATH="$GPU_LIBS:$LD_LIBRARY_PATH"
    fi
    # Preserve bundled coloredlogs/flatbuffers, but import GPU ORT first.
    export PYTHONPATH="$GPU_SITE:$ORT_DEPS:${PYTHONPATH:-}"
elif [[ -d "$ORT_DEPS/onnxruntime" ]]; then
    # Can run explicit CPU diagnostics; default backend=cuda will stay
    # unavailable until the GPU environment is installed and validated.
    export PYTHONPATH="$ORT_DEPS:${PYTHONPATH:-}"
fi
${ASURT_MODULE_RUNNER:-exec} ros2 run asurt_deep_learning_planning deep_learning_planner_node --ros-args \
    --params-file "$CONFIG" -p "model_path:=$MODEL" "$@"
