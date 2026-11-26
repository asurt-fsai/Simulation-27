#!/usr/bin/env bash
# Test the ROS CUDA adapter only in an explicitly isolated DDS domain.
# This launches no CARLA actors, sensors, controllers, or simulator ticker.
set -euo pipefail
here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
root="$(cd "$here/../../../../../.." && pwd)"
site="$here/.venv/lib/python3.10/site-packages"
if [[ -z "${ROS_DOMAIN_ID:-}" || "$ROS_DOMAIN_ID" == 0 ]]; then
    echo "Set a non-live ROS_DOMAIN_ID (for example, ROS_DOMAIN_ID=214)" >&2
    exit 2
fi
if [[ ! -d "$site/onnxruntime" ]]; then
    echo "Missing isolated ONNX Runtime GPU environment" >&2
    exit 2
fi
set +u
source /opt/ros/humble/setup.bash
source "$root/Simulation/Systems/Common/ROSUnderlays/fs-system-26-test-build/install/setup.bash"
source "$root/Simulation/Systems/PathPlanning/Workspace/planning_integration/install/setup.bash"
set -u
export ASURT_DL_TEST_BACKEND=cuda
export PYTHONPATH="$site:$root/Simulation/Systems/PathPlanning/Modules/DeepLearning/asurt_deep_learning_planning:$root/Simulation/Systems/Perception/Dependencies/onnxruntime:${PYTHONPATH:-}"
export LD_LIBRARY_PATH="$here/.cuda_libs:$site/nvidia/curand/lib:$site/nvidia/cudnn/lib:$site/nvidia/cublas/lib:$site/nvidia/cuda_runtime/lib:${LD_LIBRARY_PATH:-}"
exec "$here/.venv/bin/python" "$root/Simulation/Systems/PathPlanning/Tests/test_dl_ros_contract.py"
