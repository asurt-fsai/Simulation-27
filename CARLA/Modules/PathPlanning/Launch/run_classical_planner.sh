#!/usr/bin/env bash
set -euo pipefail

ASURT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../../.." && pwd)"
PLANNING="$ASURT_ROOT/Simulation/Systems/PathPlanning"
UNDERLAY="$ASURT_ROOT/Simulation/Systems/Common/ROSUnderlays/fs-system-26-test-build/install/setup.bash"
PERCEPTION="$ASURT_ROOT/Simulation/Systems/Perception/Workspace/autonomy_integration/install/setup.bash"
SLAM="$ASURT_ROOT/Simulation/Systems/SLAM/Workspace/slam_integration/install/setup.bash"
PLANNING_OVERLAY="$PLANNING/Workspace/planning_integration/install/setup.bash"
CONFIG="$PLANNING/Config/classical.yaml"

# Reuse the bridge and sensor publishers already launched by the native CARLA
# workflow; this executable never starts CARLA or a controller.
set +u
source /opt/ros/humble/setup.bash
for overlay in "$UNDERLAY" "$PERCEPTION" "$SLAM" "$PLANNING_OVERLAY"; do
    if [[ -f "$overlay" ]]; then
        source "$overlay"
    fi
done
set -u
if [[ ! -f "$PLANNING_OVERLAY" ]]; then
    echo "Classical planner has not been built: $PLANNING_OVERLAY" >&2
    exit 2
fi
${ASURT_MODULE_RUNNER:-exec} ros2 run asurt_classical_planning classical_planner_node --ros-args --params-file "$CONFIG" "$@"
