#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../../.." && pwd)"
PLANNING="$ROOT/Simulation/Systems/PathPlanning"
set +u
source /opt/ros/humble/setup.bash
for setup in "$ROOT/Simulation/Systems/Common/ROSUnderlays/fs-system-26-test-build/install/setup.bash" \
             "$ROOT/Simulation/Systems/Perception/Workspace/autonomy_integration/install/setup.bash" \
             "$ROOT/Simulation/Systems/SLAM/Workspace/slam_integration/install/setup.bash" \
             "$PLANNING/Workspace/planning_integration/install/setup.bash"; do
  [[ ! -r "$setup" ]] || source "$setup"
done
set -u
${ASURT_MODULE_RUNNER:-exec} ros2 run asurt_skidpad_planning skidpad_planner_node --ros-args --params-file "$PLANNING/Config/skidpad.yaml" "$@"
