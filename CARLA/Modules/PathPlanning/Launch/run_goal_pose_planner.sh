#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/../../../.." && pwd)"
P="$ROOT/Simulation/Systems/PathPlanning"
OVERLAY="$P/Workspace/planning_integration/install/setup.bash"
[[ -f "$OVERLAY" ]] || { echo "Build PathPlanning ROS overlay: $OVERLAY" >&2; exit 2; }
set +u
source /opt/ros/humble/setup.bash
for setup in \
    "$ROOT/Simulation/Systems/Common/ROSUnderlays/fs-system-26-test-build/install/setup.bash" \
    "$OVERLAY"; do
    [[ -f "$setup" ]] && source "$setup"
done
set -u
${ASURT_MODULE_RUNNER:-exec} ros2 run asurt_goal_pose_planning goal_pose_planner_node --ros-args \
    --params-file "$P/Config/goal_pose.yaml" "$@"
