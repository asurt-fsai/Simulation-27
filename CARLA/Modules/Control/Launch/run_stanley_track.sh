#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/../../../.." && pwd)"
C="$ROOT/Simulation/Systems/Control"
OVERLAY="$C/Workspace/control_integration/install/setup.bash"
[[ -f "$OVERLAY" ]] || { echo "Build Control ROS overlay first" >&2; exit 2; }
set +u
source /opt/ros/humble/setup.bash
for setup in \
    "$ROOT/Simulation/Systems/Common/ROSUnderlays/fs-system-26-test-build/install/setup.bash" \
    "$ROOT/Simulation/Systems/PathPlanning/Workspace/planning_integration/install/setup.bash" \
    "$OVERLAY"; do
    [[ -f "$setup" ]] && source "$setup"
done
set -u
${ASURT_MODULE_RUNNER:-exec} ros2 run asurt_stanley_control stanley_control_node --ros-args \
    --params-file "$C/Config/stanley_track.yaml" "$@"
