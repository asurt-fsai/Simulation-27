#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/../../../.." && pwd)"
CONTROL="$ROOT/Simulation/Systems/Control"
INSTALL="$CONTROL/Workspace/control_integration/install/setup.bash"
if [[ ! -f "$INSTALL" ]]; then
    echo "Build Control ROS workspace first: $INSTALL" >&2
    exit 2
fi
set +u
source /opt/ros/humble/setup.bash
for overlay in \
    "$ROOT/Simulation/Systems/Common/ROSUnderlays/fs-system-26-test-build/install/setup.bash" \
    "$ROOT/Simulation/Systems/Perception/Workspace/autonomy_integration/install/setup.bash" \
    "$ROOT/Simulation/Systems/PathPlanning/Workspace/planning_integration/install/setup.bash" \
    "$INSTALL"; do
    [[ ! -f "$overlay" ]] || source "$overlay"
done
set -u
# Uses the already-running CARLA bridge; creates no vehicle/sensor/ticker.
${ASURT_MODULE_RUNNER:-exec} ros2 run asurt_pure_pursuit_control pure_pursuit_control_node --ros-args \
    --params-file "$CONTROL/Config/pure_pursuit.yaml" "$@"
