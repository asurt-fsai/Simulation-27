#!/usr/bin/env bash
set -eo pipefail
entry=$1
shift
root="$(cd -- "$(dirname -- "$0")/../../../.." && pwd)"
install="$root/Simulation/Systems/Control/Workspace/control_integration/install"
source /opt/ros/humble/setup.bash
source "$install/local_setup.bash"
export ASURT_ROOT="$root"
export PYTHONPATH="$root/tools/RViz${PYTHONPATH:+:$PYTHONPATH}"
${ASURT_MODULE_RUNNER:-exec} "$install/asurt_autonomous_control/lib/asurt_autonomous_control/$entry" \
    --ros-args -p use_sim_time:=true "$@"
