#!/usr/bin/env bash
set -eo pipefail
root="${ASURT_ROOT:-$(cd -- "$(dirname -- "$0")/../../../.." && pwd)}"
if [[ -n ${ASURT_MODULE_EXECUTABLE:-} ]]; then
    exec bash "$(dirname -- "$0")/run_goal_pose_planner.sh" "$@"
fi
exec "$root/libexec/asurt-fs26/classical_goal" --ros-args -p use_sim_time:=true "$@"
