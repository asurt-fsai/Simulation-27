#!/usr/bin/env bash
# Optional simulator course map for testing against the normal SLAM topics.
set -eo pipefail
source "$(dirname -- "${BASH_SOURCE[0]}")/slam-env.sh"
set -u
export PYTHONPATH="$asurt_root/tools/RViz${PYTHONPATH:+:$PYTHONPATH}"
${ASURT_MODULE_RUNNER:-exec} "$slam_install/asurt_fake_slam/lib/asurt_fake_slam/${ASURT_FAKE_MAP_ENTRY:-fake_slam_node}" \
  --ros-args -p use_sim_time:=true "$@"
