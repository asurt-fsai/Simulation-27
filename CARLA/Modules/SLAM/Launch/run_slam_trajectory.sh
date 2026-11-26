#!/usr/bin/env bash
set -eo pipefail
source "$(dirname -- "${BASH_SOURCE[0]}")/slam-env.sh"
set -u
${ASURT_MODULE_RUNNER:-exec} "$slam_install/asurt_cone_slam/lib/asurt_cone_slam/slam_trajectory_node" --ros-args \
  --params-file "$slam/Modules/ConeMapping/asurt_cone_slam/config/slam.yaml" \
  --log-level warn "$@"
