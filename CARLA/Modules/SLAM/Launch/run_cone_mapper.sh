#!/usr/bin/env bash
# Passive source: existing /perception_landmarks, /odom and TF; no CARLA actions.
set -eo pipefail
source "$(dirname -- "${BASH_SOURCE[0]}")/slam-env.sh"
set -u
${ASURT_MODULE_RUNNER:-exec} "$slam_install/asurt_cone_slam/lib/asurt_cone_slam/cone_mapper_node" --ros-args \
  --params-file "$slam/Modules/ConeMapping/asurt_cone_slam/config/slam.yaml" \
  --log-level info "$@"
