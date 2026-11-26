#!/usr/bin/env bash
# Fuse only existing ZED and Velodyne perception observations.
set -eo pipefail
source "$(dirname -- "${BASH_SOURCE[0]}")/slam-env.sh"
set -u
${ASURT_MODULE_RUNNER:-exec} "$slam_install/asurt_slam_fusion/lib/asurt_slam_fusion/slam_fusion_node" --ros-args \
  --params-file "$slam/Modules/Fusion/asurt_slam_fusion/config/fusion.yaml" \
  --log-level info "$@"
