#!/usr/bin/env bash
# Validate historical LeGO pose geometry before publishing normalized odometry.
# Consumes only existing ROS topics; no simulator or TF ownership.
set -eo pipefail
source "$(dirname -- "${BASH_SOURCE[0]}")/slam-env.sh"
set -u
${ASURT_MODULE_RUNNER:-exec} "$slam_install/asurt_lidar_odom_adapter/lib/asurt_lidar_odom_adapter/lidar_odom_adapter_node" --ros-args \
  --params-file "$slam/Modules/LiDARMapping/asurt_lidar_odom_adapter/config/adapter.yaml" \
  --log-level info "$@"
