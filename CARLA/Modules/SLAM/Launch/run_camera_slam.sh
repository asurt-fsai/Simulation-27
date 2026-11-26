#!/usr/bin/env bash
# Shared ZED YOLO26 cone observations + existing bridge odometry prior.
set -eo pipefail
source "$(dirname -- "${BASH_SOURCE[0]}")/slam-env.sh"
set -u
${ASURT_MODULE_RUNNER:-exec} "$slam_install/asurt_camera_slam/lib/asurt_camera_slam/camera_slam_node" --ros-args \
  --params-file "$slam/Modules/CameraLocalization/asurt_camera_slam/config/camera_slam.yaml" \
  --log-level info "$@"
