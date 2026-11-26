#!/usr/bin/env bash
# Consume the existing Perception LiDAR; never create actors, /clock, or RViz.
set -eo pipefail
source "$(dirname -- "${BASH_SOURCE[0]}")/slam-env.sh"
set -u
gtsam="$slam/ThirdParty/gtsam-prefix/opt/ros/humble"
if [[ -d "$gtsam/lib/x86_64-linux-gnu" ]]; then
  export LD_LIBRARY_PATH="$gtsam/lib/x86_64-linux-gnu:${LD_LIBRARY_PATH:-}"
fi
model="${ASURT_VELODYNE_MODEL:-vlp32c}"
case "$model" in
  vlp32c|vlp16) ;;
  *) echo "Unknown ASURT_VELODYNE_MODEL (vlp32c, vlp16)" >&2; exit 2 ;;
esac
${ASURT_MODULE_RUNNER:-exec} "$slam_install/lego_loam_sr/lib/lego_loam_sr/lego_loam_sr" --ros-args \
  --params-file "$slam/Modules/LiDARMapping/LeGO-LOAM/config/asurt_$model.yaml" \
  -r /tf:=/slam/lidar/tf -r /tf_static:=/slam/lidar/tf_static \
  --log-level warn "$@"
