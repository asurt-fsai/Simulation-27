#!/usr/bin/env bash
set -eo pipefail
source "$(dirname -- "${BASH_SOURCE[0]}")/perception-env.sh"

case "${ASURT_LIDAR_PROFILE:-legacy_box}" in
    legacy_box|legacy_lut|fast) profile="${ASURT_LIDAR_PROFILE:-legacy_box}" ;;
    *) echo "Unknown ASURT_LIDAR_PROFILE (legacy_box, legacy_lut, fast)" >&2; exit 2 ;;
esac

set -u
${ASURT_MODULE_RUNNER:-exec} ros2 run asurt_lidar_perception lidar_perception_node --ros-args \
    --params-file "$WORKSPACE/src/asurt_lidar_perception/config/$profile.yaml" \
    -p use_sim_time:=true --log-level warn "$@"
