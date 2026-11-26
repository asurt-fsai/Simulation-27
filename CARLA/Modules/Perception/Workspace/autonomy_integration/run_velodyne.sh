#!/usr/bin/env bash
set -eo pipefail
source "$(dirname -- "${BASH_SOURCE[0]}")/perception-env.sh"

model="${ASURT_VELODYNE_MODEL:-vlp32c}"
case "$model" in
    vlp32c) scale=2 ;;
    vlp16) scale=1 ;;
    *) echo "Unknown ASURT_VELODYNE_MODEL (vlp32c, vlp16)" >&2; exit 2 ;;
esac
case "${ASURT_VELODYNE_PROFILE:-reference}" in
    performance) points_per_second=$((120000 * scale)) ;;
    balanced) points_per_second=$((180000 * scale)) ;;
    reference) points_per_second=$((300000 * scale)) ;;
    *) echo "Unknown ASURT_VELODYNE_PROFILE (performance, balanced, reference)" >&2; exit 2 ;;
esac

set -u
${ASURT_MODULE_RUNNER:-exec} ros2 run asurt_velodyne_sim velodyne_sim_node --ros-args \
    --params-file "$PERCEPTION/Adapters/CARLA/LiDAR/asurt_velodyne_sim/config/$model.yaml" \
    -p "host:=${ASURT_CARLA_HOST:-127.0.0.1}" -p "port:=${ASURT_CARLA_PORT:-2000}" \
    -p "points_per_second:=$points_per_second" -p use_sim_time:=true "$@"
