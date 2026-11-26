#!/usr/bin/env bash
set -euo pipefail
pc=/mnt/e/ASURT/Simulation/CARLA/Launchers
case "${1:-}" in
    "") view=rviz ;;
    --zed) view=zed-rviz ;;
    *) echo "Usage: $0 [--zed]" >&2; exit 2 ;;
esac
exec bash "$pc/ros-parity-env.sh" python "$pc/session.py" auto "$view"
