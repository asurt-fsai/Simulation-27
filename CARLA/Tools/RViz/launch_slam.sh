#!/usr/bin/env bash
set -euo pipefail
pc=/mnt/e/ASURT/Simulation/CARLA/Launchers
case "${1:-}" in
    "") view=slam-rviz ;;
    --diagnostics) view=slam-diagnostics ;;
    *) echo "Usage: $0 [--diagnostics]" >&2; exit 2 ;;
esac
exec bash "$pc/ros-parity-env.sh" python "$pc/session.py" auto "$view"
