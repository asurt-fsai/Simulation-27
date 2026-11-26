#!/usr/bin/env bash
set -euo pipefail
root="$(cd -- "$(dirname -- "$0")/../.." && pwd)"
exec "$root/.venvs/carla/bin/python" /mnt/e/ASURT/Simulation/CARLA/Launchers/session.py "${1:-auto}" manager
