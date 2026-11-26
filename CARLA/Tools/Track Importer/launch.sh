#!/usr/bin/env bash
# Run the tool-owned GUI with the bridge's existing CARLA Python environment.
set -euo pipefail
here="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
root="$(cd -- "$here/../.." && pwd)"
bridge="$root/Simulation/CARLA/Bridge/Carla_ASU_Bridge"
[[ -d "$bridge" ]] || bridge="$root/Carla_ASU_Bridge"
cd "$bridge"
exec "$root/.venvs/asurt-native/bin/python" "$here/cone_track_gui.py" "$@"
