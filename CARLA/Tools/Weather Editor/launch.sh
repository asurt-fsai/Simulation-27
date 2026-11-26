#!/usr/bin/env bash
# The application, icon and tests live in this tool directory.
set -euo pipefail
here="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
log_dir="${XDG_CACHE_HOME:-$HOME/.cache}/asurt"
mkdir -p "$log_dir"
exec 9>"$log_dir/carla-weather-gui.lock"
if ! flock -n 9; then
  command -v wmctrl >/dev/null 2>&1 && wmctrl -a "ASURT | CARLA Weather Control" >/dev/null 2>&1 || true
  exit 0
fi
cd "$here"
exec /usr/bin/python3 "$here/carla_weather_gui.py" >>"$log_dir/carla-weather-gui.log" 2>&1
