#!/usr/bin/env bash
# Dedicated high-speed manual-goal RViz with SLAM track markers.
set -euo pipefail
here="$(cd -- "$(dirname -- "$0")" && pwd)"
exec "$here/launch_planning.sh" track_control
