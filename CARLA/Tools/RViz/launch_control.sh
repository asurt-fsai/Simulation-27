#!/usr/bin/env bash
# One-click dedicated Control RViz. Does not enable/start Control or CARLA.
set -euo pipefail
here="$(cd -- "$(dirname -- "$0")" && pwd)"
exec "$here/launch_planning.sh" control
