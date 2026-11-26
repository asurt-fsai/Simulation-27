#!/usr/bin/env bash
set -eo pipefail
source "$(dirname -- "${BASH_SOURCE[0]}")/perception-env.sh"
set -u
${ASURT_MODULE_RUNNER:-exec} ros2 run perception_zed_pkg conversion_node --ros-args --log-level warn "$@"
