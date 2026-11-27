#!/usr/bin/env bash
set -eo pipefail
root="${ASURT_ROOT:-$(cd -- "$(dirname -- "$0")/../../../.." && pwd)}"
stock="$root/libexec/asurt-fs26"
if [[ -n ${ASURT_MODULE_EXECUTABLE:-} ]]; then
    exec bash "$(dirname -- "$0")/run_cone_mapper.sh" "$@"
fi
export PYTHONPATH="$stock/cone_mapping${PYTHONPATH:+:$PYTHONPATH}"
exec "$stock/cone_mapping/lib/cone_mapping/cone_mapping_node" --ros-args \
    --params-file "$stock/cone_mapping/config/cone_mapping_params.yaml" -p use_sim_time:=true "$@"
