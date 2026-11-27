#!/usr/bin/env bash
# Launch the original controller and keep simulator topic remaps outside its source.
set -eo pipefail

controller=$1
shift
case "$controller" in
    simple)
        package=simple_pure_pursuit
        executable=simple_pp_node
        config=simplepurepuirsuit_parameters.yaml
        command_topic=/ackermann_cmd
        ;;
    adaptive)
        package=adaptive_pure_pursuit
        executable=adaptive_pp_node
        config=adaptive_pp_parameters.yaml
        command_topic=/ackr
        ;;
    *)
        echo "Expected simple or adaptive FS-System-26 control." >&2
        exit 2
        ;;
esac

root="${ASURT_ROOT:-$(cd -- "$(dirname -- "$0")/../../../.." && pwd)}"
stock="$root/libexec/asurt-fs26"
export PYTHONPATH="$stock/$package:$stock/dependencies${PYTHONPATH:+:$PYTHONPATH}"
export MPLBACKEND=Agg
if [[ -n ${ASURT_MODULE_SETUP:-} ]]; then source "$ASURT_MODULE_SETUP"; fi

# In CARLA each planner has its own topic. Legacy Control receives the
# chosen stream as the original /path, in the vehicle's local frame.
exec "$stock/control_pair" "${ASURT_MODULE_EXECUTABLE:-$stock/$package/lib/$package/$executable}" \
    "$stock/$package/config/$config" "$command_topic" "$@"
