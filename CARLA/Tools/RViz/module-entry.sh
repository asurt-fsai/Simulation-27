#!/usr/bin/env bash
set -eo pipefail

setup=$1
executable=$2
launcher=$3
shift 3
if [[ ! -x "$executable" ]]; then
    echo "Local module executable is missing or not executable: $executable" >&2
    exit 126
fi

asurt_module_exec() {
    if [[ "$1" == ros2 && "$2" == run ]]; then
        shift 4
    else
        shift
    fi
    set +u
    if [[ -n "$ASURT_MODULE_SETUP" ]]; then
        source "$ASURT_MODULE_SETUP"
    fi
    exec "$ASURT_MODULE_EXECUTABLE" "$@"
}
export -f asurt_module_exec

if [[ -z "$launcher" ]]; then
    if [[ -n "$setup" ]]; then source "$setup"; fi
    exec "$executable" "$@"
fi

export ASURT_MODULE_SETUP="$setup" ASURT_MODULE_EXECUTABLE="$executable"
export ASURT_MODULE_RUNNER=asurt_module_exec
exec bash "$launcher" "$@"
