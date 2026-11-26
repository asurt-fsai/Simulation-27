#!/usr/bin/env bash
exec bash "$(dirname -- "$0")/run_autonomous_control.sh" pure_pursuit_auto_node "$@"
