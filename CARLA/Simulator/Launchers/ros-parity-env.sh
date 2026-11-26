#!/usr/bin/env bash
set -eo pipefail
ASURT_ROOT="$HOME/ASURT"
ASURT_BRIDGE="$ASURT_ROOT/Simulation/CARLA/Bridge/Carla_ASU_Bridge"
ASURT_PERCEPTION="$ASURT_ROOT/Simulation/Systems/Perception/Workspace/autonomy_integration"
ASURT_COMMON="$ASURT_ROOT/Simulation/Systems/Common/ROSUnderlays/fs-system-26-test-build"
ASURT_VENV="$ASURT_ROOT/.venvs/carla"
source /opt/ros/humble/setup.bash
source "$ASURT_COMMON/install/local_setup.bash"
source "$ASURT_BRIDGE/install/local_setup.bash"
source "$ASURT_PERCEPTION/install/local_setup.bash"
export ASURT_PERCEPTION_ENV="$ASURT_PERCEPTION"
export PATH="$ASURT_VENV/bin:$PATH"
export PYTHONPATH="$ASURT_VENV/lib/python3.10/site-packages:$PYTHONPATH"
export ROS_DOMAIN_ID=46
export ROS_LOCALHOST_ONLY=1
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export PYTHONUNBUFFERED=1
exec "$@"
