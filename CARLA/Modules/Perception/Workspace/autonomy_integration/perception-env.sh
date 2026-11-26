# Shared environment for the independently runnable PC perception modules.
WORKSPACE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PERCEPTION="$(cd -- "$WORKSPACE/../.." && pwd)"
ASURT="$(cd -- "$PERCEPTION/../../.." && pwd)"

if [[ "${ASURT_PERCEPTION_ENV:-}" != "$WORKSPACE" ]]; then
    source /opt/ros/humble/setup.bash
    source "$ASURT/Simulation/Systems/Common/ROSUnderlays/fs-system-26-test-build/install/local_setup.bash"
    source "$WORKSPACE/install/local_setup.bash"
    CARLA_VENV="$ASURT/.venvs/carla"
    export PATH="$CARLA_VENV/bin:$PATH"
    export PYTHONPATH="$CARLA_VENV/lib/python3.10/site-packages${PYTHONPATH:+:$PYTHONPATH}"
fi

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-46}"
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-1}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
