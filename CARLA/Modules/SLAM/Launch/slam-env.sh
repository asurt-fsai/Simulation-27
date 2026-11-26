# Shared PC ROS environment for standalone, passive SLAM modules.
source /opt/ros/humble/setup.bash
asurt_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../../../.." && pwd)"
slam="$asurt_root/Simulation/Systems/SLAM"
source "$asurt_root/Simulation/Systems/Common/ROSUnderlays/fs-system-26-test-build/install/local_setup.bash"
source "$asurt_root/Simulation/Systems/Perception/Workspace/autonomy_integration/install/local_setup.bash"
slam_install="$slam/Workspace/slam_integration/install"
source "$slam_install/local_setup.bash"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-46}"
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-1}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
