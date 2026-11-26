from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from pathlib import Path


def generate_launch_description():
    config = str(Path(get_package_share_directory("asurt_velodyne_sim")) / "config" / "vlp32c.yaml")
    return LaunchDescription([
        Node(
            package="asurt_velodyne_sim",
            executable="velodyne_sim_node",
            name="asurt_velodyne_sim_node",
            output="screen",
            parameters=[config, {"use_sim_time": True}],
        ),
    ])
