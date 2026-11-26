from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    settings = str(Path(get_package_share_directory("asurt_lidar_perception"))
                   / "config" / "legacy_fast.yaml")
    return LaunchDescription([
        Node(
            package="asurt_lidar_perception",
            executable="lidar_perception_node",
            name="asurt_lidar_perception_node",
            output="screen",
            parameters=[settings, {"use_sim_time": True}],
        )
    ])
