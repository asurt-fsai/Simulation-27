"""Launch the historical box-filter LiDAR profile independently of the sensor."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    params = str(
        Path(get_package_share_directory("asurt_lidar_perception"))
        / "config" / "legacy_box.yaml"
    )
    return LaunchDescription([
        Node(
            package="asurt_lidar_perception",
            executable="lidar_perception_node",
            name="asurt_lidar_perception_node",
            output="screen",
            parameters=[params, {"use_sim_time": True}],
        ),
    ])
