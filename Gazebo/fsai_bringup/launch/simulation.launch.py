import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, SetEnvironmentVariable, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    worlds_share = Path(get_package_share_directory("fsai_worlds"))
    description_share = Path(get_package_share_directory("fsai_description"))

    world = LaunchConfiguration("world")
    world_name = LaunchConfiguration("world_name")
    vehicle_file = str(description_share / "models" / "fsai_car" / "model.sdf")

    resource_parts = [str(worlds_share / "models"), str(description_share / "models")]
    if os.environ.get("GZ_SIM_RESOURCE_PATH"):
        resource_parts.append(os.environ["GZ_SIM_RESOURCE_PATH"])

    return LaunchDescription([
        DeclareLaunchArgument(
            "world",
            default_value=str(worlds_share / "worlds" / "empty.sdf"),
            description="Absolute path to the Gazebo SDF world.",
        ),
        DeclareLaunchArgument(
            "world_name",
            default_value="empty",
            description="SDF world name used by Gazebo services.",
        ),
        SetEnvironmentVariable("GZ_SIM_RESOURCE_PATH", os.pathsep.join(resource_parts)),
        ExecuteProcess(cmd=["gz", "sim", "-r", world], output="screen"),
        TimerAction(
            period=2.0,
            actions=[
                Node(
                    package="ros_gz_sim",
                    executable="create",
                    arguments=[
                        "-world", world_name,
                        "-file", vehicle_file,
                        "-name", "fsai_car",
                        "-x", "0.0",
                        "-y", "0.0",
                        "-z", "0.35",
                    ],
                    output="screen",
                )
            ],
        ),
        Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
            output="screen",
        ),
    ])
