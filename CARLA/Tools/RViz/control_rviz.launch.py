"""Start Control RViz and keep its MPC annotation node scoped to the window."""

from pathlib import Path
import sys

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    rviz = ExecuteProcess(cmd=["rviz2", "-d", LaunchConfiguration("rviz_config"),
                              "--ros-args", "-p", "use_sim_time:=true"], output="screen")
    markers = ExecuteProcess(cmd=[sys.executable, str(Path(__file__).with_name("mpc_command_markers.py"))],
                             output="screen")
    return LaunchDescription([
        DeclareLaunchArgument("rviz_config"),
        RegisterEventHandler(OnProcessExit(target_action=rviz, on_exit=[EmitEvent(event=Shutdown())])),
        markers,
        rviz,
    ])
