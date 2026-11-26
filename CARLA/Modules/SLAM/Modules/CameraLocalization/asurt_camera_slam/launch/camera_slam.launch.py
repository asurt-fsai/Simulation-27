from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import os


def generate_launch_description():
    params = os.path.join(
        get_package_share_directory('asurt_camera_slam'),
        'config', 'camera_slam.yaml')
    return LaunchDescription([
        Node(package='asurt_camera_slam', executable='camera_slam_node',
             name='asurt_camera_slam', output='screen', parameters=[params]),
    ])
