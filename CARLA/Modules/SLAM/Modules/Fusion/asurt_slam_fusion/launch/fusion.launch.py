from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    config = os.path.join(get_package_share_directory('asurt_slam_fusion'),
                          'config', 'fusion.yaml')
    return LaunchDescription([
        Node(package='asurt_slam_fusion', executable='slam_fusion_node',
             name='asurt_slam_fusion', output='screen', parameters=[config]),
    ])
