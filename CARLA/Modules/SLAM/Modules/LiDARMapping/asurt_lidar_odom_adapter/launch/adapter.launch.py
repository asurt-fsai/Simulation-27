from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import os


def generate_launch_description():
    path = os.path.join(get_package_share_directory(
        'asurt_lidar_odom_adapter'), 'config', 'adapter.yaml')
    return LaunchDescription([
        Node(package='asurt_lidar_odom_adapter',
             executable='lidar_odom_adapter_node',
             name='asurt_lidar_odom_adapter', output='screen',
             parameters=[path]),
    ])
