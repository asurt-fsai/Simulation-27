from setuptools import setup

package_name = 'asurt_lidar_odom_adapter'
setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/adapter.yaml']),
        ('share/' + package_name + '/launch', ['launch/adapter.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ASURT',
    maintainer_email='team@asu-racing.com',
    description='Optional calibrated historical LeGO LiDAR odometry normalization',
    license='MIT',
    entry_points={'console_scripts': [
        'lidar_odom_adapter_node = asurt_lidar_odom_adapter.node:main',
    ]},
)
