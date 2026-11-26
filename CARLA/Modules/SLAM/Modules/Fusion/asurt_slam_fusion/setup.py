from setuptools import setup

package_name = 'asurt_slam_fusion'
setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/fusion.yaml']),
        ('share/' + package_name + '/launch', ['launch/fusion.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ASURT',
    maintainer_email='team@asu-racing.com',
    description='Independent ASURT camera + LiDAR landmark measurement and pose fusion',
    license='MIT',
    entry_points={'console_scripts': [
        'slam_fusion_node = asurt_slam_fusion.node:main',
    ]},
)
