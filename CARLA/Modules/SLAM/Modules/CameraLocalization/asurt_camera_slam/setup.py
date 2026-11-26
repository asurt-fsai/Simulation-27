from setuptools import setup

name = 'asurt_camera_slam'

setup(
    name=name,
    version='0.1.0',
    packages=[name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + name]),
        ('share/' + name, ['package.xml']),
        ('share/' + name + '/config', ['config/camera_slam.yaml']),
        ('share/' + name + '/launch', ['launch/camera_slam.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ASURT',
    maintainer_email='team@asu-racing.com',
    description='Camera-cone-driven map-relative planar pose correction',
    license='MIT',
    entry_points={'console_scripts': [
        'camera_slam_node = asurt_camera_slam.node:main',
    ]},
)
