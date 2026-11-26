from setuptools import setup

package_name = "asurt_cone_slam"
setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", ["config/slam.yaml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="ASURT",
    maintainer_email="team@asu-racing.com",
    description="Independent odometry-assisted landmark mapping using existing ASURT perception",
    license="MIT",
    entry_points={"console_scripts": [
        "cone_mapper_node = asurt_cone_slam.cone_mapper_node:main",
        "slam_trajectory_node = asurt_cone_slam.trajectory_node:main",
    ]},
)
