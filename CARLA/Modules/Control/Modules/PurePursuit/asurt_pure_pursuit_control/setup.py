from setuptools import setup

name = "asurt_pure_pursuit_control"
setup(
    name=name,
    version="0.1.0",
    packages=[name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + name]),
        ("share/" + name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="ASURT Simulation",
    maintainer_email="asurt@example.invalid",
    description="Pure Pursuit for ASURT native CARLA ROS bridge",
    license="Apache-2.0",
    entry_points={"console_scripts": [
        "pure_pursuit_control_node = asurt_pure_pursuit_control.node:main",
    ]},
)
