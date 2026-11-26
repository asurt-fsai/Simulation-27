from glob import glob
from os.path import join
from setuptools import setup

name = "asurt_lidar_perception"
setup(
    name=name,
    version="0.1.0",
    packages=[name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + name]),
        ("share/" + name, ["package.xml"]),
        (join("share", name, "config"), glob("config/*.yaml")),
        (join("share", name, "launch"), glob("launch/*.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="ASURT",
    maintainer_email="dev@asurt.example",
    description="Swappable efficient cone pipeline for /velodyne_points",
    license="Apache-2.0",
    entry_points={
        "console_scripts": ["lidar_perception_node = asurt_lidar_perception.node:main"]
    },
)
