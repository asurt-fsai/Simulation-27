from glob import glob
from os.path import join

from setuptools import setup

package_name = "asurt_zed2i_sim"
setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (join("share", package_name, "config"), glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="ASURT",
    maintainer_email="dev@asurt.example",
    description="ZED 2i-style stereo camera and ASURT legacy perception adapter for CARLA.",
    license="Apache-2.0",
    entry_points={"console_scripts": ["zed2i_sim_node = asurt_zed2i_sim.node:main"]},
)
