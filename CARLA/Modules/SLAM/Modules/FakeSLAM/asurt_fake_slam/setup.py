from setuptools import setup


package = "asurt_fake_slam"
setup(
    name=package,
    version="0.1.0",
    packages=[package],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package]),
        ("share/" + package, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    maintainer="ASURT",
    maintainer_email="team@asu-racing.com",
    description="Opt-in CARLA course map for testing planning and control",
    license="MIT",
    entry_points={"console_scripts": [
        "fake_slam_node = asurt_fake_slam.node:main",
        "fake_slam_observer_node = asurt_fake_slam.node:observer_main",
    ]},
)
