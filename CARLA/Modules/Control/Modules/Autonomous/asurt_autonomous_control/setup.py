from setuptools import setup

PACKAGE = "asurt_autonomous_control"
setup(
    name=PACKAGE,
    version="0.1.0",
    packages=[PACKAGE],
    data_files=[("share/ament_index/resource_index/packages", [f"resource/{PACKAGE}"]),
                (f"share/{PACKAGE}", ["package.xml"])],
    maintainer="ASURT",
    maintainer_email="team@asu-racing.com",
    license="MIT",
    description="CARLA path following with the existing ASURT controller algorithms.",
    entry_points={"console_scripts": [
        "pure_pursuit_auto_node = asurt_autonomous_control.node:pure_pursuit_main",
        "stanley_auto_node = asurt_autonomous_control.node:stanley_main",
    ]},
)
