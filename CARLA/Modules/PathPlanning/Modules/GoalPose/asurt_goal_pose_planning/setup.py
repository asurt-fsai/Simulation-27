from setuptools import find_packages, setup

name = "asurt_goal_pose_planning"
setup(
    name=name, version="0.1.0", packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/"+name]),
        ("share/"+name, ["package.xml"]),
    ],
    install_requires=["setuptools"], zip_safe=True,
    maintainer="ASURT", maintainer_email="asurt@example.invalid",
    description="RViz clicked goal to forward-only Dubins path",
    license="MIT",
    entry_points={"console_scripts": [
        "goal_pose_planner_node = asurt_goal_pose_planning.node:main",
    ]},
)
