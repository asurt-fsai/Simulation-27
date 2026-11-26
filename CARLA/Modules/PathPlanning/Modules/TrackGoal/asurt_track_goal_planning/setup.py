from setuptools import setup

name = "asurt_track_goal_planning"
setup(name=name, version="0.1.0", packages=[name],
      data_files=[
          ("share/ament_index/resource_index/packages", ["resource/"+name]),
          ("share/"+name, ["package.xml"]),
      ],
      install_requires=["setuptools"], zip_safe=True,
      maintainer="ASURT", maintainer_email="asurt@example.invalid",
      description="Manual RViz goal, SLAM cone corridor, obstacle-aware route",
      license="MIT",
      entry_points={"console_scripts": [
          "track_goal_planner_node = asurt_track_goal_planning.node:main",
      ]})
