from setuptools import setup

name = "asurt_stanley_control"
setup(name=name, version="0.1.0", packages=[name],
      data_files=[
          ("share/ament_index/resource_index/packages", ["resource/"+name]),
          ("share/"+name, ["package.xml"]),
      ],
      install_requires=["setuptools"], zip_safe=True,
      maintainer="ASURT", maintainer_email="asurt@example.invalid",
      description="Independent Stanley control with curvature-aware speed",
      license="MIT",
      entry_points={"console_scripts": [
          "stanley_control_node = asurt_stanley_control.node:main",
      ]})
