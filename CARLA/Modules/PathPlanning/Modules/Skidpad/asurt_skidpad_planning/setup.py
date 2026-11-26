from setuptools import find_packages, setup

name = 'asurt_skidpad_planning'
setup(name=name, version='0.1.0', packages=find_packages(),
      data_files=[('share/ament_index/resource_index/packages', ['resource/' + name]),
                  ('share/' + name, ['package.xml'])],
      install_requires=['setuptools'], zip_safe=True, maintainer='ASURT',
      maintainer_email='asurt@example.invalid',
      description='Read-only Formula Student skidpad mission path',
      license='MIT',
      entry_points={'console_scripts': ['skidpad_planner_node = asurt_skidpad_planning.node:main']})
