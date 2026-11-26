from setuptools import find_packages, setup

package_name = 'asurt_classical_planning'
setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ASURT',
    maintainer_email='asurt@example.invalid',
    description='Read-only cone centerline planning preview',
    license='MIT',
    entry_points={'console_scripts': ['classical_planner_node = asurt_classical_planning.node:main']},
)
