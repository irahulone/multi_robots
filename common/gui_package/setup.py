from setuptools import setup, find_packages

package_name = 'gui_package'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rover4-i',
    maintainer_email='rover4-i@todo.todo',
    description='Common GUI and ROS2 utilities for teleoperation',
    license='ECL-2.0',
    tests_require=['pytest'],
)
