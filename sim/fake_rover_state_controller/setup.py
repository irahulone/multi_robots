from setuptools import find_packages, setup

package_name = 'fake_rover_state_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='vboxuser',
    maintainer_email='vboxuser@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'rover_sim = fake_rover_state_controller.rover_sim:main',
            'fake_rover = fake_rover_state_controller.fake_rover:main',
            'jointstate_publisher = fake_rover_state_controller.jointstate_publisher:main'
        ],
    },
)
