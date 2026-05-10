from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'uav_controller'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'pymavlink'],
    zip_safe=True,
    maintainer='your name',
    maintainer_email='you@example.com',
    description='ROS2 UAV controller for ArduPilot SITL',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'drone_bridge     = uav_controller.drone_bridge:main',
            'takeoff_mission  = uav_controller.takeoff_mission:main',
            'l_mission       = uav_controller.l_mission:main',
            'multi_mission   = uav_controller.multi_mission:main',
            'faculty_mission  = uav_controller.faculty_mission:main',
            'airport_mission   = uav_controller.airport_mission:main',

        ],
    },
)

#tells python how to install the package and its dependencies, and defines the console scripts for running the nodes.