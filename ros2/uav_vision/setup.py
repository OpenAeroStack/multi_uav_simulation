from setuptools import setup

package_name = 'uav_vision'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    entry_points={
        'console_scripts': [
            'camera_relay = uav_vision.camera_relay:main',
            'gcs_receiver = uav_vision.gcs_receiver:main',
            'metrics_logger = uav_vision.metrics_logger:main',
            'detector       = uav_vision.detector:main',
        ],
    },
)
