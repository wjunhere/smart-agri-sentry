from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'sentry_bringup'

setup(
    name=package_name,
    version='0.2.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', [
            '../../config/crop_profiles.yaml',
            '../../config/mission_params.yaml',
        ]),
        ('share/' + package_name + '/urdf', [
            'urdf/sentry.urdf',
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='team',
    maintainer_email='team@example.com',
    description='ROS2 nodes for Smart Agri Sentry RDK X5',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gps_node = sentry_bringup.gps_node:main',
            'camera_node = sentry_bringup.camera_node:main',
            'mipi_camera_node = sentry_bringup.mipi_camera_node:main',
        ],
    },
)
