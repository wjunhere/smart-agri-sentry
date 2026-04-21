from setuptools import find_packages, setup

package_name = 'sentry_bringup'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/sentry.launch.py']),
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
            'uart_bridge_node = sentry_bringup.uart_bridge_node:main',
            'gps_node = sentry_bringup.gps_node:main',
            'camera_node = sentry_bringup.camera_node:main',
            'ai_inference_node = sentry_bringup.ai_inference_node:main',
        ],
    },
)
