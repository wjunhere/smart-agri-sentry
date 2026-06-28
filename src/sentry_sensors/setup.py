from setuptools import find_packages, setup

package_name = 'sentry_sensors'

setup(
    name=package_name,
    version='0.2.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', [
            'launch/imu.launch.py',
            'launch/lora_bridge.launch.py',
        ]),
        ('share/' + package_name + '/config', ['config/imu.yaml', 'config/imu_filter_madgwick.yaml']),
        ('share/' + package_name + '/udev', ['udev/99-myimu.rules']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='team',
    maintainer_email='team@example.com',
    description='Sensor bridge nodes for Smart Agri Sentry v2.0',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'uart_bridge_node = sentry_sensors.uart_bridge_node:main',
            'imu_node = sentry_sensors.imu_node:main',
            'lora_bridge_node = sentry_sensors.lora_bridge_node:main',
        ],
    },
)
