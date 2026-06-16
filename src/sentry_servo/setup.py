from setuptools import find_packages, setup

package_name = 'sentry_servo'

setup(
    name=package_name,
    version='0.2.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/servo_config.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='team',
    maintainer_email='team@example.com',
    description='Direct PWM servo driver for Smart Agri Sentry on RDK X5',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'servo_keyboard = sentry_servo.servo_keyboard:main',
            'servo_driver_node = sentry_servo.servo_driver_node:main',
        ],
    },
)
