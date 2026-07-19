from setuptools import find_packages, setup
import glob

package_name = 'sentry_mission'

setup(
    name=package_name,
    version='0.2.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', [
            'config/ekf.yaml',
            'config/mission_params.yaml',
            'config/nav2_no_map.yaml',
            'config/waypoints.yaml',
        ]),
        ('share/' + package_name + '/static', [
            'static/index.html',
        ]),
        ('share/' + package_name + '/static_v2', [
            'static_v2/index.html',
        ]),
        ('share/' + package_name + '/static_v2/components',
            glob.glob('static_v2/components/*.js')),
        ('share/' + package_name + '/static_v2',
            glob.glob('static_v2/*.js') + glob.glob('static_v2/*.css')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='team',
    maintainer_email='team@example.com',
    description='Mission control node for Smart Agri Sentry v2.0',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mission_control_node = sentry_mission.mission_control_node:main',
            'wheel_odom_node = sentry_mission.wheel_odom_node:main',
            'web_remote_node = sentry_mission.web_remote_node:main',
            'chassis_cmd = sentry_mission.chassis_cmd:main',
            'imu_turn = sentry_mission.imu_turn_controller:main',
            'keyboard_control = sentry_mission.keyboard_control_node:main',
        ],
    },
)
