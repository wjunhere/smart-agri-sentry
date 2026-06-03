from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('sentry_lidar')
    config_path = os.path.join(pkg_share, 'config', 'stl19p.yaml')

    return LaunchDescription([
        Node(
            package='sentry_lidar',
            executable='sentry_lidar',
            name='sentry_lidar',
            output='screen',
            parameters=[config_path],
        ),
    ])
