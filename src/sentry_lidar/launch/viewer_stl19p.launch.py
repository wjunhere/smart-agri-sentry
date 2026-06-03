from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('sentry_lidar')
    stl19p_launch = os.path.join(pkg_share, 'launch', 'stl19p.launch.py')
    rviz_config = os.path.join(pkg_share, '..', '..', '..', '..', 'example', 'lidar',
                               'ldlidar_ros2', 'ldlidar', 'rviz2', 'ldlidar.rviz')

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(stl19p_launch)
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config] if os.path.exists(rviz_config) else [],
            output='screen',
        ),
    ])
