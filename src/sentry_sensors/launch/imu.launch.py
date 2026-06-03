from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('sentry_sensors')

    imu_config = os.path.join(pkg_share, 'config', 'imu.yaml')
    madgwick_config = os.path.join(
        pkg_share, 'config', 'imu_filter_madgwick.yaml'
    )

    imu_node = Node(
        package='sentry_sensors',
        executable='imu_node',
        name='imu_node',
        parameters=[imu_config],
        output='screen',
    )

    imu_filter_node = Node(
        package='imu_filter_madgwick',
        executable='imu_filter_madgwick_node',
        name='imu_filter_madgwick',
        parameters=[madgwick_config],
        remappings=[
            ('/imu/data_raw', '/sensor/imu/data_raw'),
            ('/imu/mag', '/sensor/imu/mag'),
            ('/imu/data', '/sensor/imu/data'),
        ],
        output='screen',
    )

    # Static TF: base_link -> imu_link
    # Adjust xyz/rpy if IMU is not mounted at robot center
    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='imu_static_tf',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'imu_link'],
    )

    return LaunchDescription([
        imu_node,
        imu_filter_node,
        static_tf_node,
    ])
