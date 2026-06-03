from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_dir = get_package_share_directory('sentry_bringup')
    config_dir = os.path.join(pkg_dir, '..', '..', '..', 'config')
    if not os.path.exists(config_dir):
        config_dir = os.path.join(os.getcwd(), 'config')

    crop_profiles_path = os.path.join(config_dir, 'crop_profiles.yaml')
    mission_params_path = os.path.join(config_dir, 'mission_params.yaml')

    lidar_launch_path = os.path.join(
        get_package_share_directory('sentry_lidar'), 'launch', 'stl19p.launch.py')

    imu_launch_path = os.path.join(
        get_package_share_directory('sentry_sensors'), 'launch', 'imu.launch.py')

    return LaunchDescription([
        DeclareLaunchArgument('crop_type', default_value='tomato'),
        DeclareLaunchArgument('use_sim_plant', default_value='false'),

        # Vision nodes
        Node(
            package='sentry_bringup',
            executable='camera_node',
            name='camera_node',
            parameters=[{'device_id': 0, 'fps': 2.0}],
            output='screen',
        ),
        Node(
            package='sentry_vision',
            executable='vision_diagnosis_node',
            name='vision_diagnosis_node',
            parameters=[{
                'crop_type': LaunchConfiguration('crop_type'),
                'model_path': '',
                'input_size': 224,
            }],
            output='screen',
        ),
        Node(
            package='sentry_vision',
            executable='plant_detector_node',
            name='plant_detector_node',
            parameters=[{
                'confidence_threshold': 0.6,
                'min_area_ratio': 0.1,
                'use_simulation': LaunchConfiguration('use_sim_plant'),
            }],
            output='screen',
        ),

        # Sensor bridge
        Node(
            package='sentry_sensors',
            executable='uart_bridge_node',
            name='uart_bridge_node',
            parameters=[{'uart_port': '/dev/ttyS2', 'baudrate': 115200}],
            output='screen',
        ),

        # LiDAR
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(lidar_launch_path)
        ),

        # IMU
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(imu_launch_path)
        ),

        # Fusion node
        Node(
            package='sentry_fusion',
            executable='fusion_node',
            name='fusion_node',
            parameters=[{
                'crop_type': LaunchConfiguration('crop_type'),
                'crop_profiles_path': crop_profiles_path,
                'mobile_stale_sec': 2.0,
                'fixed_env_window_sec': 10.0,
            }],
            output='screen',
        ),

        # Mission control
        Node(
            package='sentry_mission',
            executable='mission_control_node',
            name='mission_control_node',
            parameters=[mission_params_path],
            output='screen',
        ),
    ])
