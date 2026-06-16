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

    mission_pkg = get_package_share_directory('sentry_mission')
    ekf_config = os.path.join(mission_pkg, 'config', 'ekf.yaml')
    nav2_config = os.path.join(mission_pkg, 'config', 'nav2_no_map.yaml')
    waypoints_config = os.path.join(mission_pkg, 'config', 'waypoints.yaml')

    servo_config = os.path.join(
        get_package_share_directory('sentry_servo'), 'config', 'servo_config.yaml')

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
            parameters=[{
                'uart_port': '/dev/ttyS2',
                'baudrate': 115200,
                'forward_servo_cmd': False,
            }],
            output='screen',
        ),

        # Direct RDK X5 PWM servo driver
        Node(
            package='sentry_servo',
            executable='servo_driver_node',
            name='servo_driver_node',
            parameters=[{'config_path': servo_config}],
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

        # Wheel odometry
        Node(
            package='sentry_mission',
            executable='wheel_odom_node',
            name='wheel_odom_node',
            parameters=[{
                'wheel_base': 0.4,
                'pulses_per_meter': 1000,
            }],
            output='screen',
        ),

        # EKF fusion
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter',
            output='screen',
            parameters=[ekf_config],
        ),

        # Nav2 bringup (mapless mode)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('nav2_bringup'),
                    'launch', 'bringup_launch.py')
            ),
            launch_arguments={
                'params_file': nav2_config,
                'use_sim_time': 'False',
            }.items(),
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

        # Mission control (Nav2 + waypoint + visual servoing)
        Node(
            package='sentry_mission',
            executable='mission_control_node',
            name='mission_control_node',
            parameters=[mission_params_path, {
                'waypoints_file': waypoints_config,
                'wheel_base': 0.4,
                'pulses_per_meter': 1000,
            }],
            output='screen',
        ),

        # Web remote control
        Node(
            package='sentry_mission',
            executable='web_remote_node',
            name='web_remote_node',
            parameters=[{
                'max_linear': 0.5,
                'max_angular': 1.0,
            }],
            output='screen',
        ),

        # Phase 2: forecast + advisory + data logger
        Node(
            package='sentry_forecast',
            executable='forecast_node',
            name='forecast_node',
            parameters=[{
                'crop_type': LaunchConfiguration('crop_type'),
                'crop_profiles_path': crop_profiles_path,
                'forecast_params_path': os.path.join(config_dir, 'forecast_params.yaml'),
                'mobile_stale_sec': 2.0,
                'fusion_stale_sec': 30.0,
            }],
            output='screen',
        ),
        Node(
            package='sentry_advisory',
            executable='advisory_node',
            name='advisory_node',
            parameters=[{
                'crop_type': LaunchConfiguration('crop_type'),
                'advisory_rules_path': os.path.join(config_dir, 'advisory_rules.yaml'),
                'fusion_stale_sec': 30.0,
            }],
            output='screen',
        ),
        Node(
            package='sentry_data_logger',
            executable='data_logger_node',
            name='data_logger_node',
            parameters=[os.path.join(config_dir, 'data_logger_params.yaml')],
            output='screen',
        ),
    ])
