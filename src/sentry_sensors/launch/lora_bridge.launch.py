from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node


def generate_launch_description():
    use_mock = LaunchConfiguration('use_mock', default='true')
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_mock', default_value='true',
            description='Use mock/synthetic sensor data instead of real LoRa UART'),
        Node(
            package='sentry_sensors',
            executable='lora_bridge_node',
            name='lora_bridge_node',
            parameters=[{
                'uart_port': '/dev/ttyACM0',
                'baudrate': 9600,
                'use_mock': use_mock,
            }],
            output='screen',
        ),
    ])
