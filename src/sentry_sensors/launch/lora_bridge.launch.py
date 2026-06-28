from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='sentry_sensors',
            executable='lora_bridge_node',
            name='lora_bridge_node',
            parameters=[{
                'uart_port': '/dev/ttyACM0',
                'baudrate': 9600,
            }],
            output='screen',
        ),
    ])
