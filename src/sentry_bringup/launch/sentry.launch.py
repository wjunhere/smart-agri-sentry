from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='sentry_bringup',
            executable='uart_bridge_node',
            name='uart_bridge_node',
            parameters=[{'uart_port': '/dev/ttyS2', 'baudrate': 115200}],
            output='screen',
        ),
        Node(
            package='sentry_bringup',
            executable='gps_node',
            name='gps_node',
            parameters=[{'uart_port': '/dev/ttyS6', 'baudrate': 9600}],
            output='screen',
        ),
        Node(
            package='sentry_bringup',
            executable='camera_node',
            name='camera_node',
            parameters=[{'device_id': 0, 'fps': 2.0}],
            output='screen',
        ),
        Node(
            package='sentry_bringup',
            executable='ai_inference_node',
            name='ai_inference_node',
            parameters=[{'model_path': 'models/finetuned_mobilenetv2_int8.tflite', 'input_size': 224}],
            output='screen',
        ),
    ])
