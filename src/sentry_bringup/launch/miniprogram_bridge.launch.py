"""Launch miniprogram_bridge_node + llm_advisor_node for WeChat mini-program."""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='sentry_miniprogram',
            executable='miniprogram_bridge_node',
            name='miniprogram_bridge_node',
            output='screen',
            parameters=[],
        ),
        Node(
            package='sentry_llm',
            executable='llm_advisor_node',
            name='llm_advisor_node',
            output='screen',
            parameters=[{
                'api_key': '',
                'auto_period_sec': 600,
            }],
        ),
    ])
