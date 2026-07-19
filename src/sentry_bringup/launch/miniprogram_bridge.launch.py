"""Gateway layer launch: miniprogram bridge + web remote + weather + LLM advisor.

Started at boot by systemd sentry-bridge.service (scripts/rdk/install_autostart.sh).
Only lightweight gateway nodes run here; heavy work nodes (camera/Nav2/mission)
are started on demand via POST /stack/* -> start_robot_stack.sh.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    weather_params = os.path.join(
        get_package_share_directory('sentry_weather'),
        'config', 'weather_params.yaml')

    llm_api_key = (
        os.environ.get('SENTRY_LLM_API_KEY')
        or os.environ.get('DEEPSEEK_API_KEY', ''))

    return LaunchDescription([
        Node(
            package='sentry_miniprogram',
            executable='miniprogram_bridge_node',
            name='miniprogram_bridge_node',
            output='screen',
            parameters=[],
        ),
        Node(
            package='sentry_mission',
            executable='web_remote_node',
            name='web_remote_node',
            output='screen',
        ),
        Node(
            package='sentry_weather',
            executable='weather_node',
            name='weather_node',
            output='screen',
            parameters=[weather_params],
        ),
        Node(
            package='sentry_llm',
            executable='llm_advisor_node',
            name='llm_advisor_node',
            output='screen',
            parameters=[{
                'api_key': llm_api_key,
                'auto_period_sec': 600,
            }],
        ),
    ])
