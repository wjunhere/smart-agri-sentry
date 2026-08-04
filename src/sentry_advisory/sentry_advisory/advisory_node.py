import os

import rclpy
from rclpy.node import Node

from sentry_interfaces.msg import (
    AdvisoryAction,
    Environment,
    ForecastAlert,
    FusionResult,
    WeatherForecast,
)
from .rule_engine import RuleEngine


class AdvisoryNode(Node):
    def __init__(self):
        super().__init__('advisory_node')
        self.declare_parameter('crop_type', 'tomato')
        self.declare_parameter(
            'advisory_rules_path', 'config/advisory_rules.yaml')
        self.declare_parameter('fusion_stale_sec', 30.0)

        self.crop_type = self.get_parameter('crop_type').value
        self.fusion_stale_sec = self.get_parameter('fusion_stale_sec').value

        rules_path = self.get_parameter('advisory_rules_path').value
        self.engine = RuleEngine.from_yaml(rules_path)
        if not self.engine.rules:
            self.get_logger().warn(
                f'No advisory rules loaded from {rules_path}, using empty set')

        self.last_fusion = None
        self.last_fusion_ts = 0.0
        self.last_forecast = None
        self.last_env = None
        self.declare_parameter('weather_stale_sec', 21600)
        self.weather_stale_sec = self.get_parameter('weather_stale_sec').value

        self.last_weather = None
        self.last_weather_ts = 0.0

        self.sub_fusion = self.create_subscription(
            FusionResult, '/fusion/diagnosis', self.on_fusion, 10)
        self.sub_forecast = self.create_subscription(
            ForecastAlert, '/forecast/alert', self.on_forecast, 10)
        self.sub_env = self.create_subscription(
            Environment, '/sensor/environment_fixed', self.on_env, 10)
        self.sub_weather = self.create_subscription(
            WeatherForecast, '/weather/forecast', self.on_weather, 10)

        self.pub = self.create_publisher(
            AdvisoryAction, '/advisory/action', 10)

        self.get_logger().info(
            f'Advisory node ready (crop={self.crop_type})')

    def on_fusion(self, msg: FusionResult):
        self.last_fusion = msg
        self.last_fusion_ts = self.get_clock().now().nanoseconds / 1e9
        self._maybe_publish()

    def on_forecast(self, msg: ForecastAlert):
        self.last_forecast = msg
        self._maybe_publish()

    def on_env(self, msg: Environment):
        self.last_env = msg
        self._maybe_publish()

    def on_weather(self, msg: WeatherForecast):
        self.last_weather_ts = self.get_clock().now().nanoseconds / 1e9
        self.last_weather = {
            "hours": [{"hour_offset": h.hour_offset, "temp": h.temp,
                        "humidity": h.humidity, "precipitation": h.precipitation,
                        "wind_speed": h.wind_speed} for h in msg.hours],
            "disaster_alerts": list(msg.disaster_alerts),
        }
        self._maybe_publish()

    def _maybe_publish(self):
        if self.last_fusion is None:
            return
        now = self.get_clock().now().nanoseconds / 1e9
        if (now - self.last_fusion_ts) > self.fusion_stale_sec:
            return
        action = self._evaluate(
            self.last_fusion,
            self.last_forecast,
            self.last_env)
        self.pub.publish(action)

    def _evaluate(self, fusion, forecast, env):
        forecast = forecast or ForecastAlert()
        env = env or Environment()
        now = self.get_clock().now().nanoseconds / 1e9
        weather_stale = (
            self.last_weather is None
            or (now - self.last_weather_ts) > self.weather_stale_sec
        )
        weather_hours = (
            self.last_weather["hours"] if not weather_stale else None
        )
        disaster_alerts = (
            self.last_weather["disaster_alerts"] if not weather_stale else None
        )
        matched = self.engine.match(fusion, forecast, env, self.crop_type,
                                    weather_hours, disaster_alerts)

        msg = AdvisoryAction()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'advisory'
        msg.action_type = matched.get('action_type', 'NONE')
        msg.description = matched.get('description', '')
        msg.priority = matched.get('priority', 'LOW')
        msg.steps = matched.get('steps', [])
        return msg


def main(args=None):
    rclpy.init(args=args)
    node = AdvisoryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
