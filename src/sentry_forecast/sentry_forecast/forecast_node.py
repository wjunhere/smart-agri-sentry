import os
import time
import yaml

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from sentry_interfaces.msg import (
    Environment,
    ForecastAlert,
    FusionResult,
)


class TrendForecaster:
    """Simple linear trend extrapolation helper."""

    @staticmethod
    def linear_trend(samples, key='risk_score'):
        if len(samples) < 2:
            return 0.0
        x = [(s['timestamp'] - samples[0]['timestamp']) / 3600.0
             for s in samples]
        y = [s[key] for s in samples]
        mean_x = sum(x) / len(x)
        mean_y = sum(y) / len(y)
        num = sum((xi - mean_x) * (yi - mean_y) for xi, yi in zip(x, y))
        den = sum((xi - mean_x) ** 2 for xi in x)
        if den == 0.0:
            return 0.0
        return num / den

    @staticmethod
    def predict(samples, prediction_hours, key='risk_score'):
        if not samples:
            return 0.0
        if len(samples) < 2:
            return float(samples[-1][key])
        slope = TrendForecaster.linear_trend(samples, key)
        last = samples[-1][key]
        return max(0.0, min(1.0, last + slope * prediction_hours))


def weather_risk_model(hours, prediction_hours):
    """Compute disease risk (0-1) from hourly weather forecast data."""
    if not hours:
        return 0.0
    window = [h for h in hours if h["hour_offset"] <= prediction_hours]
    if not window:
        return 0.0

    risk = 0.0
    # Fungal window: RH > 85% and 15 < T < 25, sustained > 6h
    fungal_hours = sum(1 for h in window
                       if h["humidity"] > 85.0 and 15.0 < h["temp"] < 25.0)
    if fungal_hours > 6:
        risk += min(0.4, 0.2 + 0.033 * (fungal_hours - 6))

    # Continuous rain > 1mm/h for > 12h
    rain_hours = 0
    for h in window:
        if h["precipitation"] > 1.0:
            rain_hours += 1
        else:
            rain_hours = 0
    if rain_hours > 12:
        risk += min(0.3, 0.2 + 0.008 * (rain_hours - 12))

    # Consecutive rain days > 2
    rain_days = 0
    for day_offset in range(7):
        day_hours = [h for h in hours if day_offset * 24 <= h["hour_offset"] < (day_offset + 1) * 24]
        if day_hours and sum(h["precipitation"] for h in day_hours) > 1.0:
            rain_days += 1
        else:
            rain_days = 0
    if rain_days > 2:
        risk += min(0.2, 0.1 * (rain_days - 2))

    # Heat stress: T > 35 sustained > 6h
    heat_hours = sum(1 for h in window if h["temp"] > 35.0)
    if heat_hours > 6:
        risk += min(0.3, 0.2 + 0.016 * (heat_hours - 6))

    # Frost risk: T < 5 present
    frost_hours = sum(1 for h in window if h["temp"] < 5.0)
    if frost_hours > 0:
        risk += min(0.4, 0.2 + 0.05 * frost_hours)

    return min(1.0, risk)


def disaster_factor(disaster_alerts):
    """Boost risk based on disaster warnings (0 to 0.3)."""
    if not disaster_alerts:
        return 0.0
    return 0.3


ALERT_NONE = 'NONE'
ALERT_RISING_RISK = 'RISING_RISK'
ALERT_LATENT_OUTBREAK = 'LATENT_OUTBREAK'
ALERT_DROUGHT_STRESS = 'DROUGHT_STRESS'
ALERT_STORM_WARNING = 'STORM_WARNING'
ALERT_FROST_WARNING = 'FROST_WARNING'
ALERT_HEAT_STRESS = 'HEAT_STRESS'


class ForecastNode(Node):
    def __init__(self):
        super().__init__('forecast_node')
        self.declare_parameter('crop_type', 'tomato')
        self.declare_parameter(
            'crop_profiles_path', 'config/crop_profiles.yaml')
        self.declare_parameter(
            'forecast_params_path', 'config/forecast_params.yaml')
        self.declare_parameter('mobile_stale_sec', 2.0)
        self.declare_parameter('fusion_stale_sec', 30.0)

        self.crop_type = self.get_parameter('crop_type').value
        self.mobile_stale_sec = self.get_parameter('mobile_stale_sec').value
        self.fusion_stale_sec = self.get_parameter('fusion_stale_sec').value

        self.profiles = self._load_profiles(
            self.get_parameter('crop_profiles_path').value)
        self.profile = self.profiles.get(self.crop_type, {})
        self.params = self._load_params(
            self.get_parameter('forecast_params_path').value)

        self.history = []
        self.last_fusion = None
        self.last_fusion_ts = 0.0
        self.last_env = None
        self.last_env_ts = 0.0

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.sub_fusion = self.create_subscription(
            FusionResult, '/fusion/diagnosis', self.on_fusion, 10)
        self.sub_env = self.create_subscription(
            Environment, '/sensor/environment_mobile', self.on_env, qos)

        period = self.params.get('timer_period_sec', 600)
        self.timer = self.create_timer(float(period), self.tick)
        self.pub = self.create_publisher(
            ForecastAlert, '/forecast/alert', 10)

        self.get_logger().info(
            f'Forecast node ready (crop={self.crop_type})')

    def _load_profiles(self, path):
        if not os.path.isabs(path):
            ws = os.environ.get('COLCON_PREFIX_PATH', os.getcwd())
            candidates = [
                os.path.join(ws, '..', '..', path),
                os.path.join(ws, path),
                path,
            ]
            for c in candidates:
                if os.path.exists(c):
                    path = c
                    break
        if os.path.exists(path):
            with open(path, 'r') as f:
                return yaml.safe_load(f) or {}
        self.get_logger().warn(f'Crop profile not found: {path}, using defaults')
        return {}

    def _load_params(self, path):
        if not os.path.isabs(path):
            ws = os.environ.get('COLCON_PREFIX_PATH', os.getcwd())
            candidates = [
                os.path.join(ws, '..', '..', path),
                os.path.join(ws, path),
                path,
            ]
            for c in candidates:
                if os.path.exists(c):
                    path = c
                    break
        if os.path.exists(path):
            with open(path, 'r') as f:
                data = yaml.safe_load(f) or {}
            return data.get('forecast_node', data)
        self.get_logger().warn(f'Forecast params not found: {path}, using defaults')
        return {}

    def on_fusion(self, msg: FusionResult):
        now = self.get_clock().now().nanoseconds / 1e9
        self.last_fusion = msg
        self.last_fusion_ts = now

    def on_env(self, msg: Environment):
        now = self.get_clock().now().nanoseconds / 1e9
        self.last_env = msg
        self.last_env_ts = now

        sample = {
            'timestamp': now,
            'risk_score': (self.last_fusion.risk_score
                           if self.last_fusion is not None else 0.0),
            'humidity': msg.air_humidity,
            'lwd_hours': (self.last_fusion.lwd_hours
                          if self.last_fusion is not None else 0.0),
            'temperature': msg.air_temp,
        }
        self.history.append(sample)

    def tick(self):
        alert = self._predict_alert()
        self.pub.publish(alert)

    def _prune_history(self, now):
        window = self.params.get('history_hours', 6) * 3600.0
        cutoff = now - window
        self.history = [h for h in self.history if h['timestamp'] > cutoff]

    def _predict_alert(self) -> ForecastAlert:
        now = self.get_clock().now().nanoseconds / 1e9
        self._prune_history(now)

        msg = ForecastAlert()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'forecast'
        msg.hours_ahead = 24

        if self.last_fusion is None:
            msg.active = False
            msg.alert_type = ALERT_NONE
            msg.probability = 0.0
            msg.description = 'No fusion data yet'
            return msg

        if (now - self.last_fusion_ts) > self.fusion_stale_sec:
            msg.active = False
            msg.alert_type = ALERT_NONE
            msg.probability = 0.0
            msg.description = 'Fusion data stale'
            return msg

        prediction_hours = self.params.get('prediction_hours', 24)
        risk_threshold = self.params.get('risk_threshold', 0.7)
        lwd_margin = self.params.get('lwd_margin_hours', 2.0)
        hum_trend_th = self.params.get('humidity_trend_threshold', 0.3)
        lwd_threshold = self.profile.get('lwd_threshold_hours', 6.0)

        predicted_risk = TrendForecaster.predict(
            self.history, prediction_hours, 'risk_score')
        humidity_slope = TrendForecaster.linear_trend(self.history, 'humidity')

        alert_type = ALERT_NONE
        description = '风险平稳，无需预警'

        risk_slope = TrendForecaster.linear_trend(self.history, 'risk_score')
        if predicted_risk >= risk_threshold and risk_slope > 0:
            alert_type = ALERT_RISING_RISK
            description = f'预测 24h 风险 {predicted_risk:.2f}，呈上升趋势'
        elif (self.last_fusion.lwd_hours >= (lwd_threshold - lwd_margin)
              and humidity_slope >= hum_trend_th):
            alert_type = ALERT_LATENT_OUTBREAK
            description = (
                f'LWD 接近阈值 ({self.last_fusion.lwd_hours:.1f}h / '
                f'{lwd_threshold:.1f}h)，湿度持续上升')
        elif (self.last_env is not None
              and (now - self.last_env_ts) <= self.mobile_stale_sec
              and self.last_env.air_humidity <= 40.0
              and self.last_env.air_temp >= 30.0):
            alert_type = ALERT_DROUGHT_STRESS
            description = (
                f'干旱胁迫：温度 {self.last_env.air_temp:.1f}C，'
                f'湿度 {self.last_env.air_humidity:.1f}%')

        msg.active = alert_type != ALERT_NONE
        msg.alert_type = alert_type
        msg.probability = float(predicted_risk)
        msg.description = description
        return msg


def main(args=None):
    rclpy.init(args=args)
    node = ForecastNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
