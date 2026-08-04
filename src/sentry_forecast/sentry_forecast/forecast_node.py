import math
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
    WeatherForecast,
)


# Fallback infection model when crop_profiles.yaml lacks the section.
DEFAULT_INFECTION_MODEL = {
    'temp_optimal': [15.0, 25.0],
    'temp_tolerance': [5.0, 35.0],
    'rh_onset': 50.0,
    'rh_full': 100.0,
    'lwd_base_hours': 6.0,
    'lwd_temp_correction': 1.0,
}


class TrendForecaster:
    """Linear trend helper — used for trend DIRECTION only.

    Risk values must not be linearly extrapolated: plant disease
    epidemics follow sigmoid (logistic/Gompertz) progress curves
    (van der Plank; Kato & Koizumi 1987). Future risk is derived from
    the infection model driven by the weather forecast instead.
    """

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


def lwd_threshold_at(infection_model, temp: float) -> float:
    """Temperature-corrected LWD requirement (Mills-table style)."""
    base = float(infection_model['lwd_base_hours'])
    corr = float(infection_model['lwd_temp_correction'])
    t_lo, t_hi = infection_model['temp_optimal']
    if t_lo <= temp <= t_hi or corr <= 1.0:
        return base
    dev = (t_lo - temp) if temp < t_lo else (temp - t_hi)
    k = min(3, math.ceil(dev / 5.0))
    return base * (corr ** k)


def future_infection_risk(infection_model, hours, prediction_hours,
                          lwd_now: float) -> float:
    """Future disease risk from forecast-driven infection conditions.

    Standard operational-DSS approach (Bastiaansen 1997, Acta Hort. 461;
    Tyson et al. 2017, Phytopathology 107): extrapolate the ENVIRONMENT
    (hours favorable for infection), then map to risk via the crop's
    infection model — never extrapolate the risk series itself.

    A forecast hour counts as favorable when temperature is inside the
    crop's tolerance range AND free moisture is likely
    (RH >= rh_onset or precipitation > 0.5 mm/h).
    """
    if not hours:
        return 0.0
    window = [h for h in hours if h["hour_offset"] <= prediction_hours]
    if not window:
        return 0.0

    tol_lo, tol_hi = infection_model['temp_tolerance']
    onset = float(infection_model['rh_onset'])
    favorable = [h for h in window
                 if tol_lo <= h["temp"] <= tol_hi
                 and (h["humidity"] >= onset or h["precipitation"] > 0.5)]
    if not favorable:
        return 0.0

    t_mean = sum(h["temp"] for h in favorable) / len(favorable)
    future_lwd = lwd_now + len(favorable)
    threshold = lwd_threshold_at(infection_model, t_mean)
    return min(1.0, future_lwd / threshold) if threshold > 0 else 0.0


# Disaster warning level -> risk increment. Graded instead of a flat cap
# (blue/yellow/orange/red per CMA warning levels).
_DISASTER_LEVELS = {'红': 0.3, '橙': 0.25, '黄': 0.2, '蓝': 0.1}


def disaster_factor(disaster_alerts):
    """Risk increment from disaster warnings, graded by warning level."""
    if not disaster_alerts:
        return 0.0
    best = 0.0
    for alert in disaster_alerts:
        for kw, value in _DISASTER_LEVELS.items():
            if kw in alert:
                best = max(best, value)
    return best if best > 0.0 else 0.15


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
        self.declare_parameter('env_stale_sec', 180.0)
        self.declare_parameter('fusion_stale_sec', 30.0)

        self.crop_type = self.get_parameter('crop_type').value
        self.env_stale_sec = self.get_parameter('env_stale_sec').value
        self.fusion_stale_sec = self.get_parameter('fusion_stale_sec').value

        self.profiles = self._load_profiles(
            self.get_parameter('crop_profiles_path').value)
        self.profile = self.profiles.get(self.crop_type, {})
        self.infection_model = self._load_infection_model(self.profile)
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
            Environment, '/sensor/environment_fixed', self.on_env, qos)

        self.sub_weather = self.create_subscription(
            WeatherForecast, '/weather/forecast', self.on_weather, 10)

        self.last_weather = None
        self.last_weather_ts = 0.0
        self.weather_stale_sec = self.params.get('weather_stale_sec', 21600)

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

    def _load_infection_model(self, profile):
        im = profile.get('infection_model')
        if not im:
            self.get_logger().warn(
                f'No infection_model for crop={self.crop_type}, '
                f'falling back to defaults')
            return dict(DEFAULT_INFECTION_MODEL)
        merged = dict(DEFAULT_INFECTION_MODEL)
        merged.update(im)
        return merged

    def on_fusion(self, msg: FusionResult):
        now = self.get_clock().now().nanoseconds / 1e9
        self.last_fusion = msg
        self.last_fusion_ts = now

    def on_env(self, msg: Environment):
        now = self.get_clock().now().nanoseconds / 1e9
        self.last_env = msg
        self.last_env_ts = now

        # Fixed-interval resampling: history tracks wall-clock cadence,
        # not the (bursty) environment message rate, so regression slopes
        # are not diluted by duplicate risk values.
        sample_interval = self.params.get('history_sample_sec', 300)
        if self.history and (
                now - self.history[-1]['timestamp']) < sample_interval:
            return
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

    def on_weather(self, msg: WeatherForecast):
        now = self.get_clock().now().nanoseconds / 1e9
        self.last_weather = {
            "hours": [{"hour_offset": h.hour_offset, "temp": h.temp,
                        "humidity": h.humidity, "precipitation": h.precipitation,
                        "wind_speed": h.wind_speed} for h in msg.hours],
            "disaster_alerts": list(msg.disaster_alerts),
        }
        self.last_weather_ts = now

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
            msg.alert_source = 'LOCAL'
            return msg

        if (now - self.last_fusion_ts) > self.fusion_stale_sec:
            msg.active = False
            msg.alert_type = ALERT_NONE
            msg.probability = 0.0
            msg.description = 'Fusion data stale'
            msg.alert_source = 'LOCAL'
            return msg

        prediction_hours = self.params.get('prediction_hours', 24)
        risk_threshold = self.params.get('risk_threshold', 0.7)
        lwd_margin = self.params.get('lwd_margin_hours', 2.0)
        hum_trend_th = self.params.get('humidity_trend_threshold', 0.3)
        lwd_threshold = float(self.infection_model['lwd_base_hours'])
        w_local = self.params.get('blend_weight_local', 0.4)
        w_weather = self.params.get('blend_weight_weather', 0.6)

        current_risk = float(self.last_fusion.risk_score)
        current_lwd = float(self.last_fusion.lwd_hours)

        # Trend slopes (direction only — never used for value extrapolation)
        humidity_slope = TrendForecaster.linear_trend(self.history, 'humidity')
        risk_slope = TrendForecaster.linear_trend(self.history, 'risk_score')

        # Weather-driven future risk via infection model
        weather_available = (self.last_weather is not None
                             and (now - self.last_weather_ts) <= self.weather_stale_sec)

        if weather_available:
            future_risk = future_infection_risk(
                self.infection_model, self.last_weather["hours"],
                prediction_hours, current_lwd)
            d_factor = disaster_factor(self.last_weather["disaster_alerts"])
            blended = max(current_risk,
                          w_local * current_risk + w_weather * future_risk)
            blended = max(0.0, min(1.0, blended + d_factor))
        else:
            future_risk = 0.0
            d_factor = 0.0
            blended = current_risk

        alert_type = ALERT_NONE
        description = '风险平稳，无需预警'
        alert_source = 'LOCAL'

        # Weather-driven alerts first (higher priority)
        if weather_available:
            if d_factor > 0:
                keywords = ["暴雨", "台风", "大风"]
                for kw in keywords:
                    if any(kw in a for a in self.last_weather["disaster_alerts"]):
                        alert_type = ALERT_STORM_WARNING
                        description = f'灾害预警: {self.last_weather["disaster_alerts"][0]}'
                        alert_source = 'WEATHER'
                        break

            if alert_type == ALERT_NONE:
                frost_hours = sum(1 for h in self.last_weather["hours"][:72]
                                  if h["temp"] < 5.0)
                if frost_hours > 0:
                    alert_type = ALERT_FROST_WARNING
                    description = f'未来3天有霜冻风险（{frost_hours}h < 5°C）'
                    alert_source = 'WEATHER'

            if alert_type == ALERT_NONE:
                heat_hours = sum(1 for h in self.last_weather["hours"][:72]
                                 if h["temp"] > 35.0)
                if heat_hours > 6:
                    alert_type = ALERT_HEAT_STRESS
                    description = f'未来3天持续高温 {heat_hours}h > 35°C'
                    alert_source = 'WEATHER'

        # Local/hybrid-driven alerts
        if alert_type == ALERT_NONE:
            if blended >= risk_threshold and risk_slope > 0:
                alert_type = ALERT_RISING_RISK
                description = (
                    f'预测 {prediction_hours}h 风险 {blended:.2f} '
                    f'(当前 {current_risk:.2f}, 天气推算 {future_risk:.2f})，'
                    f'呈上升趋势')
                alert_source = 'HYBRID' if weather_available else 'LOCAL'
            elif (current_lwd >= (lwd_threshold - lwd_margin)
                  and humidity_slope >= hum_trend_th):
                alert_type = ALERT_LATENT_OUTBREAK
                description = (
                    f'LWD 接近阈值 ({current_lwd:.1f}h / '
                    f'{lwd_threshold:.1f}h)，湿度持续上升')
                alert_source = 'LOCAL'
            elif (self.last_env is not None
                  and (now - self.last_env_ts) <= self.env_stale_sec
                  and self.last_env.air_humidity <= 40.0
                  and self.last_env.air_temp >= 30.0):
                alert_type = ALERT_DROUGHT_STRESS
                description = (
                    f'干旱胁迫：温度 {self.last_env.air_temp:.1f}C，'
                    f'湿度 {self.last_env.air_humidity:.1f}%')
                alert_source = 'LOCAL'

        msg.active = alert_type != ALERT_NONE
        msg.alert_type = alert_type
        msg.probability = float(blended)
        msg.description = description
        msg.alert_source = alert_source
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
