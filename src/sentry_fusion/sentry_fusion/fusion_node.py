import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
import yaml
import os

from sentry_interfaces.msg import (
    Diagnosis, Environment, FusionResult, SoilNutrition)
from .lwd_calculator import LWDCalculator, Phase


ALERT_NORMAL = 0
ALERT_SUSPICION = 1
ALERT_WARNING = 2
ALERT_CRITICAL = 3

ALERT_NAMES = {
    ALERT_NORMAL: 'NORMAL',
    ALERT_SUSPICION: 'SUSPICION',
    ALERT_WARNING: 'WARNING',
    ALERT_CRITICAL: 'CRITICAL',
}

MODE_VISION_DOMINANT = 'VISION_DOMINANT'
MODE_LATENT_SUSPICION = 'LATENT_SUSPICION'
MODE_HIGH_HUMIDITY_PATHOGEN = 'HIGH_HUMIDITY_PATHOGEN'
MODE_DROUGHT_STRESS = 'DROUGHT_STRESS'
MODE_BALANCED = 'BALANCED'


class FusionNode(Node):
    def __init__(self):
        super().__init__('fusion_node')
        self.declare_parameter('crop_type', 'tomato')
        self.declare_parameter('crop_profiles_path',
                               'config/crop_profiles.yaml')
        self.declare_parameter('mobile_stale_sec', 2.0)
        self.declare_parameter('fixed_env_window_sec', 10.0)

        self.crop_type = self.get_parameter('crop_type').value
        self.mobile_stale_sec = self.get_parameter('mobile_stale_sec').value
        self.fixed_env_window_sec = self.get_parameter(
            'fixed_env_window_sec').value

        # Load crop profiles
        profiles_path = self.get_parameter('crop_profiles_path').value
        self.profiles = self._load_profiles(profiles_path)
        self.profile = self.profiles.get(self.crop_type, {})

        # LWD calculator
        self.lwd_calc = LWDCalculator(
            window_hours=24, interval_minutes=5, warm_up_points=12)

        # State
        self.last_vision = None
        self.last_mobile_env = None
        self.last_mobile_ts = 0.0
        self.fixed_env_samples = []
        self.last_fusion_alert = ALERT_NORMAL
        self.last_fusion_mode = MODE_BALANCED

        # Hysteresis band (alert level changes only if delta exceeds this)
        self.hysteresis_delta = 0.15

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.sub_vision = self.create_subscription(
            Diagnosis, '/vision/diagnosis', self.on_vision, 10)
        self.sub_mobile = self.create_subscription(
            Environment, '/sensor/environment_mobile', self.on_mobile_env, qos)
        self.sub_fixed = self.create_subscription(
            Environment, '/sensor/environment_fixed', self.on_fixed_env, qos)

        self.pub_fusion = self.create_publisher(
            FusionResult, '/fusion/diagnosis', 10)

        self.timer = self.create_timer(1.0, self.tick)
        self.get_logger().info(
            f'Fusion node ready (crop={self.crop_type})')

    def _load_profiles(self, path: str) -> dict:
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
        return self._default_profiles()

    def _default_profiles(self) -> dict:
        return {
            'tomato': {
                'lwd_threshold_hours': 6.0,
                'high_humidity_threshold': 85.0,
                'drought_threshold': 30.0,
                'weights': {'vision': 0.5, 'env': 0.3, 'interaction': 0.2},
                'disease_risk_classes': [
                    'late_blight', 'healthy', 'early_blight',
                    'bacterial_spot', 'leaf_mold', 'septoria_leaf_spot',
                    'tomato_yellow_leaf_curl_virus'
                ],
            },
            'wheat': {
                'lwd_threshold_hours': 8.0,
                'high_humidity_threshold': 80.0,
                'drought_threshold': 25.0,
                'weights': {'vision': 0.5, 'env': 0.3, 'interaction': 0.2},
                'disease_risk_classes': [
                    'healthy', 'wheat_powdery_mildew', 'wheat_scab',
                    'wheat_stripe_rust', 'wheat_yellow_dwarf'
                ],
            },
            'strawberry': {
                'lwd_threshold_hours': 10.0,
                'high_humidity_threshold': 85.0,
                'drought_threshold': 35.0,
                'weights': {'vision': 0.5, 'env': 0.3, 'interaction': 0.2},
                'disease_risk_classes': [
                    'leaf_spot', 'powdery_mildew_leaf', 'gray_mold',
                    'angular_leaf_spot', 'blossom_blight',
                    'powdery_mildew_fruit', 'anthracnose_fruit_rot', 'healthy'
                ],
            },
        }

    def on_vision(self, msg: Diagnosis):
        self.last_vision = msg

    def on_mobile_env(self, msg: Environment):
        now = self.get_clock().now().nanoseconds / 1e9
        self.last_mobile_env = msg
        self.last_mobile_ts = now
        self.lwd_calc.update(
            msg.air_temp, msg.air_humidity, msg.soil_humidity,
            msg.leaf_wetness, now)

    def on_fixed_env(self, msg: Environment):
        now = self.get_clock().now().nanoseconds / 1e9
        self.fixed_env_samples.append((now, msg))
        # Prune old samples
        cutoff = now - self.fixed_env_window_sec
        self.fixed_env_samples = [
            (t, m) for t, m in self.fixed_env_samples if t > cutoff]

    def tick(self):
        result = self._fuse()
        self.pub_fusion.publish(result)

    def _fuse(self) -> FusionResult:
        now = self.get_clock().now().nanoseconds / 1e9
        msg = FusionResult()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'fusion'

        profile = self.profile
        w = profile.get('weights', {'vision': 0.5, 'env': 0.3,
                                      'interaction': 0.2})
        lwd_threshold = profile.get('lwd_threshold_hours', 6.0)
        high_humidity_th = profile.get('high_humidity_threshold', 85.0)
        drought_th = profile.get('drought_threshold', 30.0)
        risk_classes = profile.get('disease_risk_classes', [])

        # --- Vision component ---
        p_vis = 0.0
        vision_class = 'none'
        vision_conf = 0.0
        if self.last_vision is not None:
            vision_class = self.last_vision.disease_class
            vision_conf = self.last_vision.confidence
            if vision_class != 'healthy' and vision_class in risk_classes:
                p_vis = vision_conf
            elif vision_class != 'healthy':
                p_vis = vision_conf * 0.5  # Unknown disease, moderate weight

        # --- Environment component ---
        env = self._effective_env(now)
        e_norm = 0.0
        trend = self.lwd_calc.recent_trend()
        lwd = self.lwd_calc.lwd_hours
        phase = self.lwd_calc.phase

        if env is not None:
            humi = env.air_humidity
            temp = env.air_temp
            # Normalize humidity risk (0-100 -> 0-1)
            humi_risk = max(0.0, min(1.0, (humi - 50.0) / 50.0))
            # Temperature factor: pathogens favor 15-25C
            temp_factor = 1.0
            if 15.0 <= temp <= 25.0:
                temp_factor = 1.0
            elif temp < 5.0 or temp > 35.0:
                temp_factor = 0.3
            else:
                temp_factor = 0.7
            # LWD factor
            lwd_factor = min(1.0, lwd / lwd_threshold) if lwd_threshold > 0 else 0.0
            e_norm = (humi_risk * 0.4 + temp_factor * 0.3
                      + lwd_factor * 0.3)

        # --- Interaction term ---
        interaction = p_vis * e_norm

        # --- Stale detection: boost vision weight if mobile env is stale ---
        mobile_stale = (now - self.last_mobile_ts) > self.mobile_stale_sec
        w_v = w['vision'] * (1.3 if mobile_stale else 1.0)
        w_e = w['env'] * (0.7 if mobile_stale else 1.0)
        w_i = w['interaction']
        total_w = w_v + w_e + w_i
        w_v /= total_w
        w_e /= total_w
        w_i /= total_w

        # --- Risk score ---
        trend_factor = 1.0 + 0.2 * max(0.0, trend)  # rising humidity amplifies
        risk = (w_v * p_vis
                + w_e * e_norm * trend_factor
                + w_i * interaction)

        # Cold-start penalty on confidence
        confidence = 1.0
        if phase == Phase.COLD_BOOT:
            confidence = 0.3
        elif phase == Phase.WARM_UP:
            confidence = 0.5 + 0.5 * self.lwd_calc.fill_ratio

        # --- Gating: determine fusion mode ---
        mode = MODE_BALANCED
        if p_vis > 0.7:
            mode = MODE_VISION_DOMINANT
        elif p_vis > 0.3 and e_norm < 0.3:
            mode = MODE_LATENT_SUSPICION
        elif e_norm > 0.6 and humi > high_humidity_th:
            mode = MODE_HIGH_HUMIDITY_PATHOGEN
        elif env is not None and env.soil_humidity < drought_th:
            mode = MODE_DROUGHT_STRESS

        # --- Alert level with hysteresis ---
        alert = self._alert_level(risk)
        # Only change if delta is significant
        if abs(risk - self._risk_from_alert(self.last_fusion_alert)) > self.hysteresis_delta:
            self.last_fusion_alert = alert
        else:
            alert = self.last_fusion_alert
        self.last_fusion_mode = mode

        # --- Evidence chain ---
        evidence = []
        if vision_class != 'none':
            evidence.append(
                f'Vision: {vision_class} (conf={vision_conf:.2f})')
        if env is not None:
            evidence.append(
                f'Env: T={env.air_temp:.1f}C H={env.air_humidity:.1f}%')
        evidence.append(f'LWD={lwd:.1f}h (phase={phase.value})')
        evidence.append(f'Mode={mode}')
        if mobile_stale:
            evidence.append('Mobile env stale: vision weight boosted')

        msg.risk_score = float(risk)
        msg.alert_level = alert
        msg.mode = mode
        msg.evidence_chain = evidence
        msg.lwd_hours = float(lwd)
        msg.confidence = float(confidence)
        return msg

    def _effective_env(self, now: float):
        """Return averaged environment from fixed + mobile sensors."""
        sources = []
        if self.last_mobile_env is not None and (
                now - self.last_mobile_ts) <= self.mobile_stale_sec:
            sources.append(self.last_mobile_env)
        if self.fixed_env_samples:
            # Average fixed env samples
            avg = Environment()
            n = len(self.fixed_env_samples)
            avg.air_temp = sum(m.air_temp for _, m in self.fixed_env_samples) / n
            avg.air_humidity = sum(m.air_humidity for _, m in self.fixed_env_samples) / n
            avg.air_co2 = sum(m.air_co2 for _, m in self.fixed_env_samples) / n
            avg.soil_temp = sum(m.soil_temp for _, m in self.fixed_env_samples) / n
            avg.soil_humidity = sum(m.soil_humidity for _, m in self.fixed_env_samples) / n
            avg.leaf_wetness = sum(m.leaf_wetness for _, m in self.fixed_env_samples) / n
            avg.data_source = 'FIXED_AVG'
            sources.append(avg)
        if not sources:
            return None
        if len(sources) == 1:
            return sources[0]
        # Blend mobile and fixed
        blended = Environment()
        blended.air_temp = sum(s.air_temp for s in sources) / len(sources)
        blended.air_humidity = sum(s.air_humidity for s in sources) / len(sources)
        blended.air_co2 = sum(s.air_co2 for s in sources) / len(sources)
        blended.soil_temp = sum(s.soil_temp for s in sources) / len(sources)
        blended.soil_humidity = sum(s.soil_humidity for s in sources) / len(sources)
        blended.leaf_wetness = sum(s.leaf_wetness for s in sources) / len(sources)
        blended.data_source = 'BLENDED'
        return blended

    def _alert_level(self, risk: float) -> int:
        if risk >= 0.8:
            return ALERT_CRITICAL
        if risk >= 0.6:
            return ALERT_WARNING
        if risk >= 0.35:
            return ALERT_SUSPICION
        return ALERT_NORMAL

    def _risk_from_alert(self, alert: int) -> float:
        return {ALERT_NORMAL: 0.15, ALERT_SUSPICION: 0.45,
                ALERT_WARNING: 0.7, ALERT_CRITICAL: 0.9}.get(alert, 0.0)


def main(args=None):
    rclpy.init(args=args)
    node = FusionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
