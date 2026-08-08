import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
import yaml
import os

from sentry_interfaces.msg import Diagnosis, Environment, FusionResult
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
MODE_UNKNOWN_DISEASE = 'UNKNOWN_DISEASE'
MODE_BALANCED = 'BALANCED'

# Fallback infection model when crop_profiles.yaml lacks the section.
# Mirrors pre-change behavior (global 15-25C window, linear humidity,
# fixed LWD threshold) so old configs keep working.
DEFAULT_INFECTION_MODEL = {
    'temp_optimal': [15.0, 25.0],
    'temp_tolerance': [5.0, 35.0],
    'rh_onset': 50.0,
    'rh_full': 100.0,
    'lwd_base_hours': 6.0,
    'lwd_temp_correction': 1.0,
}


class FusionNode(Node):
    def __init__(self):
        super().__init__('fusion_node')
        self.declare_parameter('crop_type', 'tomato')
        self.declare_parameter('crop_profiles_path',
                               'config/crop_profiles.yaml')
        # Fixed (LoRa) node is the only environment source; ~3x its 60s
        # frame period. Weight switching is latched (see below).
        self.declare_parameter('env_stale_sec', 180.0)
        self.declare_parameter('stale_latch_sec', 2.0)

        self.crop_type = self.get_parameter('crop_type').value
        self.env_stale_sec = self.get_parameter('env_stale_sec').value
        self.stale_latch_sec = self.get_parameter('stale_latch_sec').value

        # Load crop profiles
        profiles_path = self.get_parameter('crop_profiles_path').value
        self.profiles = self._load_profiles(profiles_path)
        self.profile = self.profiles.get(self.crop_type, {})

        # Infection model parameters (literature-sourced, see yaml refs)
        self.infection_model = self._load_infection_model(self.profile)

        # LWD calculator: fixed env node sends one frame per 60s,
        # so the 24h sliding window holds 1440 points at 1-minute interval.
        self.lwd_calc = LWDCalculator(
            window_hours=24, interval_minutes=1, warm_up_points=12)

        # Demo support: seed the LWD window with synthetic past environment
        # data (Nanjing August diurnal model) so the fusion chain does not
        # start in COLD_BOOT at competition time. 0 = disabled (default).
        # Only the LWD history is warmed; ongoing env data still comes from
        # the real LoRa node (or lora_bridge use_mock).
        # dynamic_typing: launch/CLI overrides arrive as int ('24'), double
        # ('24.0') or string depending on the path — accept all and coerce.
        from rcl_interfaces.msg import ParameterDescriptor
        self.declare_parameter(
            'mock_history_hours', 0.0,
            ParameterDescriptor(dynamic_typing=True))
        try:
            mock_hours = float(self.get_parameter('mock_history_hours').value)
        except (TypeError, ValueError):
            mock_hours = 0.0
        if mock_hours > 0.0:
            self._backfill_mock_history(mock_hours)

        # State
        self.last_vision = None
        self.last_env = None
        self.last_env_ts = 0.0
        self.last_fusion_alert = ALERT_NORMAL
        self.last_fusion_mode = MODE_BALANCED

        # Stale latch: avoid weight flapping on frame jitter
        self._stale_latched = False
        self._stale_pending = None
        self._stale_pending_since = 0.0

        # Hysteresis band (alert level changes only if delta exceeds this)
        self.hysteresis_delta = 0.15

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.sub_vision = self.create_subscription(
            Diagnosis, '/vision/diagnosis', self.on_vision, 10)
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
                'infection_model': dict(DEFAULT_INFECTION_MODEL),
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
                'infection_model': dict(DEFAULT_INFECTION_MODEL,
                                        **{'rh_onset': 80.0, 'rh_full': 92.0,
                                           'lwd_base_hours': 8.0}),
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
                'infection_model': dict(DEFAULT_INFECTION_MODEL,
                                        **{'lwd_base_hours': 10.0}),
                'disease_risk_classes': [
                    'leaf_spot', 'powdery_mildew_leaf', 'gray_mold',
                    'angular_leaf_spot', 'blossom_blight',
                    'powdery_mildew_fruit', 'anthracnose_fruit_rot', 'healthy'
                ],
            },
        }

    def _load_infection_model(self, profile: dict) -> dict:
        im = profile.get('infection_model')
        if not im:
            self.get_logger().warn(
                f'No infection_model for crop={self.crop_type}, '
                f'falling back to defaults (legacy behavior)')
            return dict(DEFAULT_INFECTION_MODEL)
        merged = dict(DEFAULT_INFECTION_MODEL)
        merged.update(im)
        refs = im.get('references') or []
        self.get_logger().info(
            f'infection_model loaded for {self.crop_type}: '
            f'temp_optimal={merged["temp_optimal"]}, '
            f'rh=[{merged["rh_onset"]},{merged["rh_full"]}], '
            f'lwd_base={merged["lwd_base_hours"]}h, refs={len(refs)}')
        return merged

    def _backfill_mock_history(self, hours: float):
        """Backfill the LWD window with synthetic past samples.

        Nanjing August diurnal model (27-34.5C, RH ~75-96%): piecewise
        temperature curve (05:00 low, 15:00 high, linear night decay),
        humidity inverse of temperature, small seeded noise. Samples are
        spaced by the calculator's interval and backdated, so phase
        becomes NORMAL and lwd_hours reflects a realistic humid night
        instead of a cold boot.
        """
        import random
        import time as _time

        rng = random.Random(42)
        t_low, t_high = 27.0, 34.5
        humi_min = max(55.0, 96.0 - (t_high - t_low) * 2.8)  # ~75%

        def diurnal(h):
            if 5.0 <= h <= 15.0:
                return (1.0 - math.cos(math.pi * (h - 5.0) / 10.0)) / 2.0
            past = h - 15.0 if h > 15.0 else h + 24.0 - 15.0
            return max(0.0, 1.0 - past / 14.0)

        step_min = self.lwd_calc.interval_minutes
        n = min(int(hours * 60.0 / step_min), self.lwd_calc.max_points)
        now = _time.time()
        for i in range(n, 0, -1):
            ts = now - i * step_min * 60.0
            lt = _time.localtime(ts)
            hour = lt.tm_hour + lt.tm_min / 60.0
            d = diurnal(hour)
            temp = t_low + (t_high - t_low) * d + rng.uniform(-0.4, 0.4)
            humi = min(99.0, 96.0 - (96.0 - humi_min) * d
                       + rng.uniform(-1.5, 1.5))
            soil_h = 58.0 + rng.uniform(-3.0, 3.0)
            self.lwd_calc.update(temp, humi, soil_h, None, ts)
        self.get_logger().info(
            f'Mock env history backfilled: {n} points over {hours:.1f}h, '
            f'phase={self.lwd_calc.phase.value}, '
            f'lwd={self.lwd_calc.lwd_hours:.1f}h')

    def on_vision(self, msg: Diagnosis):
        self.last_vision = msg

    def on_fixed_env(self, msg: Environment):
        now = self.get_clock().now().nanoseconds / 1e9
        # The fixed (LoRa) node is the sole environment source: it updates
        # both the "current environment" cache and the LWD calculator
        # (dedup by sampling interval keeps one point per 5 min).
        self.last_env = msg
        self.last_env_ts = now
        self.lwd_calc.update(
            msg.air_temp, msg.air_humidity, msg.soil_humidity,
            msg.leaf_wetness, now)

    def tick(self):
        result = self._fuse()
        self.pub_fusion.publish(result)

    # --- Infection-model-driven component helpers ---

    def _humi_risk(self, humi: float) -> float:
        """Threshold-style humidity risk (piecewise), per crop profile."""
        onset = float(self.infection_model['rh_onset'])
        full = float(self.infection_model['rh_full'])
        if humi < onset:
            return 0.0
        if humi >= full:
            return 1.0
        return (humi - onset) / max(1e-6, full - onset)

    def _temp_factor(self, temp: float) -> float:
        """Temperature suitability from crop-specific optimal/tolerance."""
        t_lo, t_hi = self.infection_model['temp_optimal']
        tol_lo, tol_hi = self.infection_model['temp_tolerance']
        if t_lo <= temp <= t_hi:
            return 1.0
        if temp < tol_lo or temp > tol_hi:
            return 0.3
        return 0.7

    def _lwd_threshold_at(self, temp: float) -> float:
        """Temperature-corrected LWD requirement (Mills-table style).

        threshold_at(T) = lwd_base * correction^k, where k is the number of
        5C steps by which T deviates from the optimal range. Colder/warmer
        than optimal => more wetness hours needed for infection.
        """
        base = float(self.infection_model['lwd_base_hours'])
        corr = float(self.infection_model['lwd_temp_correction'])
        t_lo, t_hi = self.infection_model['temp_optimal']
        if t_lo <= temp <= t_hi or corr <= 1.0:
            return base
        dev = (t_lo - temp) if temp < t_lo else (temp - t_hi)
        k = min(3, math.ceil(dev / 5.0))
        return base * (corr ** k)

    def _is_env_stale(self, now: float) -> bool:
        """Latched stale detection: state changes only after persisting."""
        raw = (now - self.last_env_ts) > self.env_stale_sec
        if raw == self._stale_latched:
            self._stale_pending = None
            return self._stale_latched
        if self._stale_pending != raw:
            self._stale_pending = raw
            self._stale_pending_since = now
            return self._stale_latched
        if (now - self._stale_pending_since) >= self.stale_latch_sec:
            self._stale_latched = raw
            self._stale_pending = None
        return self._stale_latched

    def _fuse(self) -> FusionResult:
        now = self.get_clock().now().nanoseconds / 1e9
        msg = FusionResult()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'fusion'

        profile = self.profile
        w = profile.get('weights', {'vision': 0.5, 'env': 0.3,
                                      'interaction': 0.2})
        high_humidity_th = profile.get('high_humidity_threshold', 85.0)
        drought_th = profile.get('drought_threshold', 30.0)
        risk_classes = profile.get('disease_risk_classes', [])

        # --- Vision component ---
        # No discount for unknown disease classes: an early-warning system
        # must not systematically suppress novel diseases (Smith et al. 2018,
        # PLoS ONE — low action thresholds preserve control efficacy).
        # Unknown classes are routed to mode UNKNOWN_DISEASE so the advisory
        # layer can recommend manual inspection instead of spraying.
        p_vis = 0.0
        vision_class = 'none'
        vision_conf = 0.0
        unknown_disease = False
        if self.last_vision is not None:
            vision_class = self.last_vision.disease_class
            vision_conf = self.last_vision.confidence
            if vision_class != 'healthy':
                p_vis = vision_conf
                unknown_disease = vision_class not in risk_classes

        # --- Environment component (infection-model-driven) ---
        env = self._effective_env(now)
        e_norm = 0.0
        trend = self.lwd_calc.recent_trend()
        lwd = self.lwd_calc.lwd_hours
        phase = self.lwd_calc.phase

        if env is not None:
            humi = env.air_humidity
            temp = env.air_temp
            humi_risk = self._humi_risk(humi)
            temp_factor = self._temp_factor(temp)
            lwd_threshold = self._lwd_threshold_at(temp)
            lwd_factor = (min(1.0, lwd / lwd_threshold)
                          if lwd_threshold > 0 else 0.0)
            e_norm = (humi_risk * 0.4 + temp_factor * 0.3
                      + lwd_factor * 0.3)
        else:
            humi = 0.0
            temp = float('nan')
            lwd_threshold = float(self.infection_model['lwd_base_hours'])

        # --- Interaction term ---
        interaction = p_vis * e_norm

        # --- Stale detection: boost vision weight if env data is stale ---
        env_stale = self._is_env_stale(now)
        w_v = w['vision'] * (1.3 if env_stale else 1.0)
        w_e = w['env'] * (0.7 if env_stale else 1.0)
        w_i = w['interaction']
        total_w = w_v + w_e + w_i
        w_v /= total_w
        w_e /= total_w
        w_i /= total_w

        # --- Risk score ---
        trend_factor = 1.0 + 0.2 * max(0.0, trend)  # rising humidity amplifies
        vision_term = w_v * p_vis
        env_term = w_e * e_norm * trend_factor
        interaction_term = w_i * interaction
        risk = vision_term + env_term + interaction_term

        # Cold-start penalty on confidence
        confidence = 1.0
        if phase == Phase.COLD_BOOT:
            confidence = 0.3
        elif phase == Phase.WARM_UP:
            confidence = 0.5 + 0.5 * self.lwd_calc.fill_ratio

        # --- Gating: determine fusion mode ---
        mode = MODE_BALANCED
        if unknown_disease and p_vis > 0.3:
            mode = MODE_UNKNOWN_DISEASE
        elif p_vis > 0.7:
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

        # --- Data-sufficiency gating (aligns with ARCHITECTURE.md §6) ---
        # Alert level is capped while LWD history is insufficient; the raw
        # risk_score is published unchanged.
        if phase == Phase.COLD_BOOT:
            alert = min(alert, ALERT_SUSPICION)
        elif phase == Phase.WARM_UP:
            alert = min(alert, ALERT_WARNING)
        self.last_fusion_mode = mode

        # --- Evidence chain ---
        evidence = []
        if vision_class != 'none':
            evidence.append(
                f'Vision: {vision_class} (conf={vision_conf:.2f})')
        if env is not None:
            evidence.append(
                f'Env: T={env.air_temp:.1f}C H={env.air_humidity:.1f}%')
        evidence.append(
            f'LWD={lwd:.1f}h/{lwd_threshold:.1f}h (phase={phase.value})')
        evidence.append(f'Mode={mode}')
        if env_stale:
            evidence.append('Env stale: vision weight boosted')
        if phase != Phase.NORMAL:
            evidence.append(f'Alert capped by data sufficiency ({phase.value})')

        msg.risk_score = float(risk)
        msg.alert_level = alert
        msg.mode = mode
        msg.evidence_chain = evidence
        msg.lwd_hours = float(lwd)
        msg.confidence = float(confidence)
        msg.vision_term = float(vision_term)
        msg.env_term = float(env_term)
        msg.interaction_term = float(interaction_term)
        return msg

    def _effective_env(self, now: float):
        """Return the latest fixed-node environment if still fresh."""
        if self.last_env is None:
            return None
        if (now - self.last_env_ts) > self.env_stale_sec:
            return None
        return self.last_env

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
