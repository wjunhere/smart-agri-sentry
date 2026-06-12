import os
import yaml


ALERT_LEVEL_MAP = {
    'NORMAL': 0,
    'SUSPICION': 1,
    'WARNING': 2,
    'CRITICAL': 3,
}

_PRIORITY_ORDER = {
    'CRITICAL': 0,
    'HIGH': 1,
    'MEDIUM': 2,
    'LOW': 3,
}


class RuleEngine:
    """YAML-based rule engine for advisory generation."""

    def __init__(self, rules):
        self.rules = rules

    @classmethod
    def from_yaml(cls, path):
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
            return cls(data.get('rules', []))
        return cls([])

    @staticmethod
    def default_action():
        return {
            'action_type': 'NONE',
            'priority': 'LOW',
            'description': '暂无明确建议，继续监测',
            'steps': [],
        }

    def match(self, fusion, forecast, env, crop_type):
        for rule in self.rules:
            if self._match_conditions(
                    rule.get('conditions', {}), fusion, forecast, env, crop_type):
                return rule.get('action', self.default_action())
        return self.default_action()

    def _match_conditions(self, cond, fusion, forecast, env, crop_type):
        if 'crop_type' in cond and cond['crop_type'] != crop_type:
            return False
        if 'alert_level' in cond:
            level_value = ALERT_LEVEL_MAP.get(cond['alert_level'])
            if level_value is None or fusion.alert_level != level_value:
                return False
        if 'mode' in cond and fusion.mode != cond['mode']:
            return False
        if 'alert_type' in cond and forecast.alert_type != cond['alert_type']:
            return False
        if 'risk_min' in cond and fusion.risk_score < cond['risk_min']:
            return False
        if 'risk_max' in cond and fusion.risk_score > cond['risk_max']:
            return False
        if env is not None:
            if ('humidity_max' in cond
                    and env.air_humidity > cond['humidity_max']):
                return False
            if ('temperature_min' in cond
                    and env.air_temp < cond['temperature_min']):
                return False
        else:
            if 'humidity_max' in cond or 'temperature_min' in cond:
                return False
        return True

    def highest_priority_action(self, actions):
        if not actions:
            return self.default_action()
        return min(actions, key=lambda a: _PRIORITY_ORDER.get(a.get('priority', 'LOW'), 99))
