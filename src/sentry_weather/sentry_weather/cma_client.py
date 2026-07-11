"""QWeather API client with JWT authentication."""
import base64
import json
import random
import time
import urllib.error
import urllib.request

from cryptography.hazmat.primitives import serialization


# QWeather free dev API base (switch to api.qweather.com for paid)
QWEATHER_BASE = "https://devapi.qweather.com/v7"
QWEATHER_PAID_BASE = "https://api.qweather.com/v7"


def _make_jwt(project_id, credential_id, private_key_path):
    """Generate a short-lived JWT for QWeather API auth."""
    with open(private_key_path, "rb") as f:
        private_key = serialization.load_pem_private_key(f.read(), password=None)
    now = int(time.time())
    header = base64.urlsafe_b64encode(
        json.dumps({"alg": "EdDSA", "kid": credential_id}).encode()
    ).rstrip(b"=").decode()
    payload = base64.urlsafe_b64encode(
        json.dumps({"sub": project_id, "iat": now, "exp": now + 3600}).encode()
    ).rstrip(b"=").decode()
    signing_input = f"{header}.{payload}".encode()
    signature = base64.urlsafe_b64encode(
        private_key.sign(signing_input)
    ).rstrip(b"=").decode()
    return f"{header}.{payload}.{signature}"


class CMAClient:
    def __init__(self, project_id="", credential_id="", private_key_path="",
                 api_key="", api_host="devapi.qweather.com",
                 mock_mode=False):
        self.project_id = project_id
        self.credential_id = credential_id
        self.private_key_path = private_key_path
        self.api_key = api_key
        self.api_host = api_host
        self.base_url = f"https://{api_host}/v7"
        self.mock_mode = mock_mode

    def fetch_grid_forecast(self, lat, lon):
        if self.mock_mode:
            return self._mock_forecast(lat, lon)

        daily = self._qweather_get(f"{self.base_url}/weather/7d",
                                   f"{lon:.2f},{lat:.2f}")
        if daily is None:
            return None

        hourly = self._qweather_get(f"{self.base_url}/weather/24h",
                                    f"{lon:.2f},{lat:.2f}")
        return self._parse_qweather(daily, hourly, lat, lon)

    def fetch_disaster_warning(self, lat, lon):
        if self.mock_mode:
            return []
        data = self._qweather_get(f"{self.base_url}/warning/now",
                                  f"{lon:.2f},{lat:.2f}")
        if data is None:
            return []
        warnings = data.get("warning", [])
        if warnings is None:
            return []
        alerts = []
        for w in warnings:
            title = w.get("title", "")
            if title:
                alerts.append(title)
        return alerts

    def _qweather_get(self, url, location):
        full_url = f"{url}?location={location}"

        # API Key auth (simpler, preferred)
        if self.api_key:
            full_url += f"&key={self.api_key}"
            return self._http_get(full_url)

        # JWT auth fallback
        if not self.project_id or not self.credential_id or not self.private_key_path:
            return None
        try:
            token = _make_jwt(self.project_id, self.credential_id,
                              self.private_key_path)
        except Exception:
            return None

        req = urllib.request.Request(full_url)
        req.add_header("Authorization", f"Bearer {token}")
        try:
            with urllib.request.urlopen(req, timeout=30) as resp:
                body = json.loads(resp.read().decode())
                if body.get("code") != "200":
                    return None
                return body
        except (urllib.error.URLError, OSError, json.JSONDecodeError, ValueError):
            return None

    def _http_get(self, url):
        try:
            req = urllib.request.Request(url)
            with urllib.request.urlopen(req, timeout=30) as resp:
                body = resp.read()
                # QWeather responses are gzip-compressed
                if body[:2] == b'\x1f\x8b':
                    import gzip
                    body = gzip.decompress(body)
                data = json.loads(body.decode())
                if data.get("code") != "200":
                    return None
                return data
        except (urllib.error.URLError, OSError, json.JSONDecodeError, ValueError):
            return None

    def _parse_qweather(self, daily_data, hourly_data, lat, lon):
        days = []
        for d in daily_data.get("daily", [])[:7]:
            days.append({
                "day_offset": len(days),
                "temp_high": float(d.get("tempMax", 0)),
                "temp_low": float(d.get("tempMin", 0)),
                "humidity": float(d.get("humidity", 50)),
                "precipitation": float(d.get("precip", 0)),
                "wind_speed": float(d.get("windSpeedDay", 0)),
                "weather_desc": d.get("textDay", ""),
            })

        hours = []
        for h in hourly_data.get("hourly", []):
            hours.append({
                "hour_offset": len(hours),
                "temp": float(h.get("temp", 0)),
                "humidity": float(h.get("humidity", 50)),
                "precipitation": float(h.get("precip", 0)),
                "wind_speed": float(h.get("windSpeed", 0)),
            })

        return {"city": "", "lat": lat, "lon": lon, "days": days, "hours": hours}

    def _mock_forecast(self, lat, lon):
        weather_descs = ["晴", "多云", "阴", "小雨", "中雨", "晴", "多云"]
        days = []
        for offset in range(7):
            base_temp = 22.0 + random.uniform(-5, 5)
            days.append({
                "day_offset": offset,
                "temp_high": round(base_temp + random.uniform(2, 8), 1),
                "temp_low": round(base_temp - random.uniform(3, 10), 1),
                "humidity": round(random.uniform(40, 95), 1),
                "precipitation": round(random.uniform(0, 15), 1),
                "wind_speed": round(random.uniform(0, 12), 1),
                "weather_desc": weather_descs[offset % len(weather_descs)],
            })

        hours = []
        for offset in range(168):
            hour_temp = 22.0 + random.uniform(-8, 8)
            hours.append({
                "hour_offset": offset,
                "temp": round(hour_temp, 1),
                "humidity": round(random.uniform(40, 95), 1),
                "precipitation": round(random.uniform(0, 3), 1),
                "wind_speed": round(random.uniform(0, 8), 1),
            })

        return {"city": "MockCity", "lat": lat, "lon": lon, "days": days, "hours": hours}
