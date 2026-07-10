"""China Meteorological Administration API client."""
import random
import urllib.request
import urllib.error
import json


class CMAClient:
    def __init__(self, api_base_url="", api_key="", mock_mode=False):
        self.api_base_url = api_base_url
        self.api_key = api_key
        self.mock_mode = mock_mode

    def fetch_grid_forecast(self, lat, lon):
        if self.mock_mode:
            return self._mock_forecast(lat, lon)
        return self._http_get(self._build_url(lat, lon))

    def fetch_disaster_warning(self, lat, lon):
        if self.mock_mode:
            return []
        data = self._http_get(self._build_warning_url(lat, lon))
        if data is None:
            return []
        return data.get("alerts", [])

    def _build_url(self, lat, lon):
        return f"{self.api_base_url}?lat={lat}&lon={lon}&key={self.api_key}"

    def _build_warning_url(self, lat, lon):
        return f"{self.api_base_url}/warning?lat={lat}&lon={lon}&key={self.api_key}"

    def _http_get(self, url):
        try:
            req = urllib.request.Request(url)
            with urllib.request.urlopen(req, timeout=30) as resp:
                return json.loads(resp.read().decode())
        except (urllib.error.URLError, OSError, json.JSONDecodeError, ValueError):
            return None

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
