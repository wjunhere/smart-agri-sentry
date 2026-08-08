"""Weather data ingestion node for Smart Agri Sentry."""
import json
import os

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from sentry_interfaces.msg import WeatherForecast, WeatherDay, WeatherHour
from .cma_client import CMAClient
from .cache_manager import CacheManager


class WeatherNode(Node):
    def __init__(self):
        super().__init__('weather_node')
        self.declare_parameter('lat', 39.9)
        self.declare_parameter('lon', 116.4)
        self.declare_parameter('city', '')
        self.declare_parameter('fetch_interval_sec', 10800)
        self.declare_parameter('cache_path', '/tmp/sentry_weather_cache.json')
        self.declare_parameter('location_cache_path',
                               '~/.sentry/weather_location.json')
        self.declare_parameter('qweather_api_key', '')
        self.declare_parameter('qweather_api_host', 'devapi.qweather.com')
        self.declare_parameter('qweather_project_id', '')
        self.declare_parameter('qweather_credential_id', '')
        self.declare_parameter('qweather_private_key_path', '')
        self.declare_parameter('mock_mode', True)

        self.lat = self.get_parameter('lat').value
        self.lon = self.get_parameter('lon').value
        self.city = self.get_parameter('city').value
        self.fetch_interval_sec = self.get_parameter('fetch_interval_sec').value
        self.mock_mode = self.get_parameter('mock_mode').value

        self.location_cache_path = os.path.expanduser(
            self.get_parameter('location_cache_path').value)

        # A location pushed at runtime (frontend / mini-program geolocation)
        # overrides the configured parameters and survives restarts.
        self._load_saved_location()

        cache_path = self.get_parameter('cache_path').value
        api_key = self.get_parameter('qweather_api_key').value
        api_host = self.get_parameter('qweather_api_host').value
        project_id = self.get_parameter('qweather_project_id').value
        credential_id = self.get_parameter('qweather_credential_id').value
        private_key_path = self.get_parameter('qweather_private_key_path').value

        self.cache = CacheManager(cache_path)
        self.client = CMAClient(project_id=project_id,
                                credential_id=credential_id,
                                private_key_path=private_key_path,
                                api_key=api_key,
                                api_host=api_host,
                                mock_mode=self.mock_mode)
        self.last_published = None

        self.pub = self.create_publisher(WeatherForecast, '/weather/forecast', 10)
        self.create_subscription(
            NavSatFix, '/weather/set_location', self._on_set_location, 10)

        # Publish cached data on startup
        cached = self._load_from_cache()
        if cached is not None:
            msg = self._raw_to_msg(cached, True)
            self.pub.publish(msg)
            self.last_published = msg

        # Initial fetch
        self._fetch_and_publish()

        interval = 60.0 if self.mock_mode else float(self.fetch_interval_sec)
        self.timer = self.create_timer(interval, self._on_timer)
        # Republish the last forecast every 60s so late subscribers (e.g. the
        # miniprogram bridge started after this node) still receive data even
        # when the real-mode fetch interval is hours long.
        self.repub_timer = self.create_timer(60.0, self._on_republish)
        self.get_logger().info(f'Weather node ready (mock={self.mock_mode})')

    def _on_set_location(self, msg):
        lat = float(msg.latitude)
        lon = float(msg.longitude)
        if not (-90.0 <= lat <= 90.0 and -180.0 <= lon <= 180.0):
            self.get_logger().warn(
                f'Ignoring invalid location lat={lat}, lon={lon}')
            return
        self.lat = lat
        self.lon = lon
        city = self.client.lookup_city(lat, lon)
        if city:
            self.city = city
        self._save_location()
        self.get_logger().info(
            f'Weather location updated: {self.city or "?"} '
            f'({self.lat:.4f}, {self.lon:.4f}), refetching')
        self._fetch_and_publish()

    def _load_saved_location(self):
        try:
            with open(self.location_cache_path) as f:
                saved = json.load(f)
            lat = float(saved['lat'])
            lon = float(saved['lon'])
            if -90.0 <= lat <= 90.0 and -180.0 <= lon <= 180.0:
                self.lat = lat
                self.lon = lon
                self.city = saved.get('city', '') or self.city
                self.get_logger().info(
                    f'Restored saved location {self.city or "?"} '
                    f'({lat:.4f}, {lon:.4f})')
        except (OSError, KeyError, TypeError, ValueError):
            pass

    def _save_location(self):
        try:
            os.makedirs(os.path.dirname(self.location_cache_path),
                        exist_ok=True)
            with open(self.location_cache_path, 'w') as f:
                json.dump({'lat': self.lat, 'lon': self.lon,
                           'city': self.city}, f)
        except OSError as e:
            self.get_logger().warn(f'Failed to save location: {e}')

    def _on_timer(self):
        self._fetch_and_publish()

    def _on_republish(self):
        if self.last_published is not None:
            self.pub.publish(self.last_published)

    def _fetch_and_publish(self):
        raw = self.client.fetch_grid_forecast(self.lat, self.lon)
        if raw is None:
            self.get_logger().warn('API fetch failed, trying cache')
            cached = self._load_from_cache()
            if cached is not None:
                msg = self._raw_to_msg(cached, True)
                self.pub.publish(msg)
                self.last_published = msg
            return

        raw["city"] = self.city or raw.get("city", "")
        raw["lat"] = self.lat
        raw["lon"] = self.lon

        alerts = self.client.fetch_disaster_warning(self.lat, self.lon)
        raw["disaster_alerts"] = alerts if alerts else []

        self.cache.save(raw)
        msg = self._raw_to_msg(raw, False)
        self.pub.publish(msg)
        self.last_published = msg
        self.get_logger().info('Weather forecast published')

    def _load_from_cache(self):
        if self.cache.is_valid():
            return self.cache.load()
        return None

    def _raw_to_msg(self, raw, stale):
        msg = WeatherForecast()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'weather'
        msg.city = raw.get("city", "")
        msg.lat = float(raw.get("lat", self.lat))
        msg.lon = float(raw.get("lon", self.lon))
        msg.stale = stale

        for i, d in enumerate(raw.get("days", [])[:7]):
            day = WeatherDay()
            day.day_offset = d["day_offset"]
            day.temp_high = d["temp_high"]
            day.temp_low = d["temp_low"]
            day.humidity = d["humidity"]
            day.precipitation = d["precipitation"]
            day.wind_speed = d["wind_speed"]
            day.weather_desc = d.get("weather_desc", "")
            msg.days[i] = day

        for h in raw.get("hours", []):
            hour = WeatherHour()
            hour.hour_offset = h["hour_offset"]
            hour.temp = h["temp"]
            hour.humidity = h["humidity"]
            hour.precipitation = h["precipitation"]
            hour.wind_speed = h["wind_speed"]
            msg.hours.append(hour)

        msg.disaster_alerts = raw.get("disaster_alerts", [])
        return msg


def main(args=None):
    rclpy.init(args=args)
    node = WeatherNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
