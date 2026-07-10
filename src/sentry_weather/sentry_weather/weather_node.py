"""Weather data ingestion node for Smart Agri Sentry."""
import rclpy
from rclpy.node import Node
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
        self.declare_parameter('qweather_project_id', '')
        self.declare_parameter('qweather_credential_id', '')
        self.declare_parameter('qweather_private_key_path', '')
        self.declare_parameter('use_paid_api', False)
        self.declare_parameter('mock_mode', True)

        self.lat = self.get_parameter('lat').value
        self.lon = self.get_parameter('lon').value
        self.city = self.get_parameter('city').value
        self.fetch_interval_sec = self.get_parameter('fetch_interval_sec').value
        self.mock_mode = self.get_parameter('mock_mode').value

        cache_path = self.get_parameter('cache_path').value
        project_id = self.get_parameter('qweather_project_id').value
        credential_id = self.get_parameter('qweather_credential_id').value
        private_key_path = self.get_parameter('qweather_private_key_path').value
        use_paid_api = self.get_parameter('use_paid_api').value

        self.cache = CacheManager(cache_path)
        self.client = CMAClient(project_id=project_id,
                                credential_id=credential_id,
                                private_key_path=private_key_path,
                                use_paid_api=use_paid_api,
                                mock_mode=self.mock_mode)
        self.last_published = None

        self.pub = self.create_publisher(WeatherForecast, '/weather/forecast', 10)

        # Publish cached data on startup
        cached = self._load_from_cache()
        if cached is not None:
            msg = self._raw_to_msg(cached, True)
            self.pub.publish(msg)
            self.last_published = msg

        # Initial fetch
        self._fetch_and_publish()

        self.timer = self.create_timer(float(self.fetch_interval_sec), self._on_timer)
        self.get_logger().info('Weather node ready (mock=%s)', str(self.mock_mode))

    def _on_timer(self):
        self._fetch_and_publish()

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

        raw.setdefault("city", self.city)
        raw.setdefault("lat", self.lat)
        raw.setdefault("lon", self.lon)

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

        for d in raw.get("days", []):
            day = WeatherDay()
            day.day_offset = d["day_offset"]
            day.temp_high = d["temp_high"]
            day.temp_low = d["temp_low"]
            day.humidity = d["humidity"]
            day.precipitation = d["precipitation"]
            day.wind_speed = d["wind_speed"]
            day.weather_desc = d.get("weather_desc", "")
            msg.days.append(day)

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
