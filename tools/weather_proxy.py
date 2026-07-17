"""Local weather proxy — serves real QWeather data to frontend."""
import json
import sys
import time
import threading
import gzip
import urllib.request
from pathlib import Path

SRC = str(Path(__file__).resolve().parent.parent / "src" / "sentry_weather")
sys.path.insert(0, SRC)
from sentry_weather.cma_client import CMAClient, _make_jwt

# --- Config ---
LAT, LON = 34.26, 117.20  # Xuzhou, Jiangsu
REFRESH_SEC = 3600        # 1 hour
PORT = 8090

# --- QWeather client ---
with open("config/qweather_private.pem", "rb") as f:
    from cryptography.hazmat.primitives import serialization
    _pk = serialization.load_pem_private_key(f.read(), password=None)
    f.seek(0)  # rewind for _make_jwt

client = CMAClient(
    project_id="3MTGWPJB8K",
    credential_id="C4B99K2NAQ",
    private_key_path="config/qweather_private.pem",
    api_host="mv4ewv56hj.re.qweatherapi.com",
    mock_mode=False,
)

_cache = {"data": None, "ts": 0, "lock": threading.Lock()}


def fetch_weather():
    data = client.fetch_grid_forecast(LAT, LON)
    alerts = client.fetch_disaster_warning(LAT, LON)
    if data:
        data["disaster_alerts"] = alerts if alerts else []
        data["city"] = "徐州"
        data["lat"] = LAT
        data["lon"] = LON
        data["fetch_time"] = time.strftime("%H:%M:%S")
    return data


def refresh_loop():
    while True:
        try:
            data = fetch_weather()
            with _cache["lock"]:
                _cache["data"] = data
                _cache["ts"] = time.time()
            print(f"[{time.strftime('%H:%M:%S')}] Weather refreshed ({len(data['hours'])}h, {len(data['days'])}d)")
        except Exception as e:
            print(f"[{time.strftime('%H:%M:%S')}] Refresh failed: {e}")
        time.sleep(REFRESH_SEC)


# Start background refresh
threading.Thread(target=refresh_loop, daemon=True).start()
# Wait for first fetch
for _ in range(30):
    with _cache["lock"]:
        if _cache["data"] is not None:
            break
    time.sleep(1)

print(f"Weather proxy: http://localhost:{PORT}/weather.json (refresh every {REFRESH_SEC // 60} min)")

# --- Minimal HTTP server ---
from http.server import HTTPServer, BaseHTTPRequestHandler

class Handler(BaseHTTPRequestHandler):
    def do_GET(self):
        if self.path == "/weather.json":
            with _cache["lock"]:
                data = _cache["data"]
            self.send_response(200)
            self.send_header("Content-Type", "application/json; charset=utf-8")
            self.send_header("Access-Control-Allow-Origin", "*")
            self.end_headers()
            self.wfile.write(json.dumps(data, ensure_ascii=False).encode())
        else:
            self.send_response(404)
            self.end_headers()

    def log_message(self, format, *args):
        pass  # silence

HTTPServer(("", PORT), Handler).serve_forever()
