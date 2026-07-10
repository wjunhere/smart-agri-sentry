"""JSON file cache for weather forecast data."""
import json
import os
import time


class CacheManager:
    def __init__(self, cache_path: str):
        self.cache_path = cache_path

    def save(self, data: dict) -> None:
        os.makedirs(os.path.dirname(self.cache_path), exist_ok=True)
        with open(self.cache_path, 'w') as f:
            json.dump(data, f, ensure_ascii=False, indent=2)

    def load(self) -> dict | None:
        if not os.path.exists(self.cache_path):
            return None
        with open(self.cache_path, 'r') as f:
            return json.load(f)

    def is_valid(self) -> bool:
        if not os.path.exists(self.cache_path):
            return False
        mtime = os.path.getmtime(self.cache_path)
        age_sec = time.time() - mtime
        return age_sec < 24 * 3600
