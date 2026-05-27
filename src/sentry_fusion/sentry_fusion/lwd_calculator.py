"""Leaf Wetness Duration (LWD) calculator with 24h sliding window.

288 data points at 5-minute intervals. Supports COLD_BOOT / WARM_UP / NORMAL
phases. LWD is estimated from air humidity and temperature using a simple
empirical model.
"""

from collections import deque
from enum import Enum
import time


class Phase(Enum):
    COLD_BOOT = 'COLD_BOOT'
    WARM_UP = 'WARM_UP'
    NORMAL = 'NORMAL'


class LWDCalculator:
    """24-hour LWD sliding window calculator.

    Parameters
    ----------
    window_hours : int
        Total window size in hours (default 24).
    interval_minutes : int
        Sampling interval in minutes (default 5).
    warm_up_points : int
        Number of points needed to exit COLD_BOOT.
    """

    def __init__(self, window_hours: int = 24, interval_minutes: int = 5,
                 warm_up_points: int = 12):
        self.max_points = (window_hours * 60) // interval_minutes
        self.warm_up_points = warm_up_points
        self.interval_minutes = interval_minutes
        self._history = deque(maxlen=self.max_points)
        self._last_ts = 0.0

    @property
    def phase(self) -> Phase:
        n = len(self._history)
        if n == 0:
            return Phase.COLD_BOOT
        if n < self.warm_up_points:
            return Phase.WARM_UP
        return Phase.NORMAL

    @property
    def lwd_hours(self) -> float:
        """Return accumulated LWD over the current window in hours."""
        if not self._history:
            return 0.0
        wet_minutes = sum(
            1 for _, _, wet in self._history if wet) * self.interval_minutes
        return wet_minutes / 60.0

    @property
    def fill_ratio(self) -> float:
        """Fraction of the window that is filled."""
        return len(self._history) / self.max_points

    def update(self, air_temp: float, air_humidity: float,
               soil_humidity: float = None, leaf_wetness: float = None,
               timestamp: float = None) -> float:
        """Add a new sample and return current LWD hours.

        Leaf wetness is estimated when no direct sensor is available:
        - Direct leaf_wetness > 0.5  => wet
        - Or air_humidity > 90% and air_temp between 10-30C => wet
        - Or air_humidity > 85% and soil_humidity > 70% => wet
        """
        ts = timestamp if timestamp is not None else time.time()
        # Skip if called too frequently (protect from duplicate data)
        if ts - self._last_ts < (self.interval_minutes * 60) * 0.8:
            return self.lwd_hours
        self._last_ts = ts

        wet = self._estimate_wetness(
            air_temp, air_humidity, soil_humidity, leaf_wetness)
        self._history.append((ts, air_humidity, wet))
        return self.lwd_hours

    def _estimate_wetness(self, air_temp: float, air_humidity: float,
                          soil_humidity: float = None,
                          leaf_wetness: float = None) -> bool:
        if leaf_wetness is not None and leaf_wetness > 0.5:
            return True
        if air_humidity > 90.0 and 10.0 <= air_temp <= 30.0:
            return True
        if (air_humidity > 85.0 and soil_humidity is not None
                and soil_humidity > 70.0):
            return True
        return False

    def recent_trend(self, lookback_points: int = 6) -> float:
        """Return humidity trend over last N points (-1 to +1).

        Positive = rising humidity, negative = falling.
        """
        if len(self._history) < 2:
            return 0.0
        recent = list(self._history)[-lookback_points:]
        if len(recent) < 2:
            return 0.0
        first_h = recent[0][1]
        last_h = recent[-1][1]
        # Normalize to roughly -1..+1 based on 0-100% humidity scale
        return max(-1.0, min(1.0, (last_h - first_h) / 50.0))
