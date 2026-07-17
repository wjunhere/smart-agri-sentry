"""Software adaptive exposure controller.

Closed-loop AE for cameras whose hardware auto exposure is unreliable
(Hikrobot MV-CS016-10UC converges to ~6ms regardless of scene).
Pure Python + numpy; no ROS dependencies so it can be unit-tested
off-board.

Policy (evaluated per frame, register writes rate-limited):
  1. Saturation guard: >2% pixels clipped -> exposure *= 0.6
  2. Motion clamp: exposure above the moving cap -> clamp immediately
  3. Deadband: |target/mean - 1| < 5% -> no change
  4. Brighten: exposure first (x ratio), then gain at the cap
     Darken:  gain first (- step), then exposure at gain_min
"""

from dataclasses import dataclass

import numpy as np


@dataclass
class LumaStats:
    mean: float
    saturated_ratio: float


@dataclass
class AeCommand:
    exposure_us: float
    gain: float


def compute_luma_stats(bgr_frame, sat_threshold=250, stride=4):
    """Mean luminance and saturated-pixel ratio of a BGR8 frame."""
    small = bgr_frame[::stride, ::stride].astype(np.float32)
    luma = (0.114 * small[..., 0] + 0.587 * small[..., 1]
            + 0.299 * small[..., 2])
    return LumaStats(
        mean=float(luma.mean()),
        saturated_ratio=float((luma > sat_threshold).mean()))


class AdaptiveExposureController:
    def __init__(self, *, target_luma=80.0, deadband=0.05, max_step=1.4,
                 sat_limit=0.02, exp_min_us=2000.0,
                 exp_max_moving_us=20000.0, exp_max_still_us=100000.0,
                 gain_min=0.0, gain_max=12.0, gain_step=1.0,
                 update_period_s=0.4):
        self.target_luma = target_luma
        self.deadband = deadband
        self.max_step = max_step
        self.sat_limit = sat_limit
        self.exp_min_us = exp_min_us
        self.exp_max_moving_us = exp_max_moving_us
        self.exp_max_still_us = exp_max_still_us
        self.gain_min = gain_min
        self.gain_max = gain_max
        self.gain_step = gain_step
        self.update_period_s = update_period_s
        self.exposure_us = exp_min_us
        self.gain = gain_min
        self._last_update_s = None

    def seed(self, exposure_us, gain):
        self.exposure_us = float(exposure_us)
        self.gain = float(gain)

    def update(self, stats, moving, now_s):
        """Return an AeCommand when registers should change, else None."""
        exp_max = (self.exp_max_moving_us if moving
                   else self.exp_max_still_us)

        # Safety paths bypass the rate limit: blown-out frames and motion
        # must react within one frame, not one rate window.
        if stats.saturated_ratio > self.sat_limit:
            return self._apply(
                max(self.exp_min_us, self.exposure_us * 0.6),
                self.gain, now_s)

        if self.exposure_us > exp_max:
            return self._apply(exp_max, self.gain, now_s)

        if (self._last_update_s is not None
                and now_s - self._last_update_s < self.update_period_s):
            return None

        if stats.mean <= 0.0:
            ratio = self.max_step
        else:
            ratio = self.target_luma / stats.mean
        if abs(ratio - 1.0) < self.deadband:
            return None
        ratio = min(max(ratio, 1.0 / self.max_step), self.max_step)

        exposure = self.exposure_us
        gain = self.gain
        if ratio > 1.0:
            exposure = min(exposure * ratio, exp_max)
            if exposure >= exp_max:
                gain = min(gain + self.gain_step, self.gain_max)
        else:
            if gain > self.gain_min:
                gain = max(gain - self.gain_step, self.gain_min)
            else:
                exposure = max(exposure * ratio, self.exp_min_us)
        return self._apply(exposure, gain, now_s)

    def _apply(self, exposure_us, gain, now_s):
        if (abs(exposure_us - self.exposure_us) < 1.0
                and abs(gain - self.gain) < 0.01):
            return None
        self.exposure_us = exposure_us
        self.gain = gain
        self._last_update_s = now_s
        return AeCommand(exposure_us=exposure_us, gain=gain)
