"""Tests for the pure-Python adaptive exposure controller."""

import sys
from pathlib import Path

import numpy as np
import pytest

PKG_ROOT = Path(__file__).resolve().parents[1]
if str(PKG_ROOT) not in sys.path:
    sys.path.insert(0, str(PKG_ROOT))

from sentry_bringup.auto_exposure import (
    AdaptiveExposureController,
    LumaStats,
    compute_luma_stats,
)


def make_controller(**overrides):
    kwargs = dict(
        target_luma=80.0, deadband=0.05, max_step=1.4, sat_limit=0.02,
        exp_min_us=2000.0, exp_max_moving_us=20000.0,
        exp_max_still_us=100000.0, gain_min=0.0, gain_max=12.0,
        gain_step=1.0, update_period_s=0.4,
    )
    kwargs.update(overrides)
    return AdaptiveExposureController(**kwargs)


def test_compute_luma_stats_white_frame():
    frame = np.full((480, 640, 3), 255, dtype=np.uint8)
    stats = compute_luma_stats(frame)
    assert stats.mean == pytest.approx(255.0)
    assert stats.saturated_ratio == pytest.approx(1.0)


def test_compute_luma_stats_black_frame():
    frame = np.zeros((480, 640, 3), dtype=np.uint8)
    stats = compute_luma_stats(frame)
    assert stats.mean == pytest.approx(0.0)
    assert stats.saturated_ratio == pytest.approx(0.0)


def test_dark_frame_raises_exposure_first():
    ctl = make_controller()
    ctl.seed(exposure_us=10000.0, gain=3.0)
    cmd = ctl.update(LumaStats(mean=40.0, saturated_ratio=0.0),
                     moving=False, now_s=0.0)
    assert cmd.exposure_us == pytest.approx(14000.0)  # ratio clamped to 1.4
    assert cmd.gain == pytest.approx(3.0)             # gain untouched


def test_exposure_at_still_cap_then_gain_increases():
    ctl = make_controller()
    ctl.seed(exposure_us=100000.0, gain=3.0)
    cmd = ctl.update(LumaStats(mean=40.0, saturated_ratio=0.0),
                     moving=False, now_s=0.0)
    assert cmd.exposure_us == pytest.approx(100000.0)
    assert cmd.gain == pytest.approx(4.0)


def test_gain_capped_at_max_returns_none():
    ctl = make_controller()
    ctl.seed(exposure_us=100000.0, gain=12.0)
    cmd = ctl.update(LumaStats(mean=40.0, saturated_ratio=0.0),
                     moving=False, now_s=0.0)
    assert cmd is None  # already at both caps, nothing to do


def test_bright_frame_lowers_gain_first():
    ctl = make_controller()
    ctl.seed(exposure_us=20000.0, gain=5.0)
    cmd = ctl.update(LumaStats(mean=200.0, saturated_ratio=0.01),
                     moving=False, now_s=0.0)
    assert cmd.gain == pytest.approx(4.0)
    assert cmd.exposure_us == pytest.approx(20000.0)


def test_bright_frame_at_min_gain_lowers_exposure():
    ctl = make_controller()
    ctl.seed(exposure_us=20000.0, gain=0.0)
    cmd = ctl.update(LumaStats(mean=200.0, saturated_ratio=0.01),
                     moving=False, now_s=0.0)
    # ratio = 80/200 = 0.4 -> clamped to 1/1.4
    assert cmd.exposure_us == pytest.approx(20000.0 / 1.4)
    assert cmd.gain == pytest.approx(0.0)


def test_saturated_frame_fast_exposure_cut():
    ctl = make_controller()
    ctl.seed(exposure_us=50000.0, gain=3.0)
    cmd = ctl.update(LumaStats(mean=100.0, saturated_ratio=0.05),
                     moving=False, now_s=0.0)
    assert cmd.exposure_us == pytest.approx(30000.0)  # 50000 * 0.6
    assert cmd.gain == pytest.approx(3.0)


def test_deadband_no_change():
    ctl = make_controller()
    ctl.seed(exposure_us=20000.0, gain=3.0)
    cmd = ctl.update(LumaStats(mean=82.0, saturated_ratio=0.0),
                     moving=False, now_s=0.0)
    assert cmd is None


def test_moving_clamps_exposure_even_on_target():
    ctl = make_controller()
    ctl.seed(exposure_us=100000.0, gain=3.0)
    cmd = ctl.update(LumaStats(mean=80.0, saturated_ratio=0.0),
                     moving=True, now_s=0.0)
    assert cmd.exposure_us == pytest.approx(20000.0)  # moving cap


def test_moving_cap_limits_brightening():
    ctl = make_controller()
    ctl.seed(exposure_us=18000.0, gain=0.0)
    cmd = ctl.update(LumaStats(mean=40.0, saturated_ratio=0.0),
                     moving=True, now_s=0.0)
    assert cmd.exposure_us == pytest.approx(20000.0)
    assert cmd.gain == pytest.approx(1.0)  # hit cap, gain takes over


def test_rate_limit_blocks_fast_repeat():
    ctl = make_controller()
    ctl.seed(exposure_us=10000.0, gain=3.0)
    first = ctl.update(LumaStats(mean=40.0, saturated_ratio=0.0),
                       moving=False, now_s=0.0)
    assert first is not None
    second = ctl.update(LumaStats(mean=30.0, saturated_ratio=0.0),
                        moving=False, now_s=0.2)
    assert second is None  # inside 0.4s window
    third = ctl.update(LumaStats(mean=30.0, saturated_ratio=0.0),
                       moving=False, now_s=0.5)
    assert third is not None


def test_zero_mean_treated_as_very_dark():
    ctl = make_controller()
    ctl.seed(exposure_us=10000.0, gain=3.0)
    cmd = ctl.update(LumaStats(mean=0.0, saturated_ratio=0.0),
                     moving=False, now_s=0.0)
    assert cmd.exposure_us == pytest.approx(14000.0)
