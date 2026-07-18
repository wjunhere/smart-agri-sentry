# 海康相机软件自适应曝光实现计划

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 为 hikrobot_camera_node 增加软件闭环自适应曝光，按车体运动状态动态限制曝光上限，解决亮场景过曝与巡航拖影。

**Architecture:** 新增纯 Python 无 ROS 依赖的 `AdaptiveExposureController`（`auto_exposure.py`），相机节点每帧统计 raw BGR 亮度（gamma LUT 之前）闭环调节 ExposureTime/Gain 寄存器；订阅 `/wheel/odom` 判断 moving，moving 时曝光上限 20ms、静止 100ms；gamma 降为固定 2.0。

**Tech Stack:** Python 3.11（本地 conda py311，numpy+pytest 离线 TDD）、ROS2 Humble（板端）、Hikrobot MVS SDK。

**Spec:** `docs/superpowers/specs/2026-07-17-hikrobot-adaptive-exposure-design.md`

---

### Task 0: 板端基线入库（无 TDD，git 操作）

**背景:** codex 的曝光调优成果（100ms 曝光/gamma/回读日志等）仅以未提交改动存在于板端 `~/dev_ws`，板端无 GitHub 推送凭据（https remote），所以走"板端出 patch → 本地提交推送 → 板端对齐"。

**注意:** 板端基线中 `test_hikrobot_launch_uses_adaptive_exposure_for_low_light` 断言与基线 launch 不一致（断言 `exposure_auto: True`/`gamma: 2.0`，launch 实为 `False`/`3.0`），基线提交后该测试失败是**已知的、预期的**，Task 3 会修正它。

- [ ] **Step 1: 板端确认工作区无未跟踪文件**

```bash
ssh rdk "cd ~/dev_ws && git status --porcelain | grep '^??' || echo 'CLEAN: no untracked files'"
```

Expected: `CLEAN: no untracked files`。若有未跟踪文件，**停下来报告用户**，不要继续。

- [ ] **Step 2: 板端生成 patch 并传回本地**

```bash
ssh rdk "cd ~/dev_ws && git diff HEAD --binary > /tmp/ae_baseline.patch && wc -l /tmp/ae_baseline.patch"
scp rdk:/tmp/ae_baseline.patch /tmp/ae_baseline.patch
```

- [ ] **Step 3: 本地检查并应用 patch**

```bash
git apply --stat /tmp/ae_baseline.patch
git apply --check /tmp/ae_baseline.patch && git apply /tmp/ae_baseline.patch
git status --short | head -25
```

Expected: patch 干净应用，工作区出现与板端一致的 M/D 文件列表（hikrobot_camera_node.py、sentry_v2.launch.py、web_remote_node.py 等约 20 个文件）。

- [ ] **Step 4: 提交基线并推送**

```bash
git add -A
git commit -m "Baseline: codex Hikrobot exposure tuning and vision mode work

Manual 100ms exposure + gamma LUT + hardware readback diagnostics
from on-board codex session; previously uncommitted on RDK."
git push origin codex/hikrobot-vision-node
```

- [ ] **Step 5: 板端对齐到远端（先验证无差异）**

```bash
ssh rdk "cd ~/dev_ws && git fetch origin && git diff HEAD origin/codex/hikrobot-vision-node --stat"
```

Expected: 输出为空（板端工作区与远端提交内容完全一致）。
**仅当输出为空时**执行：

```bash
ssh rdk "cd ~/dev_ws && git reset --hard origin/codex/hikrobot-vision-node"
```

若输出非空：停止，报告用户差异内容。

- [ ] **Step 6: 本地开功能分支**

```bash
git checkout -b feat/adaptive-exposure
```

---

### Task 1: `auto_exposure.py` 纯控制器（TDD）

**Files:**
- Create: `src/sentry_bringup/sentry_bringup/auto_exposure.py`
- Test: `src/sentry_bringup/tests/test_auto_exposure.py`

- [ ] **Step 1: 写失败测试**

创建 `src/sentry_bringup/tests/test_auto_exposure.py`：

```python
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
```

- [ ] **Step 2: 运行测试确认失败**

```bash
python -m pytest src/sentry_bringup/tests/test_auto_exposure.py -v
```

Expected: 全部 FAIL，`ModuleNotFoundError: No module named 'sentry_bringup.auto_exposure'`

- [ ] **Step 3: 实现控制器**

创建 `src/sentry_bringup/sentry_bringup/auto_exposure.py`：

```python
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
        if (self._last_update_s is not None
                and now_s - self._last_update_s < self.update_period_s):
            return None

        exp_max = (self.exp_max_moving_us if moving
                   else self.exp_max_still_us)

        if stats.saturated_ratio > self.sat_limit:
            return self._apply(
                max(self.exp_min_us, self.exposure_us * 0.6),
                self.gain, now_s)

        if self.exposure_us > exp_max:
            return self._apply(exp_max, self.gain, now_s)

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
```

- [ ] **Step 4: 运行测试确认通过**

```bash
python -m pytest src/sentry_bringup/tests/test_auto_exposure.py -v
```

Expected: 13 passed

- [ ] **Step 5: Commit**

```bash
git add src/sentry_bringup/sentry_bringup/auto_exposure.py src/sentry_bringup/tests/test_auto_exposure.py
git commit -m "Add pure-Python adaptive exposure controller with tests"
```

---

### Task 2: 相机节点集成（TDD）

**Files:**
- Modify: `src/sentry_bringup/sentry_bringup/hikrobot_camera_node.py`
- Test: `src/sentry_bringup/tests/test_hikrobot_camera_node.py`

- [ ] **Step 1: 测试 fixture 增加 nav_msgs mock**

修改 `src/sentry_bringup/tests/test_hikrobot_camera_node.py` 的 `mock_ros2_and_cv` fixture，在 `modules` 字典中追加：

```python
        'nav_msgs': types.ModuleType('nav_msgs'),
        'nav_msgs.msg': types.ModuleType('nav_msgs.msg'),
```

并在 fixture 的赋值区追加：

```python
    modules['nav_msgs.msg'].Odometry = type('Odometry', (), {})
```

- [ ] **Step 2: 写失败的节点集成测试**

在 `src/sentry_bringup/tests/test_hikrobot_camera_node.py` 末尾追加：

```python
def _make_ae_node():
    from sentry_bringup.hikrobot_camera_node import HikrobotCameraNode

    node = HikrobotCameraNode.__new__(HikrobotCameraNode)
    node.ae_move_speed_thresh = 0.05
    node.ae_still_speed_thresh = 0.02
    node._moving = True
    node._last_odom_time = None
    node._still_since = None
    node._now = mock.MagicMock(return_value=100.0)
    node.get_logger = mock.MagicMock()
    return node


def _make_odom(speed_x, speed_y=0.0):
    return types.SimpleNamespace(
        twist=types.SimpleNamespace(
            twist=types.SimpleNamespace(
                linear=types.SimpleNamespace(x=speed_x, y=speed_y))))


def test_on_odom_marks_moving_above_threshold():
    node = _make_ae_node()
    node._moving = False

    node._on_odom(_make_odom(0.1))

    assert node._moving is True
    assert node._last_odom_time == 100.0


def test_on_odom_clears_moving_after_still_hold():
    node = _make_ae_node()
    node._on_odom(_make_odom(0.0))
    assert node._moving is True  # hold period not elapsed

    node._now.return_value = 101.5
    node._on_odom(_make_odom(0.0))
    assert node._moving is False  # still for 1.5s >= 1.0s hold


def test_is_moving_true_when_odom_never_received():
    node = _make_ae_node()
    assert node._is_moving() is True


def test_is_moving_true_when_odom_stale():
    node = _make_ae_node()
    node._moving = False
    node._last_odom_time = 97.0  # 3s ago at _now=100
    assert node._is_moving() is True


def test_is_moving_uses_state_when_odom_fresh():
    node = _make_ae_node()
    node._moving = False
    node._last_odom_time = 99.5
    assert node._is_moving() is False


def test_ae_update_from_frame_writes_registers_on_command():
    from sentry_bringup.auto_exposure import AeCommand
    node = _make_ae_node()
    node.cam = mock.MagicMock()
    node.cam.MV_CC_SetFloatValue.return_value = 0
    node.ae_controller = mock.MagicMock()
    node.ae_controller.update.return_value = AeCommand(
        exposure_us=14285.0, gain=3.0)
    node._last_ae_exposure = 20000.0
    node._last_ae_gain = 3.0
    node._moving = False
    node._last_odom_time = 99.5
    frame = np.zeros((480, 640, 3), dtype=np.uint8)

    node._ae_update_from_frame(frame, 100.0)

    node.cam.MV_CC_SetFloatValue.assert_called_once_with(
        'ExposureTime', 14285.0)


def test_ae_update_from_frame_skips_when_no_command():
    node = _make_ae_node()
    node.cam = mock.MagicMock()
    node.ae_controller = mock.MagicMock()
    node.ae_controller.update.return_value = None
    node._last_ae_exposure = 20000.0
    node._last_ae_gain = 3.0
    node._moving = False
    node._last_odom_time = 99.5
    frame = np.zeros((480, 640, 3), dtype=np.uint8)

    node._ae_update_from_frame(frame, 100.0)

    node.cam.MV_CC_SetFloatValue.assert_not_called()
```

- [ ] **Step 3: 运行测试确认失败**

```bash
python -m pytest src/sentry_bringup/tests/test_hikrobot_camera_node.py -v -k "ae_ or odom or moving"
```

Expected: FAIL（`AttributeError: ... no attribute '_on_odom'` 等；`_make_ae_node` 中 `import numpy as np` 需在文件头部确认已存在，若没有则在 imports 区加 `import numpy as np`）

- [ ] **Step 4: 节点集成实现**

修改 `src/sentry_bringup/sentry_bringup/hikrobot_camera_node.py`：

**(a) 文件头部 imports 区**追加：

```python
import math
import time
```

并在 `from sensor_msgs.msg import Image` 之后追加：

```python
from nav_msgs.msg import Odometry

from sentry_bringup.auto_exposure import (
    AdaptiveExposureController,
    compute_luma_stats,
)
```

**(b) `__init__` 参数声明区**（`self.declare_parameter('gamma', 3.0)` 之后）追加：

```python
        self.declare_parameter('ae_enabled', True)
        self.declare_parameter('ae_target_luma', 80.0)
        self.declare_parameter('ae_deadband', 0.05)
        self.declare_parameter('ae_max_step', 1.4)
        self.declare_parameter('ae_sat_limit', 0.02)
        self.declare_parameter('ae_exp_min_us', 2000.0)
        self.declare_parameter('ae_exp_max_moving_us', 20000.0)
        self.declare_parameter('ae_exp_max_still_us', 100000.0)
        self.declare_parameter('ae_gain_min', 0.0)
        self.declare_parameter('ae_gain_max', 12.0)
        self.declare_parameter('ae_move_speed_thresh', 0.05)
        self.declare_parameter('ae_still_speed_thresh', 0.02)
        self.declare_parameter('ae_update_period_s', 0.4)
        self.declare_parameter('odom_topic', '/wheel/odom')
```

**(c) `__init__` 参数读取区**（`self.gamma_lut = ...` 之后）追加：

```python
        self.ae_enabled = bool(self.get_parameter('ae_enabled').value)
        self.ae_move_speed_thresh = float(
            self.get_parameter('ae_move_speed_thresh').value)
        self.ae_still_speed_thresh = float(
            self.get_parameter('ae_still_speed_thresh').value)
        self.ae_controller = None
        self._moving = True
        self._last_odom_time = None
        self._still_since = None
        self._now = time.monotonic
        self._last_ae_exposure = None
        self._last_ae_gain = None
        if self.ae_enabled:
            self.ae_controller = AdaptiveExposureController(
                target_luma=float(self.get_parameter('ae_target_luma').value),
                deadband=float(self.get_parameter('ae_deadband').value),
                max_step=float(self.get_parameter('ae_max_step').value),
                sat_limit=float(self.get_parameter('ae_sat_limit').value),
                exp_min_us=float(self.get_parameter('ae_exp_min_us').value),
                exp_max_moving_us=float(
                    self.get_parameter('ae_exp_max_moving_us').value),
                exp_max_still_us=float(
                    self.get_parameter('ae_exp_max_still_us').value),
                gain_min=float(self.get_parameter('ae_gain_min').value),
                gain_max=float(self.get_parameter('ae_gain_max').value),
                update_period_s=float(
                    self.get_parameter('ae_update_period_s').value))
```

**(d) `__init__` 中 `self._open_camera()` 之后、publisher 创建之后**追加里程计订阅：

```python
        if self.ae_enabled:
            self.odom_sub = self.create_subscription(
                Odometry, self.get_parameter('odom_topic').value,
                self._on_odom, 10)
```

**(e) `_open_camera()` 中 `self._configure_exposure_and_gain()` 调用之后**追加种子回读：

```python
        if self.ae_enabled:
            exposure = (self._read_optional_float('ExposureTime')
                        or self.exposure_time_us)
            gain = self._read_optional_float('Gain') or self.gain
            self._seed_ae_controller(exposure, gain)
```

**(f) 新增方法**（放在 `_configure_exposure_and_gain` 之后）：

```python
    def _seed_ae_controller(self, exposure_us, gain):
        self.ae_controller.seed(exposure_us, gain)
        self._last_ae_exposure = exposure_us
        self._last_ae_gain = gain
        self.get_logger().info(
            f'AE seeded from hardware: exposure={exposure_us:.0f}us '
            f'gain={gain:.2f}')

    def _on_odom(self, msg):
        speed = math.hypot(msg.twist.twist.linear.x,
                           msg.twist.twist.linear.y)
        now = self._now()
        self._last_odom_time = now
        if speed > self.ae_move_speed_thresh:
            self._moving = True
            self._still_since = None
        elif speed < self.ae_still_speed_thresh:
            if self._still_since is None:
                self._still_since = now
            elif now - self._still_since >= 1.0:
                self._moving = False
        else:
            self._still_since = None

    def _is_moving(self):
        if self._last_odom_time is None:
            return True
        if self._now() - self._last_odom_time > 2.0:
            return True
        return self._moving

    def _write_float_register(self, name, value):
        ret = self.cam.MV_CC_SetFloatValue(name, value)
        if ret != MV_OK:
            self.get_logger().warn(
                f'AE set {name}={value:.1f} failed: {_to_hex(ret)}')

    def _ae_update_from_frame(self, bgr, now_s):
        stats = compute_luma_stats(bgr)
        cmd = self.ae_controller.update(stats, self._is_moving(), now_s)
        if cmd is None:
            return
        if self._last_ae_exposure is None or abs(
                cmd.exposure_us - self._last_ae_exposure) > 1.0:
            self._write_float_register('ExposureTime', cmd.exposure_us)
            self._last_ae_exposure = cmd.exposure_us
        if self._last_ae_gain is None or abs(
                cmd.gain - self._last_ae_gain) > 0.01:
            self._write_float_register('Gain', cmd.gain)
            self._last_ae_gain = cmd.gain
        self.get_logger().info(
            f'AE: mean={stats.mean:.1f} sat={stats.saturated_ratio:.3f} '
            f'moving={self._is_moving()} -> exp={cmd.exposure_us:.0f}us '
            f'gain={cmd.gain:.2f}')
```

注意：`_write_float_register` 不能像 `_set_optional_float` 那样跳过 `value <= 0`——增益 0.0 是合法目标值。

**(g) `capture()` 中**，在 `bgr = self._convert_frame_to_bgr(frame)` 之后、`bgr = self._apply_image_enhancement(bgr)` 之前插入：

```python
            if self.ae_enabled:
                self._ae_update_from_frame(bgr, time.monotonic())
```

（统计必须在 gamma LUT 之前取 raw BGR。）

- [ ] **Step 5: 运行测试确认通过**

```bash
python -m pytest src/sentry_bringup/tests/test_hikrobot_camera_node.py src/sentry_bringup/tests/test_auto_exposure.py -v
```

Expected: 全部 PASS（除 Task 0 已知失败的 `test_hikrobot_launch_uses_adaptive_exposure_for_low_light`）

- [ ] **Step 6: Commit**

```bash
git add src/sentry_bringup/sentry_bringup/hikrobot_camera_node.py src/sentry_bringup/tests/test_hikrobot_camera_node.py
git commit -m "Integrate adaptive exposure into Hikrobot camera node

Subscribes /wheel/odom for motion-linked exposure caps; stats are
measured on raw BGR before the gamma LUT; registers written at most
every 0.4s; ae_enabled=false restores previous behavior."
```

---

### Task 3: launch 参数与既有测试修正（TDD）

**Files:**
- Modify: `src/sentry_bringup/launch/sentry_v2.launch.py`（hikrobot_camera_node 参数块）
- Test: `src/sentry_bringup/tests/test_hikrobot_camera_node.py`

- [ ] **Step 1: 修正既有 launch 测试为新预期（先失败）**

把 `test_hikrobot_camera_node.py` 中的 `test_hikrobot_launch_uses_adaptive_exposure_for_low_light` 整体替换为：

```python
def test_hikrobot_launch_uses_adaptive_exposure():
    repo_root = Path(__file__).parents[2]
    launch_source = (repo_root / 'sentry_bringup' / 'launch' /
                     'sentry_v2.launch.py').read_text(encoding='utf-8')

    assert "'ae_enabled': True" in launch_source
    assert "'exposure_auto': False" in launch_source
    assert "'gain_auto': False" in launch_source
    assert "'ae_exp_max_moving_us': 20000.0" in launch_source
    assert "'ae_exp_max_still_us': 100000.0" in launch_source
    assert "'enable_image_enhancement': True" in launch_source
    assert "'gamma': 2.0" in launch_source
```

运行：

```bash
python -m pytest src/sentry_bringup/tests/test_hikrobot_camera_node.py -v -k launch
```

Expected: FAIL（launch 还是旧值 `'exposure_auto': False`/`'gain_auto': True`/`'gamma': 3.0` 且无 `ae_` 参数）

- [ ] **Step 2: 更新 launch 文件**

修改 `src/sentry_bringup/launch/sentry_v2.launch.py` 的 hikrobot_camera_node 参数块，替换为：

```python
            parameters=[{
                'fps': 5.0,
                'output_width': 640,
                'output_height': 480,
                'frame_id': 'camera',
                'image_topic': '/sentry/camera/image_raw',
                'mvs_common_runenv': '/opt/MVS/lib',
                'mvs_python_path': '/opt/MVS/Samples/aarch64/Python/MvImport',
                'mvs_library_path': '/opt/MVS/lib/aarch64',
                'exposure_time_us': 20000.0,
                'gain': 3.0,
                'exposure_auto': False,
                'gain_auto': False,
                'enable_image_enhancement': True,
                'gamma': 2.0,
                'ae_enabled': True,
                'ae_target_luma': 80.0,
                'ae_exp_min_us': 2000.0,
                'ae_exp_max_moving_us': 20000.0,
                'ae_exp_max_still_us': 100000.0,
                'ae_gain_min': 0.0,
                'ae_gain_max': 12.0,
            }],
```

（`auto_exposure_min_us/max_us`、`auto_gain_min/max` 参数块删除——硬件 AE 已弃用，软件 AE 的 `ae_gain_min/max` 取代。`exposure_time_us`/`gain` 保留为 AE 种子及 `ae_enabled=false` 时的手动回退值。）

- [ ] **Step 3: 运行全部 bringup 测试**

```bash
python -m pytest src/sentry_bringup/tests/ -v
```

Expected: 全部 PASS（包括修正后的 launch 测试）

- [ ] **Step 4: Commit**

```bash
git add src/sentry_bringup/launch/sentry_v2.launch.py src/sentry_bringup/tests/test_hikrobot_camera_node.py
git commit -m "Switch launch to software AE: gain_auto off, gamma 2.0, ae_* params"
```

---

### Task 4: 板端部署与实测验证

**无 TDD——硬件验收。每步都要看实际输出，不许凭预期下结论。**

- [ ] **Step 1: 推送并部署到板端**

```bash
git push origin feat/adaptive-exposure
ssh rdk "cd ~/dev_ws && git fetch origin && git checkout feat/adaptive-exposure && git reset --hard origin/feat/adaptive-exposure"
ssh rdk "bash -lc 'source /opt/ros/humble/setup.bash && cd ~/dev_ws && colcon build --packages-select sentry_bringup'"
```

- [ ] **Step 2: 重启相机节点（AE 开启）**

板端当前已有 web 前端和压缩转码链在跑（`/sentry/camera/image_raw` → `/out/compressed`）。
**只重启相机节点**，不要拉起完整栈（此前完整栈因底盘串口超时失败过）：

```bash
ssh rdk "pkill -f hikrobot_camera_node; sleep 1"
ssh rdk "bash -lc 'source /opt/ros/humble/setup.bash && source ~/dev_ws/install/setup.bash && nohup ros2 run sentry_bringup hikrobot_camera_node --ros-args -p ae_enabled:=true > /tmp/ae_verify.log 2>&1 &'"
```

（独立运行使用代码内默认 AE 参数，与 spec 一致；web 链路的压缩转码节点会继续订阅同名图像话题，前端不断流。）

- [ ] **Step 3: 验证亮场景收敛（当前过曝场景）**

等 10 秒后：

```bash
ssh rdk "grep 'AE:' /tmp/ae_verify.log | tail -10; grep 'AE seeded' /tmp/ae_verify.log"
```

Expected: 日志显示曝光从种子值（约 20000µs）持续下调、饱和占比 sat 降到 0.02 以下、mean 收敛到 80 附近。同时在已打开的浏览器前端（http://10.66.175.106:5000/）确认画面不再整片刷白。

- [ ] **Step 4: 验证硬件回读**

```bash
ssh rdk "grep -A2 'Hardware exposure' /tmp/ae_verify.log | tail -6"
```

Expected: `ExposureAuto=0`、`GainAuto=0`，ExposureTime 与 AE 日志最终值一致。

- [ ] **Step 5: 验证逃生舱（ae_enabled:=false）**

```bash
ssh rdk "pkill -f hikrobot_camera_node; sleep 1"
ssh rdk "bash -lc 'source /opt/ros/humble/setup.bash && source ~/dev_ws/install/setup.bash && nohup ros2 run sentry_bringup hikrobot_camera_node --ros-args -p ae_enabled:=false -p exposure_time_us:=100000.0 > /tmp/ae_off.log 2>&1 &'"
sleep 5
ssh rdk "grep -c 'AE:' /tmp/ae_off.log || echo 'no AE lines (expected)'"
```

Expected: 无 `AE:` 日志，行为与旧版一致。验证后恢复 AE 版本（重复 Step 2 命令）。

- [ ] **Step 6:（可选，需动车）巡航拖影验证**

低速巡航或手动推车移动时观察前端画面与日志：曝光应被钳制在 ≤20000µs（日志 `moving=true`），画面无明显拖影。

---

### Task 5: Code review 与收尾

- [ ] **Step 1: 调用 superpowers:requesting-code-review 对 feat/adaptive-exposure 分支做审查**，处理反馈（superpowers:receiving-code-review）。

- [ ] **Step 2: 更新项目上下文**

按 `.claude/PROJECT_CONTEXT.md` 的渐进式披露要求，把自适应曝光能力、参数表、板端验收结论更新到对应 docs 模块文档与 PROJECT_CONTEXT.md。

- [ ] **Step 3: 更新 PLAN.md**，勾选全部完成项，记录板端实测结论。

- [ ] **Step 4: 最终验证与合并决策**

```bash
python -m pytest src/sentry_bringup/tests/ -v
```

全部通过后，调用 superpowers:finishing-a-development-branch 决定合并/PR。
