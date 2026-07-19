# 舵机随巡航换行自动翻转 实现计划

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 巡航到达换行航点时，mission_control_node 按航段几何自动判定并翻转舵机（0°=右 / 180°=左），同时给植株检测加冷却窗口。

**Architecture:** 方案 A——在 `mission_control_node` 的 waypoint 完成回调中做"长航段反平行判定"并发布 `/sentry/servo_cmd`；`min_row_segment_length` 支持 0=auto 自动推导；冷却逻辑挂在现有 `_should_trigger_scan` 上。spec：`docs/superpowers/specs/2026-07-19-servo-row-switch-flip-design.md`（v6，评审 Approved）。

**Tech Stack:** ROS2 humble (rclpy), pytest + unittest.mock, launch Python。

**关键背景（实现者须知）：**

- **分支**：按 AGENT.md 要求，在分支 `feat/servo-row-switch-flip` 上开发（spec 已提交在 main）
- **测试环境**：本机 Windows **没有 rclpy**，pytest 跑不起来（AGENT.md:13）。工作流 = 本地写代码+测试并 commit → push 到 GitHub（wjunhere/smart-agri-sentry）→ 板端 `ssh rdk`（`~/dev_ws`）pull → `colcon build` → 板端跑 pytest。本地唯一可做的验证：`python -m pytest src/sentry_mission/tests/test_autonomous_cruise_offboard.py`（纯 Python）和 launch 文件 `ast.parse` 语法自检
- 板端测试命令：`ssh rdk 'cd ~/dev_ws && git pull && colcon build --packages-select sentry_mission && source install/setup.bash && python3 -m pytest src/sentry_mission/tests/ -v'`
- 航点是 dict 列表：`{'x': float, 'y': float, 'yaw': float}`，用 `wp[i]['x']` 访问
- `ServoCmd` 消息（`src/sentry_interfaces/msg/ServoCmd.msg`）：`uint8 pitch` + `uint8 yaw`；`package.xml:16` 已有 `sentry_interfaces` 依赖，**setup.py/package.xml 无需改动**
- 测试 fixture 参考 `src/sentry_mission/tests/test_mission_control_node.py:12-24`（patch BasicNavigator 后实例化节点）
- `tick()` 顶部已有局部变量 `now = self.get_clock().now().nanoseconds / 1e9`
- 节点内里程计状态：`self.odom_x` / `self.odom_y`；扫描去抖状态：`self.reference_x` / `self.reference_y` / `self.has_scan_reference`
- 按 AGENT.md 要求维护根目录 `PLAN.md`：计划阶段初始化、每完成一步即时勾选

---

### Task 0: 分支与 PLAN.md 初始化

- [ ] **Step 1:** `git checkout -b feat/servo-row-switch-flip`
- [ ] **Step 2:** 重置 `PLAN.md`，写入本计划的 Task 1-5 复选框列表

---

### Task 1: 声明参数、初始化成员、auto 推导、ServoCmd publisher

**Files:**
- Modify: `src/sentry_mission/sentry_mission/mission_control_node.py`（import :26-27；declare_parameter 块尾 :119 后；get_parameter 块尾 :187 后；waypoints 加载后 :200 后；publishers :244 后；De-duplication 成员 :305 后）
- Test: `src/sentry_mission/tests/test_servo_auto_flip.py`（新建）

- [ ] **Step 1: 写测试（auto 推导 + 默认值，本地写好，板端跑）**

新建 `src/sentry_mission/tests/test_servo_auto_flip.py`：

```python
"""Tests for servo auto-flip on row switch."""

import math
import pytest
import rclpy
from unittest.mock import patch

from sentry_mission.mission_control_node import MissionControlNode


@pytest.fixture(scope='module')
def ros_context():
    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.fixture
def node(ros_context):
    with patch('sentry_mission.mission_control_node.BasicNavigator'):
        n = MissionControlNode()
        yield n
        n.destroy_node()


def _serpentine():
    """Layout-(b) two-row serpentine, robot starts at origin."""
    return [
        {'x': 2.5, 'y': 0.0, 'yaw': 1.5708},
        {'x': 2.5, 'y': 1.0, 'yaw': 3.1416},
        {'x': 0.0, 'y': 1.0, 'yaw': 3.1416},
    ]


def test_default_params_and_init_members(node):
    assert node.enable_servo_auto_flip is False
    assert node.servo_yaw_right == 0
    assert node.servo_yaw_left == 180
    assert node.servo_pitch_hold == 0
    assert node.flip_heading_threshold == pytest.approx(2.09)
    assert node.servo_flip_cooldown_sec == pytest.approx(8.0)
    assert node.servo_flip_cooldown_distance == pytest.approx(0.8)
    assert node._servo_side == 'right'
    assert node._servo_flip_time is None
    assert node._servo_flip_position is None
    assert node._mission_start_pose == (0.0, 0.0)


def test_auto_derive_min_segment_length(node):
    node.waypoints = _serpentine()
    # segments: 1.0 (corner), 2.5 (row) -> (1.0 + 2.5) / 2
    assert node._derive_min_segment_length() == pytest.approx(1.75)


def test_auto_derive_disabled_with_fewer_than_two_waypoints(node):
    node.waypoints = [{'x': 1.0, 'y': 0.0, 'yaw': 0.0}]
    assert node._derive_min_segment_length() is None


def test_manual_min_segment_length_overrides_auto(node):
    node._min_seg_len_manual = 1.2
    node.waypoints = _serpentine()
    assert node._derive_min_segment_length() == pytest.approx(1.2)
```

- [ ] **Step 2: 实现**

2a. import（:26-27 sentry_interfaces.msg 导入改为）：

```python
from sentry_interfaces.msg import (
    PlantDetection, FusionResult, MissionStatus, Diagnosis, ObstacleInfo,
    ServoCmd)
```

2b. declare_parameter（加在 :119 `lidar_base_yaw` 之后）：

```python
        self.declare_parameter('enable_servo_auto_flip', False)
        self.declare_parameter('servo_yaw_right', 0)
        self.declare_parameter('servo_yaw_left', 180)
        self.declare_parameter('servo_pitch_hold', 0)
        self.declare_parameter('flip_heading_threshold', 2.09)
        self.declare_parameter('min_row_segment_length', 0.0)
        self.declare_parameter('servo_flip_cooldown_sec', 8.0)
        self.declare_parameter('servo_flip_cooldown_distance', 0.8)
```

2c. get_parameter（加在 :187 `self.lidar_base_yaw = ...` 之后）：

```python
        self.enable_servo_auto_flip = self.get_parameter(
            'enable_servo_auto_flip').value
        self.servo_yaw_right = self.get_parameter('servo_yaw_right').value
        self.servo_yaw_left = self.get_parameter('servo_yaw_left').value
        self.servo_pitch_hold = self.get_parameter('servo_pitch_hold').value
        self.flip_heading_threshold = self.get_parameter(
            'flip_heading_threshold').value
        self._min_seg_len_manual = self.get_parameter(
            'min_row_segment_length').value
        self.servo_flip_cooldown_sec = self.get_parameter(
            'servo_flip_cooldown_sec').value
        self.servo_flip_cooldown_distance = self.get_parameter(
            'servo_flip_cooldown_distance').value
```

2d. waypoints 加载块之后（:200 之后，`:202 self.current_wp_idx = 0` 之前）：

```python
        self.min_row_segment_length = self._derive_min_segment_length()
```

2e. publishers（:244 `self.pub_diag = ...` 之后）：

```python
        self.pub_servo_cmd = self.create_publisher(
            ServoCmd, '/sentry/servo_cmd', 10)
```

2f. De-duplication 成员（:305 `self.has_scan_reference = False` 之后）：

```python
        self._servo_side = 'right'
        self._servo_flip_time = None
        self._servo_flip_position = None
        self._mission_start_pose = (0.0, 0.0)
```

2g. 新方法 `_derive_min_segment_length`（放在 `_load_fixed_point_stops` 之后）：

```python
    def _derive_min_segment_length(self):
        """Effective min row segment length: manual override or auto-derived.

        Auto: (shortest + longest waypoint segment) / 2, which falls inside
        the open interval (row_spacing, row_length) for serpentine paths.
        Returns None when fewer than 2 waypoints (auto-flip disabled).
        """
        if self._min_seg_len_manual > 0.0:
            return self._min_seg_len_manual
        segments = []
        for i in range(1, len(self.waypoints)):
            dx = self.waypoints[i]['x'] - self.waypoints[i - 1]['x']
            dy = self.waypoints[i]['y'] - self.waypoints[i - 1]['y']
            segments.append(math.hypot(dx, dy))
        if not segments:
            self.get_logger().warn(
                'servo auto-flip: fewer than 2 waypoints, disabled')
            return None
        return (min(segments) + max(segments)) / 2.0
```

- [ ] **Step 3: 本地语法自检**

Run: `python -c "import ast; ast.parse(open('src/sentry_mission/sentry_mission/mission_control_node.py', encoding='utf-8').read())"`
Expected: 无输出

- [ ] **Step 4: Commit**

```bash
git add src/sentry_mission/sentry_mission/mission_control_node.py src/sentry_mission/tests/test_servo_auto_flip.py
git commit -m "feat(mission): add servo auto-flip params, init members and segment-length derivation"
```

（测试在 Task 4 完成后于板端统一执行，见 Task 4 Step 5）

---

### Task 2: `_maybe_flip_servo` 几何判定 + tick 钩子 + 起点位姿记录

**Files:**
- Modify: `src/sentry_mission/sentry_mission/mission_control_node.py`（`_send_next_waypoint` :327-353；tick 完成回调 :816-821；新方法放 `_derive_min_segment_length` 之后）
- Test: `src/sentry_mission/tests/test_servo_auto_flip.py`

- [ ] **Step 1: 写测试（几何判定）**

追加到 `test_servo_auto_flip.py`：

```python
def _arm(node, idx):
    node.enable_servo_auto_flip = True
    node.waypoints = _serpentine()
    node.min_row_segment_length = node._derive_min_segment_length()
    node.current_wp_idx = idx


def test_flip_at_first_row_end(node):
    """Default waypoints: arrival at wp0 (first row end) flips right->left."""
    _arm(node, 1)
    node._mission_start_pose = (0.0, 0.0)
    with patch.object(node.pub_servo_cmd, 'publish') as mock_pub:
        node._maybe_flip_servo(now=100.0)
    assert mock_pub.called
    msg = mock_pub.call_args[0][0]
    assert msg.yaw == 180
    assert node._servo_side == 'left'
    assert node._servo_flip_time == 100.0
    assert node.has_scan_reference is True


def test_no_flip_at_corner_end(node):
    """Arrival at wp1 (corner end, short segment) must not flip."""
    _arm(node, 2)
    with patch.object(node.pub_servo_cmd, 'publish') as mock_pub:
        node._maybe_flip_servo(now=100.0)
    mock_pub.assert_not_called()
    assert node._servo_side == 'right'


def test_no_flip_at_final_waypoint(node):
    """Arrival at last waypoint: no following segment, no flip."""
    _arm(node, 3)
    with patch.object(node.pub_servo_cmd, 'publish') as mock_pub:
        node._maybe_flip_servo(now=100.0)
    mock_pub.assert_not_called()


def test_toggle_back_on_second_row_switch(node):
    """Two consecutive row switches toggle side back to right."""
    node.enable_servo_auto_flip = True
    node.waypoints = [
        {'x': 2.5, 'y': 0.0, 'yaw': 0.0},
        {'x': 2.5, 'y': 1.0, 'yaw': 0.0},
        {'x': 0.0, 'y': 1.0, 'yaw': 0.0},
        {'x': 0.0, 'y': 2.0, 'yaw': 0.0},
        {'x': 2.5, 'y': 2.0, 'yaw': 0.0},
    ]
    node.min_row_segment_length = node._derive_min_segment_length()
    node._mission_start_pose = (0.0, 0.0)
    with patch.object(node.pub_servo_cmd, 'publish') as mock_pub:
        node.current_wp_idx = 1
        node._maybe_flip_servo(now=1.0)
        node.current_wp_idx = 3
        node._maybe_flip_servo(now=2.0)
    assert mock_pub.call_count == 2
    assert mock_pub.call_args_list[0][0][0].yaw == 180
    assert mock_pub.call_args_list[1][0][0].yaw == 0
    assert node._servo_side == 'right'


def test_no_flip_on_l_turn(node):
    """L-shaped path (delta ~= 90 deg) is not a row switch."""
    node.enable_servo_auto_flip = True
    node.waypoints = _serpentine()
    node.min_row_segment_length = node._derive_min_segment_length()
    # Start pose makes first segment head +y instead of +x
    node._mission_start_pose = (2.5, -2.5)
    node.current_wp_idx = 1
    with patch.object(node.pub_servo_cmd, 'publish') as mock_pub:
        node._maybe_flip_servo(now=1.0)
    mock_pub.assert_not_called()


def test_no_flip_on_straight_midpoint(node):
    """Collinear waypoint mid-row (delta ~= 0) is not a row switch."""
    node.enable_servo_auto_flip = True
    node.waypoints = [
        {'x': 2.5, 'y': 0.0, 'yaw': 0.0},
        {'x': 5.0, 'y': 0.0, 'yaw': 0.0},
        {'x': 5.0, 'y': 1.0, 'yaw': 0.0},
        {'x': 0.0, 'y': 1.0, 'yaw': 0.0},
    ]
    node.min_row_segment_length = 1.0  # manual: rows 2.5/5.0, corner 1.0
    node._mission_start_pose = (0.0, 0.0)
    node.current_wp_idx = 1
    with patch.object(node.pub_servo_cmd, 'publish') as mock_pub:
        node._maybe_flip_servo(now=1.0)
    mock_pub.assert_not_called()


def test_no_flip_when_disabled(node):
    _arm(node, 1)
    node.enable_servo_auto_flip = False
    node._mission_start_pose = (0.0, 0.0)
    with patch.object(node.pub_servo_cmd, 'publish') as mock_pub:
        node._maybe_flip_servo(now=1.0)
    mock_pub.assert_not_called()


def test_no_flip_when_segment_length_underivable(node):
    _arm(node, 1)
    node.min_row_segment_length = None
    node._mission_start_pose = (0.0, 0.0)
    with patch.object(node.pub_servo_cmd, 'publish') as mock_pub:
        node._maybe_flip_servo(now=1.0)
    mock_pub.assert_not_called()


def test_mission_start_pose_recorded_on_first_waypoint(node):
    """_send_next_waypoint records odom as mission start pose for idx=0."""
    node.waypoints = _serpentine()
    node._nav2_ready = True
    node.state = 'PATROL'
    node.current_wp_idx = 0
    node.odom_x = 0.3
    node.odom_y = -0.1
    node._send_next_waypoint()
    assert node._mission_start_pose == (0.3, -0.1)


def test_mission_rerun_still_flips(node):
    """After mission rerun (idx reset to 0), wp0 arrival flips again."""
    _arm(node, 1)
    node._mission_start_pose = (0.0, 0.0)
    node._servo_side = 'left'  # side from previous run
    with patch.object(node.pub_servo_cmd, 'publish') as mock_pub:
        node._maybe_flip_servo(now=1.0)
    assert mock_pub.called
    assert mock_pub.call_args[0][0].yaw == 0  # toggled back to right


def test_tick_calls_maybe_flip_servo_on_waypoint_reached(node):
    """Waypoint completion in tick() triggers the flip check."""
    from nav2_simple_commander.robot_navigator import TaskResult
    node.state = 'PATROL'
    node._nav2_ready = True
    node.current_wp_idx = 0
    node.sending_goal = True
    node.last_goal_sent_time = 0.0
    node.waypoints = _serpentine()
    with patch.object(node.navigator, 'isTaskComplete', return_value=True), \
         patch.object(node.navigator, 'getResult',
                      return_value=TaskResult.SUCCEEDED), \
         patch.object(node, '_maybe_flip_servo') as mock_flip:
        node.tick()
    mock_flip.assert_called_once()
    assert node.current_wp_idx == 1
```

注意 `test_mission_rerun_still_flips` 也揭示了 spec §5 的已知限制行为：重跑时 `_servo_side` 不归零（舵机物理位置也保持），toggle 语义与实际舵机位置一致——这是正确行为，测试将其固化。

- [ ] **Step 2: 实现**

2a. `_send_next_waypoint` 中记录起点（`:336 if self.current_wp_idx >= len(self.waypoints):` 块之后、`:338 wp = self.waypoints[...]` 之前）：

```python
        if self.current_wp_idx == 0:
            self._mission_start_pose = (self.odom_x, self.odom_y)
```

2b. 新方法 `_maybe_flip_servo`（放在 `_derive_min_segment_length` 之后）：

```python
    def _maybe_flip_servo(self, now: float) -> None:
        """Flip the servo when a row-switch U-turn is detected.

        Layout (b) only: row-end and corner must be separate waypoints.
        The completed segment must be a long patrol segment; skip short
        corner segments ahead and compare headings of the two long
        segments. Anti-parallel (>= flip_heading_threshold) means the
        next long segment is the new row traversed in reverse, so the
        plant row is now on the other side: toggle the servo.
        """
        if not self.enable_servo_auto_flip:
            return
        if self.min_row_segment_length is None:
            return
        idx = self.current_wp_idx
        wp = self.waypoints

        if idx == 1:
            x0, y0 = self._mission_start_pose
        else:
            x0 = wp[idx - 2]['x']
            y0 = wp[idx - 2]['y']

        dx0 = wp[idx - 1]['x'] - x0
        dy0 = wp[idx - 1]['y'] - y0
        if math.hypot(dx0, dy0) < self.min_row_segment_length:
            return

        j = idx
        while j < len(wp) and math.hypot(
                wp[j]['x'] - wp[j - 1]['x'],
                wp[j]['y'] - wp[j - 1]['y']) < self.min_row_segment_length:
            j += 1
        if j >= len(wp):
            return

        h_done = math.atan2(dy0, dx0)
        h_next = math.atan2(wp[j]['y'] - wp[j - 1]['y'],
                            wp[j]['x'] - wp[j - 1]['x'])
        delta = (h_next - h_done + math.pi) % (2.0 * math.pi) - math.pi
        if abs(delta) < self.flip_heading_threshold:
            return

        self._servo_side = 'left' if self._servo_side == 'right' else 'right'
        yaw = (self.servo_yaw_left if self._servo_side == 'left'
               else self.servo_yaw_right)
        msg = ServoCmd()
        msg.yaw = int(yaw)
        msg.pitch = int(self.servo_pitch_hold)
        self.pub_servo_cmd.publish(msg)
        self.get_logger().info(
            f'Row switch detected (delta={math.degrees(delta):.1f} deg), '
            f'servo flipped to {self._servo_side} (yaw={yaw})')
        self._servo_flip_time = now
        self._servo_flip_position = (self.odom_x, self.odom_y)
        self.reference_x = self.odom_x
        self.reference_y = self.odom_y
        self.has_scan_reference = True
```

2c. tick 钩子（:818-819 `self.get_logger().info(f'Reached waypoint ...')` 之后、`:820 if self.current_wp_idx < len(...)` 之前）：

```python
                    self._maybe_flip_servo(now)
```

- [ ] **Step 3: 本地语法自检**

Run: `python -c "import ast; ast.parse(open('src/sentry_mission/sentry_mission/mission_control_node.py', encoding='utf-8').read())"`
Expected: 无输出

- [ ] **Step 4: Commit**

```bash
git add src/sentry_mission/sentry_mission/mission_control_node.py src/sentry_mission/tests/test_servo_auto_flip.py
git commit -m "feat(mission): flip servo on serpentine row switch via segment geometry"
```

---

### Task 3: 检测冷却窗口

**Files:**
- Modify: `src/sentry_mission/sentry_mission/mission_control_node.py:513-522`（`_should_trigger_scan`）
- Test: `src/sentry_mission/tests/test_servo_auto_flip.py`

- [ ] **Step 1: 写测试**

追加到 `test_servo_auto_flip.py`：

```python
def test_cooldown_suppresses_scan_trigger(node):
    """Within cooldown window (time AND distance), scan trigger is blocked."""
    node._servo_flip_time = node.get_clock().now().nanoseconds / 1e9
    node._servo_flip_position = (0.0, 0.0)
    node.odom_x = 0.1
    node.odom_y = 0.0
    node.has_scan_reference = True
    node.reference_x = 0.0
    node.reference_y = 0.0
    assert node._should_trigger_scan() is False


def test_cooldown_expires_after_distance(node):
    """Driving past the cooldown distance ends the window."""
    node._servo_flip_time = node.get_clock().now().nanoseconds / 1e9
    node._servo_flip_position = (0.0, 0.0)
    node.odom_x = 1.0  # > servo_flip_cooldown_distance (0.8)
    node.odom_y = 0.0
    node.has_scan_reference = True
    node.reference_x = 0.0
    node.reference_y = 0.0
    assert node._should_trigger_scan() is True
    assert node._servo_flip_time is None


def test_cooldown_expires_after_time(node):
    """Cooldown ends after servo_flip_cooldown_sec even without moving."""
    node._servo_flip_time = (
        node.get_clock().now().nanoseconds / 1e9
        - node.servo_flip_cooldown_sec - 1.0)
    node._servo_flip_position = (0.0, 0.0)
    node.odom_x = 0.0
    node.odom_y = 0.0
    node.has_scan_reference = True
    node.reference_x = 0.0
    node.reference_y = 0.0
    # min_resume_distance not met (0 < 0.5), so the distance check still
    # returns False; what we verify is the cooldown window was cleared
    assert node._should_trigger_scan() is False
    assert node._servo_flip_time is None
```

- [ ] **Step 2: 实现**

`_should_trigger_scan`（:513）开头插入（插在 docstring 之后、`if not self.has_scan_reference:` 之前）：

```python
        if self._servo_flip_time is not None:
            now = self.get_clock().now().nanoseconds / 1e9
            if now - self._servo_flip_time < self.servo_flip_cooldown_sec:
                dx = self.odom_x - self._servo_flip_position[0]
                dy = self.odom_y - self._servo_flip_position[1]
                if math.hypot(dx, dy) < self.servo_flip_cooldown_distance:
                    return False
            self._servo_flip_time = None
```

- [ ] **Step 3: 本地语法自检 + Commit**

```bash
python -c "import ast; ast.parse(open('src/sentry_mission/sentry_mission/mission_control_node.py', encoding='utf-8').read())"
git add src/sentry_mission/sentry_mission/mission_control_node.py src/sentry_mission/tests/test_servo_auto_flip.py
git commit -m "feat(mission): suppress scan triggers during servo flip cooldown window"
```

---

### Task 4: launch 参数接线 + servo 初始角修正 + 板端全量测试

**Files:**
- Modify: `src/sentry_bringup/launch/sentry_v2.launch.py`（launch argument 声明区 :50 附近；mission_control_node parameters :350-364）
- Modify: `src/sentry_servo/config/servo_config.yaml:12`

- [ ] **Step 1: 写静态检查测试**

追加到 `test_servo_auto_flip.py`：

```python
def test_launch_wires_servo_auto_flip_params():
    import pathlib
    launch = pathlib.Path(__file__).parents[2].joinpath(
        'sentry_bringup', 'launch', 'sentry_v2.launch.py')
    text = launch.resolve().read_text(encoding='utf-8')
    for key in ('enable_servo_auto_flip', 'servo_yaw_right', 'servo_yaw_left',
                'servo_pitch_hold', 'flip_heading_threshold',
                'min_row_segment_length', 'servo_flip_cooldown_sec',
                'servo_flip_cooldown_distance'):
        assert key in text, f'{key} not wired in sentry_v2.launch.py'


def test_servo_config_initial_angle_is_zero():
    import pathlib
    import yaml
    cfg = pathlib.Path(__file__).parents[2].joinpath(
        'sentry_servo', 'config', 'servo_config.yaml')
    data = yaml.safe_load(cfg.resolve().read_text(encoding='utf-8'))
    assert data['servos']['yaw']['initial_angle'] == 0
```

（`pathlib.Path(__file__).parents[2]` = `src/`，直接 join 包名，**不要**加 `'..'`）

- [ ] **Step 2: 实现**

2a. `sentry_v2.launch.py` launch argument（`enable_servo` 声明附近，格式与现有声明一致）：

```python
        DeclareLaunchArgument(
            'enable_servo_auto_flip',
            default_value='false',
            description='Flip servo on serpentine row switch'),
```

2b. mission_control_node parameters 字典（:363 `'max_scan_shots': 3,` 之后）：

```python
                'enable_servo_auto_flip': ParameterValue(
                    LaunchConfiguration('enable_servo_auto_flip'),
                    value_type=bool),
                'servo_yaw_right': 0,
                'servo_yaw_left': 180,
                'servo_pitch_hold': 0,
                'flip_heading_threshold': 2.09,
                'min_row_segment_length': 0.0,
                'servo_flip_cooldown_sec': 8.0,
                'servo_flip_cooldown_distance': 0.8,
```

2c. `servo_config.yaml:12`：`initial_angle: 67.5` → `initial_angle: 0`

- [ ] **Step 3: 本地语法自检 + 纯 Python 测试**

Run: `python -c "import ast; ast.parse(open('src/sentry_bringup/launch/sentry_v2.launch.py', encoding='utf-8').read())"`
Run: `python -m pytest src/sentry_mission/tests/test_autonomous_cruise_offboard.py -q`
Expected: 无输出；纯 Python 测试 passed

- [ ] **Step 4: Commit + push 分支**

```bash
git add src/sentry_bringup/launch/sentry_v2.launch.py src/sentry_servo/config/servo_config.yaml src/sentry_mission/tests/test_servo_auto_flip.py
git commit -m "feat(bringup): wire servo auto-flip params in launch; set servo initial angle to 0 (right)"
git push -u origin feat/servo-row-switch-flip
```

- [ ] **Step 5: 板端拉取 + 构建 + 全量测试**

```bash
ssh rdk 'cd ~/dev_ws && git fetch origin && git checkout feat/servo-row-switch-flip && git pull && colcon build --packages-select sentry_mission sentry_servo sentry_bringup && source install/setup.bash && python3 -m pytest src/sentry_mission/tests/ -v'
```

Expected: 全部 passed（含既有测试无回归）。如有失败，本地修复后 commit + push，板端 `git pull` 后重跑。

---

### Task 5: 板端实车验证（手动步骤，无代码）

**前置：** Task 4 Step 5 全部测试通过；舵机信号线接 32 脚（PWM6），舵机独立供电共地。

- [ ] **Step 1:** 启动：`ENABLE_VISION=true bash scripts/rdk/start_robot_stack.sh`，launch 参数加 `enable_servo:=true enable_servo_auto_flip:=true`（start 脚本目前不透传这两个参数，可直接 `ros2 launch sentry_bringup sentry_v2.launch.py enable_servo:=true enable_servo_auto_flip:=true` 或先给 start 脚本加透传——若加透传，记得同步把新参数写进脚本并测试）
- [ ] **Step 2:** 默认 waypoints.yaml 验证：到达 (2.5, 0) 时观察舵机翻转到 180°（朝左），日志出现 `Row switch detected ... servo flipped to left`
- [ ] **Step 3:** 到达 (2.5, 1)（拐角）与 (0, 1)（终点）时舵机不再动作
- [ ] **Step 4:** 转弯+冷却窗口内确认不误触发停车诊断
- [ ] **Step 5:** 记录验证结果；如有偏差调整 `flip_heading_threshold` / 冷却参数
- [ ] **Step 6:** 全部通过后按 AGENT.md 更新 `.claude/PROJECT_CONTEXT.md` 相关模块文档，分支合并回 main
