# Fixed-Point Diagnosis Stop Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add editable, persisted fixed-point diagnosis stops that pause the active patrol at configured odometry coordinates, show the configured tomato disease result, and then resume the unchanged Nav2 waypoint route.

**Architecture:** The Flask control plane validates and stores `fixed_point_stops` alongside `cruise_speed` in `mission_params.yaml`; the browser edits that list through HTTP. `mission_control_node` snapshots the list at startup, detects the first unhandled point inside its radius only in `PATROL`, and enters the current scan/analyze/resume lifecycle without changing any Nav2 waypoint. An active point overrides only the diagnosis publication produced by that scan.

**Tech Stack:** ROS 2 Humble Python, Flask, PyYAML, Vue 3 template components, pytest.

## Global Constraints

- Use `/odometry/filtered` coordinates and the existing `odom` navigation frame.
- Default fixed-point trigger radius is `0.20` metres; all values are editable before preheat.
- Support only current tomato model classes: `late_blight`, `healthy`, `early_blight`, `bacterial_spot`, `leaf_mold`, `septoria_leaf_spot`, `tomato_yellow_leaf_curl_virus`.
- Do not modify Nav2 waypoints, goals, obstacle avoidance behavior, plant detection thresholds, or real visual diagnosis behavior.
- A point triggers exactly once per autonomous patrol run and never in avoidance states.
- Use test-driven development: run each new test and confirm it fails before production implementation.

---

### Task 1: Persist and validate fixed-point rules

**Files:**
- Modify: `src/sentry_mission/sentry_mission/web_remote_node.py`
- Modify: `src/sentry_mission/tests/test_web_remote_node.py`
- Modify: `src/sentry_mission/config/mission_params.yaml`

**Interfaces:**
- Produces: `_validate_fixed_point_stops(value) -> list[dict]`
- Produces: `_read_mission_params(path: Path) -> dict`, `_write_mission_params(path: Path, params: dict) -> None`
- Produces: `GET|POST /fixed-point-stops` with JSON `fixed_point_stops`.

- [ ] **Step 1: Write failing persistence and validation tests**

```python
def test_fixed_point_stops_round_trip_preserves_cruise_speed(tmp_path):
    config_path = tmp_path / 'mission_params.yaml'
    _write_mission_params(config_path, {
        'cruise_speed': 0.22,
        'fixed_point_stops': [{
            'x': 1.2, 'y': -0.5, 'radius': 0.2,
            'disease_class': 'early_blight',
        }],
    })
    assert _read_mission_params(config_path)['cruise_speed'] == 0.22
    assert _read_mission_params(config_path)['fixed_point_stops'][0]['disease_class'] == 'early_blight'


def test_validate_fixed_point_stops_rejects_unknown_tomato_disease():
    with pytest.raises(ValueError, match='disease_class'):
        _validate_fixed_point_stops([{
            'x': 0, 'y': 0, 'radius': 0.2, 'disease_class': 'unknown',
        }])
```

- [ ] **Step 2: Run test to verify it fails**

Run: `pytest src/sentry_mission/tests/test_web_remote_node.py -q -k fixed_point`

Expected: FAIL because the validation and persistence helpers do not exist.

- [ ] **Step 3: Implement normalized YAML configuration and HTTP endpoints**

```python
TOMATO_DISEASE_CLASSES = frozenset({...})

def _validate_fixed_point_stops(stops):
    if not isinstance(stops, list):
        raise ValueError('fixed_point_stops must be a list')
    normalized = []
    for index, stop in enumerate(stops):
        x, y, radius = float(stop['x']), float(stop['y']), float(stop.get('radius', 0.20))
        disease_class = str(stop['disease_class'])
        if not all(math.isfinite(value) for value in (x, y, radius)) or radius <= 0:
            raise ValueError(f'fixed_point_stops[{index}] has invalid coordinates or radius')
        if disease_class not in TOMATO_DISEASE_CLASSES:
            raise ValueError(f'fixed_point_stops[{index}] has invalid disease_class')
        normalized.append({'x': x, 'y': y, 'radius': radius, 'disease_class': disease_class})
    return normalized
```

Use safe YAML load/dump that preserves `cruise_speed`; route writes both installed and source configuration when both exist, following `/cruise-speed`.

- [ ] **Step 4: Run focused tests to verify they pass**

Run: `pytest src/sentry_mission/tests/test_web_remote_node.py -q -k "fixed_point or cruise_speed"`

Expected: PASS.

### Task 2: Trigger fixed-point stops through the existing mission lifecycle

**Files:**
- Modify: `src/sentry_mission/sentry_mission/mission_control_node.py`
- Modify: `src/sentry_mission/tests/test_mission_control_node.py`

**Interfaces:**
- Consumes: startup parameter `fixed_point_stops: list[dict]`.
- Produces: `_find_unhandled_fixed_point_stop() -> tuple[int, dict] | None`
- Produces: `_accept_fixed_point_stop(index: int, stop: dict, now: float) -> None`

- [ ] **Step 1: Write failing patrol and exactly-once tests**

```python
def test_patrol_enters_stop_state_at_configured_fixed_point(node):
    node.state = 'PATROL'
    node._nav2_ready = True
    node.sending_goal = True
    node.fixed_point_stops = [{'x': 1.0, 'y': -0.5, 'radius': 0.2,
                               'disease_class': 'late_blight'}]
    node.odom_x, node.odom_y = 1.1, -0.5
    with patch.object(node, '_cancel_nav2_task_async'):
        node.tick()
    assert node.state == 'STOPPED'
    assert node.active_fixed_point_disease == 'late_blight'


def test_fixed_point_does_not_trigger_again_in_same_patrol(node):
    node.handled_fixed_point_stops = {0}
    node.fixed_point_stops = [{'x': 1.0, 'y': 0.0, 'radius': 0.2,
                               'disease_class': 'healthy'}]
    node.odom_x, node.odom_y = 1.0, 0.0
    assert node._find_unhandled_fixed_point_stop() is None
```

- [ ] **Step 2: Run test to verify it fails**

Run: `pytest src/sentry_mission/tests/test_mission_control_node.py -q -k fixed_point`

Expected: FAIL because fixed-point mission state does not exist.

- [ ] **Step 3: Implement startup snapshot, detection, and shared stop preparation**

```python
self.declare_parameter('fixed_point_stops', [])
self.fixed_point_stops = self._normalize_fixed_point_stops(
    self.get_parameter('fixed_point_stops').value)
self.handled_fixed_point_stops = set()
self.active_fixed_point_disease = None

def _find_unhandled_fixed_point_stop(self):
    for index, stop in enumerate(self.fixed_point_stops):
        if index not in self.handled_fixed_point_stops and math.hypot(
                self.odom_x - stop['x'], self.odom_y - stop['y']) <= stop['radius']:
            return index, stop
    return None
```

In `PATROL`, retain obstacle priority, then check this helper before plant detection. The accepting method saves the current waypoint index, clears stale fusion state, marks the point handled, stores `active_fixed_point_disease`, cancels Nav2 asynchronously, clears `sending_goal`, and transitions to `STATE_STOPPED`. `_prepare_autonomous_start` clears both the handled set and override disease.

- [ ] **Step 4: Add safety regression tests and run them**

```python
def test_fixed_point_does_not_trigger_outside_radius(node): ...
def test_fixed_point_does_not_trigger_during_obstacle_turn(node): ...
def test_new_auto_patrol_clears_handled_fixed_points(node): ...
```

Run: `pytest src/sentry_mission/tests/test_mission_control_node.py -q`

Expected: PASS.

### Task 3: Publish the configured diagnosis only for successful fixed-point scans

**Files:**
- Modify: `src/sentry_mission/sentry_mission/mission_control_node.py`
- Modify: `src/sentry_mission/tests/test_mission_control_node.py`

**Interfaces:**
- Produces: `_fixed_point_diagnosis(result: Diagnosis) -> Diagnosis`.

- [ ] **Step 1: Write the failing override test**

```python
def test_fixed_point_pipeline_result_publishes_configured_disease_with_bounded_confidence(node):
    node.active_fixed_point_disease = 'early_blight'
    result = Diagnosis()
    result.disease_class = 'healthy'
    overridden = node._fixed_point_diagnosis(result)
    assert overridden.disease_class == 'early_blight'
    assert 0.80 <= overridden.confidence <= 0.90
```

- [ ] **Step 2: Run test to verify it fails**

Run: `pytest src/sentry_mission/tests/test_mission_control_node.py -q -k fixed_point_pipeline`

Expected: FAIL because the override helper does not exist.

- [ ] **Step 3: Implement bounded deterministic override and pipeline use**

Use a pure helper that returns the original result when no active fixed point
exists; otherwise it creates a `Diagnosis`, retains crop/header data, sets the
configured class and a clock-derived `0.80 + (phase * 0.10)` confidence, and
publishes it in the successful pipeline response branch. Do not override a
pipeline failure/timeout. Clear the active override only after `ACTION` has
finished so downstream fusion observes the same event.

- [ ] **Step 4: Run mission tests to verify they pass**

Run: `pytest src/sentry_mission/tests/test_mission_control_node.py -q`

Expected: PASS.

### Task 4: Wire the saved fixed-point list into the launch configuration

**Files:**
- Modify: `src/sentry_bringup/launch/sentry_v2.launch.py`
- Modify: `src/sentry_mission/tests/test_autonomous_cruise_offboard.py`

**Interfaces:**
- Consumes: `src/sentry_mission/config/mission_params.yaml`.
- Produces: `fixed_point_stops` parameter passed to `/mission_control_node`.

- [ ] **Step 1: Write the failing launch contract test**

```python
def test_launch_passes_saved_fixed_point_stops_to_mission_control():
    text = Path('src/sentry_bringup/launch/sentry_v2.launch.py').read_text()
    assert "'fixed_point_stops': mission_params.get('fixed_point_stops', [])" in text
```

- [ ] **Step 2: Run test to verify it fails**

Run: `pytest src/sentry_mission/tests/test_autonomous_cruise_offboard.py -q -k fixed_point`

Expected: FAIL because launch does not forward the configuration.

- [ ] **Step 3: Add the parameter and a default empty list**

Load `fixed_point_stops` from the same YAML dictionary used for saved cruise
speed and pass it only to `mission_control_node`.

- [ ] **Step 4: Run launch contract tests**

Run: `pytest src/sentry_mission/tests/test_autonomous_cruise_offboard.py -q`

Expected: PASS.

### Task 5: Add editable bottom-bar controls and frontend configuration flow

**Files:**
- Modify: `src/sentry_mission/static_v2/components/env-data-bar.js`
- Modify: `src/sentry_mission/static_v2/ros.js`
- Modify: `src/sentry_mission/static_v2/style.css`
- Modify: `src/sentry_mission/tests/test_autonomous_cruise_offboard.py`

**Interfaces:**
- Consumes: `GET|POST /fixed-point-stops`.
- Produces: `store.fixedPointStops`, `store.addFixedPointStop()`,
  `store.removeFixedPointStop(index)`, `store.saveFixedPointStops()`.

- [ ] **Step 1: Write the failing static frontend contract test**

```python
def test_bottom_bar_exposes_scrollable_fixed_point_stop_editor():
    component = Path('src/sentry_mission/static_v2/components/env-data-bar.js').read_text()
    style = Path('src/sentry_mission/static_v2/style.css').read_text()
    assert '固定点停车' in component
    assert 'store.fixedPointStops' in component
    assert component.index('固定点停车') > component.index('视觉逻辑')
    assert '.env-bar-inner' in style and 'overflow-y: auto' in style
```

- [ ] **Step 2: Run test to verify it fails**

Run: `pytest src/sentry_mission/tests/test_autonomous_cruise_offboard.py -q -k fixed_point`

Expected: FAIL because the editor is absent.

- [ ] **Step 3: Implement the data flow and compact controls**

Initialize `store.fixedPointStops = []` and the exact tomato class list. Fetch
the saved list on page load. Provide rows with number inputs for X/Y/radius,
a select control for disease class, a familiar delete icon/button, and add/save
commands. Place the section after the visual-logic section in `.env-bar-inner`;
add narrow, stable grid CSS so the existing vertical scroll reveals the editor
without covering dashboard content.

- [ ] **Step 4: Run frontend contract tests**

Run: `pytest src/sentry_mission/tests/test_autonomous_cruise_offboard.py -q`

Expected: PASS.

### Task 6: Verify integration and deploy to the RDK board

**Files:**
- Modify: no additional source files expected.

- [ ] **Step 1: Run all affected desktop tests**

Run:

```powershell
pytest src/sentry_mission/tests/test_web_remote_node.py -q
pytest src/sentry_mission/tests/test_mission_control_node.py -q
pytest src/sentry_mission/tests/test_autonomous_cruise_offboard.py -q
```

Expected: all PASS.

- [ ] **Step 2: Deploy only changed mission and bringup files**

Copy changed source/config/launch files to `/home/sunrise/dev_ws`, then run:

```bash
source /opt/ros/humble/setup.bash
cd /home/sunrise/dev_ws
colcon build --packages-select sentry_mission sentry_bringup --symlink-install
```

Expected: build finishes successfully.

- [ ] **Step 3: Perform a stationary board validation**

With the robot held stationary or auto mode disabled, inspect loaded
`fixed_point_stops` through the mission node parameter and confirm the web
endpoint round-trip. Do not begin a physical autonomous patrol without the
operator explicitly placing the car in a safe test area and requesting it.

- [ ] **Step 4: Review final diff and report results**

Run: `git diff --check` and affected test commands. Report tests, deployment
result, and any required real-car test instruction.
