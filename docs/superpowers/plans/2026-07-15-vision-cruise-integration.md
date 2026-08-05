# Vision Cruise Integration Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make frontend-started AUTO cruise run the camera plant-detection pipeline, stop on a detected plant, classify disease through the stopped scan pipeline, and resume waypoint cruising without using stale fusion output.

**Architecture:** Keep `mission_control_node` as the orchestration owner. `plant_detector_node` publishes trigger detections during PATROL; `mission_control_node` cancels Nav2, calls `/vision/pipeline/trigger`, publishes the returned `Diagnosis`, waits for fresh fusion or timeout, and resumes the saved waypoint. Frontend stack start passes vision/advisory env flags to the existing RDK script.

**Tech Stack:** ROS 2 Humble, Python ROS nodes, Nav2, `pytest`, `std_srvs/SetBool`, `sentry_interfaces/PipelineTrigger`.

## Global Constraints

- Local Windows workspace has no guaranteed ROS 2 runtime; run unit tests where available and report any environment limitation.
- Do not change firmware or message definitions for this integration.
- Keep `mission_control_node` as the only `/cmd_vel` publisher during AUTO besides frontend MANUAL ownership behavior already present.
- Use TDD for behavior changes.

---

### Task 1: Frontend Stack Starts Vision Cruise Dependencies

**Files:**
- Modify: `src/sentry_mission/sentry_mission/web_remote_node.py`
- Test: `src/sentry_mission/tests/test_web_remote_node.py`

**Interfaces:**
- Consumes: `_stack_script_env(base_env=None) -> dict`
- Produces: environment values `SENTRY_PRESERVE_WEB=1`, `ENABLE_WEB=false`, `ENABLE_VISION=true`, `ENABLE_ADVISORY=true`

- [ ] **Step 1: Write the failing test**

```python
def test_stack_script_env_enables_vision_and_advisory_for_cruise():
    env = _stack_script_env({'ENABLE_VISION': 'false', 'ENABLE_ADVISORY': 'false'})
    assert env['SENTRY_PRESERVE_WEB'] == '1'
    assert env['ENABLE_WEB'] == 'false'
    assert env['ENABLE_VISION'] == 'true'
    assert env['ENABLE_ADVISORY'] == 'true'
```

- [ ] **Step 2: Run test to verify it fails**

Run: `pytest src/sentry_mission/tests/test_web_remote_node.py::test_stack_script_env_enables_vision_and_advisory_for_cruise -q`
Expected: FAIL because `ENABLE_VISION` and `ENABLE_ADVISORY` are missing.

- [ ] **Step 3: Write minimal implementation**

Set the two env keys in `_stack_script_env`.

- [ ] **Step 4: Run test to verify it passes**

Run: `pytest src/sentry_mission/tests/test_web_remote_node.py::test_stack_script_env_enables_vision_and_advisory_for_cruise -q`
Expected: PASS.

### Task 2: Mission Analysis Uses Fresh Fusion Only

**Files:**
- Modify: `src/sentry_mission/sentry_mission/mission_control_node.py`
- Test: `src/sentry_mission/tests/test_mission_control_node.py`

**Interfaces:**
- Consumes: `MissionControlNode.last_fusion`
- Produces: plant-triggered STOPPED transition clears `last_fusion` before the scan result is published

- [ ] **Step 1: Write the failing test**

```python
def test_plant_trigger_clears_stale_fusion_before_analysis(node):
    node.state = 'PATROL'
    node._nav2_ready = True
    node.sending_goal = True
    node.last_fusion = FusionResult()
    node.last_plant = PlantDetection()
    node.last_plant.detected = True
    node.last_plant.confidence = 0.95
    node.last_plant.area_ratio = 0.20
    node.reference_x = 0.0
    node.reference_y = 0.0
    node.odom_x = 1.0
    node.odom_y = 0.0
    with patch.object(node, '_cancel_nav2_task_async'), patch.object(node.navigator, 'isTaskComplete', return_value=False):
        node.tick()
    assert node.state == 'STOPPED'
    assert node.last_fusion is None
```

- [ ] **Step 2: Run test to verify it fails**

Run: `pytest src/sentry_mission/tests/test_mission_control_node.py::test_plant_trigger_clears_stale_fusion_before_analysis -q`
Expected: FAIL because stale fusion remains.

- [ ] **Step 3: Write minimal implementation**

Set `self.last_fusion = None` immediately when PATROL accepts a plant trigger and before entering STOPPED.

- [ ] **Step 4: Run test to verify it passes**

Run: `pytest src/sentry_mission/tests/test_mission_control_node.py::test_plant_trigger_clears_stale_fusion_before_analysis -q`
Expected: PASS.

### Task 3: Avoid Duplicate Continuous Classification In Integrated Stack

**Files:**
- Modify: `src/sentry_bringup/launch/sentry_v2.launch.py`

**Interfaces:**
- Consumes: launch arguments `enable_vision`
- Produces: new launch argument `enable_live_diagnosis=false`; `vision_diagnosis_node` only starts when explicitly enabled

- [ ] **Step 1: Add the launch argument**

Add `DeclareLaunchArgument('enable_live_diagnosis', default_value='false')`.

- [ ] **Step 2: Gate standalone diagnosis**

Change `vision_diagnosis_node` condition from `IfCondition(LaunchConfiguration('enable_vision'))` to `IfCondition(LaunchConfiguration('enable_live_diagnosis'))`.

- [ ] **Step 3: Verify by inspection**

Run: `rg -n "enable_live_diagnosis|vision_diagnosis_node" src/sentry_bringup/launch/sentry_v2.launch.py`
Expected: the argument exists and only the standalone diagnosis node uses it.

### Task 4: Targeted Verification

**Files:**
- Test: `src/sentry_mission/tests/test_web_remote_node.py`
- Test: `src/sentry_mission/tests/test_mission_control_node.py`

**Interfaces:**
- Produces: repeatable local test evidence or a clear ROS-runtime limitation.

- [ ] **Step 1: Run focused mission/web tests**

Run: `pytest src/sentry_mission/tests/test_web_remote_node.py src/sentry_mission/tests/test_mission_control_node.py -q`
Expected: PASS in a ROS-enabled environment.

- [ ] **Step 2: On RDK, run stack check**

Run: `ENABLE_VISION=true ENABLE_ADVISORY=true ./scripts/rdk/start_robot_stack.sh`
Expected: `/plant_detector_node`, `/vision_pipeline_node`, `/fusion_node`, `/mission_control_node`, and `/set_auto_mode` are available.
