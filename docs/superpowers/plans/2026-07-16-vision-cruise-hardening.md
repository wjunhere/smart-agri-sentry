# Vision Cruise Hardening Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make a stopped scan consume a fresh Hikrobot/MIPI frame, load models from stable RDK paths, and resume waypoint patrol only after the matching diagnosis flow has completed or timed out.

**Architecture:** The vision pipeline uses a condition-protected frame sequence and a multi-threaded ROS executor, so the service callback waits for a later camera callback without recursively spinning its own node. Mission control records a time watermark immediately before publishing its scan diagnosis and ignores fusion messages earlier than that watermark. Plant detections count only when PATROL accepts one as a stop trigger. The web node terminates only the independent diagnosis process group it owns.

**Tech Stack:** ROS 2 Humble, Python, rclpy executors/callback groups, pytest.

## Global Constraints

- RDK model paths must be absolute and use `/home/sunrise/dev_ws/models/...`.
- Keep the existing analysis timeout and resume behavior.
- Do not alter ROS message definitions.
- Preserve the user's untracked plan and captured camera frame.

### Task 1: Make Pipeline Frame Waiting Executor-Safe

**Files:**
- Modify: `src/sentry_vision/sentry_vision/vision_pipeline_node.py`
- Test: `src/sentry_vision/tests/test_vision_pipeline.py`

- [ ] Write failing source-contract tests requiring a frame condition, sequence counter, reentrant callback group, multi-threaded executor, and absolute YOLO parameter.
- [ ] Run the tests and confirm they fail against the nested-spin implementation.
- [ ] Replace nested spinning with condition waiting for a frame newer than the captured sequence; run the service under `MultiThreadedExecutor(num_threads=2)`.
- [ ] Set and consume `yolo_model_path`, with the RDK workspace path passed from launch.
- [ ] Run the focused vision tests.

### Task 2: Correlate Mission Fusion With the Current Scan

**Files:**
- Modify: `src/sentry_mission/sentry_mission/mission_control_node.py`
- Test: `src/sentry_mission/tests/test_mission_control_node.py`

- [ ] Write failing tests showing stale fusion is rejected and an accepted plant is counted once when patrol stops.
- [ ] Run the tests and confirm they fail.
- [ ] Add a diagnosis publication timestamp watermark, filter older fusion messages, and increment detections only at the PATROL-to-STOPPED transition.
- [ ] Run the focused mission tests.

### Task 3: Scope Independent Diagnosis Process Cleanup

**Files:**
- Modify: `src/sentry_mission/sentry_mission/web_remote_node.py`
- Test: `src/sentry_mission/tests/test_web_remote_node.py`

- [ ] Write a failing test that verifies stopping an owned process does not invoke global `pkill`.
- [ ] Run the test and confirm it fails.
- [ ] Import `signal` and terminate only the owned process group.
- [ ] Run the focused web-node tests.

### Task 4: Verify Launch Wiring and RDK Deployment

**Files:**
- Modify: `src/sentry_bringup/launch/sentry_v2.launch.py`
- Test: `src/sentry_vision/tests/test_vision_pipeline.py`

- [ ] Add a failing assertion for the absolute pipeline YOLO model parameter in launch.
- [ ] Pass `/home/sunrise/dev_ws/models/yolov8n_crop_weed_bayese_640x640_nv12.bin` to `vision_pipeline_node`.
- [ ] Run syntax checks and all locally runnable focused tests.
- [ ] Synchronize the changed source to RDK, build the affected ROS packages, and run the RDK-focused tests if the ROS environment is healthy.
