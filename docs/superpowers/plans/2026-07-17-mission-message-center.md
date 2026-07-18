# 巡航检测消息推送 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 前端顶部加消息中心：巡航批次内检测到的植株快照（带检测框）+ 病害类别，存 web_remote_node 内存，支持未读角标与一键清理。

**Architecture:** 方案 A（spec 已批准）：`BatchRecorder` 纯 Python 类管批次/记录/未读（off-board TDD）；web_remote_node 订阅 `/vision/plant_detected`、`/vision/diagnosis`，在 PATROL→STOPPED 边沿用 `/out/compressed` 最新帧画框存快照；Flask 暴露 `/api/messages*`；static_v2 加铃铛按钮 + message-center modal。

**Tech Stack:** Python (Flask, cv2), pytest（mock ROS2），Vue 3 全局组件（无构建）。

**Spec:** `docs/superpowers/specs/2026-07-17-mission-message-center-design.md`

**关键已有集成点：**
- `web_remote_node.py:246-253` `latest_camera_jpeg` + `_on_camera_image`（快照帧来源）
- `web_remote_node.py:282-303` `on_mission_status`（PATROL→STOPPED 边沿插入点；MANUAL 分支已有）
- `web_remote_node.py:305-324` `set_mode_auto`（批次开/关挂钩点）、`:348-360` `emergency_stop`
- `web_remote_node.py:601-616` `get_status`（加 `message_unread`）、`:624+` `_get_app`（Flask 路由模式）
- `ros.js:715-730` `refreshStackStatus`（3s 轮询 `/status`，同步 unread）；`index.html:33-34` modal 注册模式；`app.js:17-32` 组件注册
- 测试 mock 模式：`tests/test_web_remote_node.py:17-62`
- 本地 `D:/anaconda/python.exe` 有 cv2 4.13 + pytest；运行测试用 `D:/anaconda/python.exe -m pytest`

---

### Task 1: BatchRecorder 纯逻辑 TDD

**Files:**
- Create: `src/sentry_mission/sentry_mission/batch_recorder.py`
- Test: `src/sentry_mission/tests/test_batch_recorder.py`

- [ ] **Step 1: 写失败测试**

```python
"""Tests for the pure-Python mission batch recorder."""

import sys
from pathlib import Path

PKG_ROOT = Path(__file__).resolve().parents[1]
if str(PKG_ROOT) not in sys.path:
    sys.path.insert(0, str(PKG_ROOT))

from sentry_mission.batch_recorder import BatchRecorder


class FakeClock:
    def __init__(self):
        self.t = 1000.0
    def __call__(self):
        return self.t


def make_recorder():
    clock = FakeClock()
    return BatchRecorder(now=clock), clock


def test_auto_mode_opens_batch():
    rec, _ = make_recorder()
    rec.on_mode_change('AUTO')
    assert rec.current is not None
    assert rec.current.id == 1
    assert rec.current.name.startswith('批次#1 · ')


def test_leaving_auto_closes_batch_and_counts_unread():
    rec, _ = make_recorder()
    rec.on_mode_change('AUTO')
    rec.on_stop_trigger([0.1, 0.1, 0.5, 0.5], 0.9, b'jpeg1')
    rec.on_mode_change('MANUAL')
    assert rec.current is None
    assert len(rec.batches) == 1
    assert rec.unread == 1


def test_empty_batch_is_discarded():
    rec, _ = make_recorder()
    rec.on_mode_change('AUTO')
    rec.on_mode_change('MANUAL')
    assert rec.batches == []
    assert rec.unread == 0


def test_stop_trigger_ignored_without_open_batch():
    rec, _ = make_recorder()
    rec.on_stop_trigger([0.1, 0.1, 0.5, 0.5], 0.9, b'jpeg1')
    assert rec.batches == []


def test_stop_trigger_ignored_without_bbox():
    rec, _ = make_recorder()
    rec.on_mode_change('AUTO')
    rec.on_stop_trigger(None, 0.0, b'jpeg1')  # 固定点停车无检测框
    assert rec.current.records == []


def test_diagnosis_fills_latest_unfilled_record():
    rec, clock = make_recorder()
    rec.on_mode_change('AUTO')
    rec.on_stop_trigger([0.1, 0.1, 0.5, 0.5], 0.9, b'jpeg1')
    clock.t += 3.0
    rec.on_diagnosis('early_blight', 0.87)
    record = rec.current.records[0]
    assert record.disease_class == 'early_blight'
    assert record.disease_confidence == 0.87


def test_diagnosis_ignored_after_window():
    rec, clock = make_recorder()
    rec.on_mode_change('AUTO')
    rec.on_stop_trigger([0.1, 0.1, 0.5, 0.5], 0.9, b'jpeg1')
    clock.t += 20.0  # > 15s 窗口
    rec.on_diagnosis('early_blight', 0.87)
    assert rec.current.records[0].disease_class is None


def test_mark_read_and_clear():
    rec, _ = make_recorder()
    rec.on_mode_change('AUTO')
    rec.on_stop_trigger([0.1, 0.1, 0.5, 0.5], 0.9, b'jpeg1')
    rec.on_mode_change('MANUAL')
    assert rec.unread == 1
    rec.mark_read()
    assert rec.unread == 0
    rec.clear()
    assert rec.batches == []
    assert rec.unread == 0


def test_to_dict_newest_batch_first_with_snapshot_url():
    rec, _ = make_recorder()
    for _ in range(2):
        rec.on_mode_change('AUTO')
        rec.on_stop_trigger([0.1, 0.1, 0.5, 0.5], 0.9, b'jpeg')
        rec.on_mode_change('MANUAL')
    data = rec.to_dict()
    assert [b['id'] for b in data['batches']] == [2, 1]
    record = data['batches'][0]['records'][0]
    assert record['snapshot_url'] == '/api/messages/2/0/snapshot'
    assert data['unread'] == 2


def test_get_snapshot_returns_stored_bytes():
    rec, _ = make_recorder()
    rec.on_mode_change('AUTO')
    rec.on_stop_trigger([0.1, 0.1, 0.5, 0.5], 0.9, b'jpeg1')
    rec.on_mode_change('MANUAL')
    assert rec.get_snapshot(1, 0) == b'jpeg1'
    assert rec.get_snapshot(1, 9) is None
    assert rec.get_snapshot(99, 0) is None
```

- [ ] **Step 2: 跑测试确认失败**

Run: `D:/anaconda/python.exe -m pytest src/sentry_mission/tests/test_batch_recorder.py -q`
Expected: FAIL `ModuleNotFoundError: sentry_mission.batch_recorder`

- [ ] **Step 3: 实现 batch_recorder.py**

```python
"""In-memory mission detection message recorder.

Collects per-cruise batches of plant-detection snapshots for the web
message center. Pure Python (no ROS) so it can be unit-tested off-board.
Data lives until the hosting node exits.
"""

import time
from dataclasses import dataclass, field


@dataclass
class DetectionRecord:
    seq: int
    timestamp: float
    bbox: list
    plant_confidence: float
    jpeg_bytes: bytes
    disease_class: str = None
    disease_confidence: float = None


@dataclass
class MissionBatch:
    id: int
    name: str
    started_at: float
    ended_at: float = None
    records: list = field(default_factory=list)


class BatchRecorder:
    DIAGNOSIS_WINDOW_S = 15.0

    def __init__(self, now=time.time):
        self._now = now
        self.batches = []
        self.current = None
        self.unread = 0
        self._seq = 0
        self._mode = None

    def on_mode_change(self, new_mode):
        old, self._mode = self._mode, new_mode
        if old != 'AUTO' and new_mode == 'AUTO':
            self._seq += 1
            stamp = time.strftime('%m-%d %H:%M',
                                  time.localtime(self._now()))
            self.current = MissionBatch(
                id=self._seq, name=f'批次#{self._seq} · {stamp}',
                started_at=self._now())
        elif old == 'AUTO' and new_mode != 'AUTO':
            batch, self.current = self.current, None
            if batch is not None:
                batch.ended_at = self._now()
                if batch.records:
                    self.batches.append(batch)
                    self.unread += 1

    def on_stop_trigger(self, bbox, plant_confidence, jpeg_bytes):
        if self.current is None or bbox is None:
            return
        self.current.records.append(DetectionRecord(
            seq=len(self.current.records), timestamp=self._now(),
            bbox=list(bbox), plant_confidence=float(plant_confidence),
            jpeg_bytes=jpeg_bytes))

    def on_diagnosis(self, disease_class, confidence):
        if self.current is None:
            return
        for record in reversed(self.current.records):
            if record.disease_class is not None:
                break
            if self._now() - record.timestamp <= self.DIAGNOSIS_WINDOW_S:
                record.disease_class = disease_class
                record.disease_confidence = float(confidence)
            return

    def mark_read(self):
        self.unread = 0

    def clear(self):
        self.batches = []
        self.current = None
        self.unread = 0
        self._mode = None

    def get_snapshot(self, batch_id, seq):
        for batch in self.batches:
            if batch.id == batch_id and 0 <= seq < len(batch.records):
                return batch.records[seq].jpeg_bytes
        return None

    def to_dict(self):
        return {
            'unread': self.unread,
            'batches': [self._batch_to_dict(b)
                        for b in reversed(self.batches)],
        }

    @staticmethod
    def _batch_to_dict(batch):
        return {
            'id': batch.id,
            'name': batch.name,
            'started_at': batch.started_at,
            'ended_at': batch.ended_at,
            'records': [{
                'seq': r.seq,
                'timestamp': r.timestamp,
                'disease_class': r.disease_class,
                'disease_confidence': r.disease_confidence,
                'plant_confidence': r.plant_confidence,
                'snapshot_url': f'/api/messages/{batch.id}/{r.seq}/snapshot',
            } for r in batch.records],
        }
```

- [ ] **Step 4: 跑测试确认通过**

Run: `D:/anaconda/python.exe -m pytest src/sentry_mission/tests/test_batch_recorder.py -q`
Expected: 10 passed

- [ ] **Step 5: Commit**

```bash
git add src/sentry_mission/sentry_mission/batch_recorder.py src/sentry_mission/tests/test_batch_recorder.py
git commit -m "Add BatchRecorder for mission detection messages"
```

---

### Task 2: 快照画框 draw_bbox_on_jpeg TDD

**Files:**
- Modify: `src/sentry_mission/sentry_mission/batch_recorder.py`（追加函数）
- Test: `src/sentry_mission/tests/test_batch_recorder.py`（追加）

- [ ] **Step 1: 追加失败测试**

```python
def test_draw_bbox_on_jpeg_burns_rectangle():
    import cv2
    import numpy as np
    from sentry_mission.batch_recorder import draw_bbox_on_jpeg

    img = np.zeros((480, 640, 3), dtype=np.uint8)
    ok, enc = cv2.imencode('.jpg', img)
    assert ok
    out = draw_bbox_on_jpeg(enc.tobytes(), [0.5, 0.5, 0.75, 0.75],
                            'Plant 90%')
    decoded = cv2.imdecode(np.frombuffer(out, dtype=np.uint8),
                           cv2.IMREAD_COLOR)
    assert decoded is not None
    # bbox 左边框 x=320 附近应有绿色像素（BGR ~ (16,185,129)）
    column = decoded[300, 320]
    assert column[1] > 100  # G 通道明显抬升


def test_draw_bbox_on_jpeg_passes_through_undecodable():
    from sentry_mission.batch_recorder import draw_bbox_on_jpeg

    assert draw_bbox_on_jpeg(b'not-a-jpeg', [0, 0, 1, 1], 'x') == b'not-a-jpeg'
```

- [ ] **Step 2: 跑测试确认失败**

Run: `D:/anaconda/python.exe -m pytest src/sentry_mission/tests/test_batch_recorder.py -q -k draw_bbox`
Expected: FAIL `ImportError: cannot import name 'draw_bbox_on_jpeg'`

- [ ] **Step 3: 实现（追加到 batch_recorder.py 末尾）**

```python
def draw_bbox_on_jpeg(jpeg_bytes, bbox, label):
    """Burn a normalized bbox + label onto a JPEG frame (#10B981 green)."""
    import cv2
    import numpy as np

    arr = np.frombuffer(jpeg_bytes, dtype=np.uint8)
    img = cv2.imdecode(arr, cv2.IMREAD_COLOR)
    if img is None:
        return jpeg_bytes
    h, w = img.shape[:2]
    x1, y1 = int(bbox[0] * w), int(bbox[1] * h)
    x2, y2 = int(bbox[2] * w), int(bbox[3] * h)
    color = (16, 185, 129)  # BGR of #10B981
    cv2.rectangle(img, (x1, y1), (x2, y2), color, 3)
    cv2.putText(img, label, (x1, max(20, y1 - 8)),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
    ok, enc = cv2.imencode('.jpg', img)
    return enc.tobytes() if ok else jpeg_bytes
```

- [ ] **Step 4: 跑测试确认通过**

Run: `D:/anaconda/python.exe -m pytest src/sentry_mission/tests/test_batch_recorder.py -q`
Expected: 12 passed

- [ ] **Step 5: Commit**

```bash
git add src/sentry_mission/sentry_mission/batch_recorder.py src/sentry_mission/tests/test_batch_recorder.py
git commit -m "Add bbox snapshot drawing for detection messages"
```

---

### Task 3: web_remote_node 接线 + HTTP API TDD

**Files:**
- Modify: `src/sentry_mission/sentry_mission/web_remote_node.py`
- Test: `src/sentry_mission/tests/test_web_remote_node.py`（mock fixture 加 `PlantDetection`/`Diagnosis`，追加测试）

- [ ] **Step 1: 追加失败测试**

mock fixture（`mock_ros2`）的 `modules['sentry_interfaces.msg']` 处追加：

```python
    PlantDetection = type('PlantDetection', (), {})
    modules['sentry_interfaces.msg'].PlantDetection = PlantDetection
    Diagnosis = type('Diagnosis', (), {})
    modules['sentry_interfaces.msg'].Diagnosis = Diagnosis
```

测试函数（注意 `sys.modules.pop('sentry_mission.web_remote_node', None)` 已在其它测试里用到则保持一致；直接复用模块级 import 即可）：

```python
def _make_wired_node():
    from sentry_mission.web_remote_node import WebRemoteNode
    from sentry_mission.batch_recorder import BatchRecorder

    node = WebRemoteNode.__new__(WebRemoteNode)
    node.batch_recorder = BatchRecorder()
    node.latest_plant = None
    node.latest_plant_time = 0.0
    node.latest_camera_jpeg = b'frame'
    node._last_mission_state = None
    import threading
    node.lock = threading.Lock()
    node.get_logger = mock.MagicMock()
    return node


def _status(state):
    return types.SimpleNamespace(state=state, total_wps=3, current_wp_idx=0)


def test_patrol_to_stopped_records_snapshot_with_fresh_plant():
    node = _make_wired_node()
    node.batch_recorder.on_mode_change('AUTO')
    node.latest_plant = ([0.1, 0.1, 0.5, 0.5], 0.9)
    node.latest_plant_time = __import__('time').time()

    node.on_mission_status(_status('PATROL'))
    node.on_mission_status(_status('STOPPED'))

    assert len(node.batch_recorder.current.records) == 1


def test_patrol_to_stopped_without_plant_records_nothing():
    node = _make_wired_node()
    node.batch_recorder.on_mode_change('AUTO')

    node.on_mission_status(_status('PATROL'))
    node.on_mission_status(_status('STOPPED'))  # 固定点停车，无检测框

    assert node.batch_recorder.current.records == []


def test_stale_plant_detection_is_ignored():
    node = _make_wired_node()
    node.batch_recorder.on_mode_change('AUTO')
    node.latest_plant = ([0.1, 0.1, 0.5, 0.5], 0.9)
    node.latest_plant_time = __import__('time').time() - 5.0  # 5s 前

    node.on_mission_status(_status('PATROL'))
    node.on_mission_status(_status('STOPPED'))

    assert node.batch_recorder.current.records == []


def test_diagnosis_sentinel_class_id_ignored():
    node = _make_wired_node()
    node.batch_recorder.on_mode_change('AUTO')
    node.latest_plant = ([0.1, 0.1, 0.5, 0.5], 0.9)
    node.latest_plant_time = __import__('time').time()
    node.on_mission_status(_status('PATROL'))
    node.on_mission_status(_status('STOPPED'))

    node._on_diagnosis(types.SimpleNamespace(
        class_id=254, disease_class='', confidence=0.0))
    assert node.batch_recorder.current.records[0].disease_class is None

    node._on_diagnosis(types.SimpleNamespace(
        class_id=1, disease_class='early_blight', confidence=0.8))
    assert node.batch_recorder.current.records[0].disease_class == 'early_blight'


def test_get_status_includes_message_unread():
    node = _make_wired_node()
    node.mode = 'MANUAL'
    node.linear = 0.0
    node.angular = 0.0
    node.last_cmd_time = __import__('time').time()
    node.TIMEOUT = 0.5
    node.mode_srv = mock.MagicMock()
    node.frontend_started_auto = False
    node.completion_stop_started = False
    node.stack_ready = False
    node.cruise_speed = 0.18
    node.vision_inference_mode = 'triggered'
    node.batch_recorder.unread = 2

    assert node.get_status()['message_unread'] == 2
```

- [ ] **Step 2: 跑测试确认失败**

Run: `D:/anaconda/python.exe -m pytest src/sentry_mission/tests/test_web_remote_node.py -q -k "patrol or stale or sentinel or message_unread"`
Expected: FAIL（`_make_wired_node` 里 `on_mission_status` 不记录快照 / `get_status` 无 `message_unread`）

- [ ] **Step 3: 实现 web_remote_node 修改**

a) import 区（`:21` 附近）：

```python
from sentry_interfaces.msg import MissionStatus, PlantDetection, Diagnosis
from sentry_mission.batch_recorder import BatchRecorder, draw_bbox_on_jpeg
```

b) `__init__`（`:246-248` 相机订阅之后）追加：

```python
        self.latest_plant = None
        self.latest_plant_time = 0.0
        self._last_mission_state = None
        self.batch_recorder = BatchRecorder()
        self.plant_sub = self.create_subscription(
            PlantDetection, '/vision/plant_detected',
            self._on_plant_detected, 10)
        self.diagnosis_sub = self.create_subscription(
            Diagnosis, '/vision/diagnosis', self._on_diagnosis, 10)
```

c) 新增方法（放 `_on_camera_image` 后）：

```python
    def _on_plant_detected(self, msg):
        with self.lock:
            if msg.detected:
                self.latest_plant = (list(msg.bbox), float(msg.confidence))
                self.latest_plant_time = time.time()
            else:
                self.latest_plant = None

    def _on_diagnosis(self, msg):
        if getattr(msg, 'class_id', 0) == 254:
            return
        self.batch_recorder.on_diagnosis(
            msg.disease_class, float(msg.confidence))

    def _record_detection_snapshot(self):
        with self.lock:
            plant = self.latest_plant
            plant_time = self.latest_plant_time
            jpeg = self.latest_camera_jpeg
        if plant is None or (time.time() - plant_time) > 2.0:
            return  # 固定点停车且无有效检测框 -> 不记录
        if not jpeg:
            self.get_logger().warn('Detection snapshot skipped: no frame')
            return
        bbox, conf = plant
        try:
            snap = draw_bbox_on_jpeg(jpeg, bbox, f'Plant {conf * 100:.0f}%')
        except Exception as exc:
            self.get_logger().warn(f'bbox draw failed: {exc}')
            snap = jpeg
        self.batch_recorder.on_stop_trigger(bbox, conf, snap)
```

d) `on_mission_status` 开头（`:282` 注释后）插入边沿检测：

```python
        state = getattr(msg, 'state', '')
        prev_state = self._last_mission_state
        self._last_mission_state = state
        if prev_state == 'PATROL' and state == 'STOPPED':
            self._record_detection_snapshot()
        if state == 'MANUAL':
            self.batch_recorder.on_mode_change('MANUAL')
```

（原 `if getattr(msg, 'state', '') == 'MANUAL':` 分支保留其余逻辑不变。）

e) `set_mode_auto`（`:315` 的 `with self.lock:` 块后）追加：

```python
        self.batch_recorder.on_mode_change('AUTO' if auto else 'MANUAL')
```

f) `emergency_stop`（`:360` `self.get_logger().warn(...)` 前）追加：

```python
        self.batch_recorder.on_mode_change('MANUAL')
```

g) `get_status` 返回 dict 追加一行：

```python
                'message_unread': self.batch_recorder.unread,
```

h) Flask 路由（`_get_app` 内，`/camera/capture` 路由之后）：

```python
    @_app.route('/api/messages')
    def api_messages():
        return jsonify(node.batch_recorder.to_dict())

    @_app.route('/api/messages/<int:batch_id>/<int:seq>/snapshot')
    def api_message_snapshot(batch_id, seq):
        from flask import Response
        jpeg = node.batch_recorder.get_snapshot(batch_id, seq)
        if jpeg is None:
            return jsonify({'error': 'not found'}), 404
        return Response(jpeg, mimetype='image/jpeg')

    @_app.route('/api/messages/read', methods=['POST'])
    def api_messages_read():
        node.batch_recorder.mark_read()
        return jsonify({'status': 'ok'})

    @_app.route('/api/messages/clear', methods=['POST'])
    def api_messages_clear():
        node.batch_recorder.clear()
        return jsonify({'status': 'ok'})
```

注意 `/<path:filename>` 通配路由在 `/api/...` 之前注册会抢先匹配——把 `/api/messages*` 路由放在 `v2_static` **之前**定义（Flask 按注册顺序匹配，`/api/messages` 无对应静态文件，但 `/api/messages/1/0/snapshot` 会被 `v2_static` 捕获，所以必须先注册）。

- [ ] **Step 4: 跑测试确认通过**

Run: `D:/anaconda/python.exe -m pytest src/sentry_mission/tests/test_web_remote_node.py src/sentry_mission/tests/test_batch_recorder.py -q`
Expected: 全部 passed（含已有测试不回归）

- [ ] **Step 5: Commit**

```bash
git add src/sentry_mission/sentry_mission/web_remote_node.py src/sentry_mission/tests/test_web_remote_node.py
git commit -m "Wire detection message recording into web_remote_node + HTTP API"
```

---

### Task 4: 前端消息中心

**Files:**
- Create: `src/sentry_mission/static_v2/components/message-center.js`
- Modify: `src/sentry_mission/static_v2/components/top-bar.js`、`ros.js`、`index.html`、`app.js`、`style.css`

- [ ] **Step 1: ros.js — store 字段 + API 函数**

store 初始化（`_rawWaypoints: []` 行后）追加：

```javascript
  messageUnread: 0,
  messageBatches: [],
  showMessages: false,
```

`refreshStackStatus` 的 `.then(data => {...})` 内追加：

```javascript
      store.messageUnread = Number(data.message_unread || 0);
```

文件末尾追加：

```javascript
// ── Mission message center ──
function fetchMessages() {
  return fetch('/api/messages')
    .then(resp => resp.json())
    .then(data => {
      store.messageBatches = data.batches || [];
      store.messageUnread = Number(data.unread || 0);
      return data;
    })
    .catch(() => null);
}

store.openMessages = async function() {
  await fetchMessages();
  store.showMessages = true;
  store.messageUnread = 0;
  fetch('/api/messages/read', { method: 'POST' }).catch(() => {});
};

store.clearMessages = async function() {
  if (!window.confirm('确定清空所有巡航批次的快照与记录？')) return;
  await fetch('/api/messages/clear', { method: 'POST' }).catch(() => {});
  store.messageBatches = [];
  store.messageUnread = 0;
};

store.formatMsgTime = function(ts) {
  return new Date(ts * 1000).toLocaleTimeString('zh-CN', { hour12: false });
};
```

- [ ] **Step 2: message-center.js 组件**

```javascript
const MessageCenter = {
  template: `
  <div class="modal-overlay" v-if="visible" @click.self="close">
    <div class="modal" style="max-width: 640px;">
      <h2>巡航消息</h2>
      <div class="msg-actions">
        <button class="btn btn-pause" @click="store.clearMessages()"
                :disabled="store.messageBatches.length === 0">一键清理</button>
        <button class="btn btn-resume" @click="close">关闭</button>
      </div>
      <div v-if="store.messageBatches.length === 0" class="muted"
           style="text-align:center;padding:24px">
        暂无巡航检测记录
      </div>
      <div v-for="batch in store.messageBatches" :key="batch.id" class="msg-batch">
        <div class="msg-batch-header">
          {{ batch.name }} · {{ batch.records.length }} 株
        </div>
        <div v-for="rec in batch.records" :key="rec.seq" class="msg-record"
             @click="preview = rec.snapshot_url">
          <img class="msg-thumb" :src="rec.snapshot_url" loading="lazy" />
          <div class="msg-record-info">
            <div class="msg-disease">{{ rec.disease_class || '未知' }}</div>
            <div class="muted">
              检测 {{ (rec.plant_confidence * 100).toFixed(0) }}%
              <template v-if="rec.disease_confidence !== null">
                · 诊断 {{ (rec.disease_confidence * 100).toFixed(0) }}%
              </template>
            </div>
            <div class="muted">{{ store.formatMsgTime(rec.timestamp) }}</div>
          </div>
        </div>
      </div>
      <div class="modal-overlay" v-if="preview" @click.self="preview = null">
        <img class="msg-preview" :src="preview" @click="preview = null" />
      </div>
    </div>
  </div>`,
  props: { visible: Boolean },
  emits: ['close'],
  data() { return { preview: null }; },
  methods: { close() { this.preview = null; this.$emit('close'); } },
};
```

- [ ] **Step 3: top-bar.js 加铃铛按钮**（拍摄按钮 `</button>` 后插入）

```html
    <button class="message-btn" @click="store.openMessages()" title="巡航消息">
      <svg width="16" height="16" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2">
        <path d="M18 8a6 6 0 10-12 0c0 7-3 9-3 9h18s-3-2-3-9"/>
        <path d="M13.7 21a2 2 0 01-3.4 0"/>
      </svg>
      <span class="msg-badge" v-if="store.messageUnread > 0">{{ store.messageUnread }}</span>
    </button>
```

- [ ] **Step 4: index.html 注册**

`<alert-detail-modal></alert-detail-modal>` 行后插入：

```html
    <message-center :visible="store.showMessages" @close="store.showMessages = false"></message-center>
```

`<script src="components/waypoint-editor.js"></script>` 行后插入：

```html
  <script src="components/message-center.js"></script>
```

- [ ] **Step 5: app.js 注册组件**

`app.component('WaypointEditor', WaypointEditor);` 行后插入：

```javascript
app.component('MessageCenter', MessageCenter);
```

- [ ] **Step 6: style.css 追加样式**（文件末尾）

```css
/* ── Message center ── */
.message-btn {
  position: relative;
  display: flex; align-items: center; justify-content: center;
  width: 32px; height: 32px;
  background: var(--bg-card);
  border: 1px solid var(--border);
  border-radius: var(--radius-pill); color: var(--text); cursor: pointer;
}
.msg-badge {
  position: absolute; top: -6px; right: -6px;
  min-width: 16px; height: 16px; padding: 0 4px;
  background: var(--red); color: #fff; border-radius: 8px;
  font-size: 10px; line-height: 16px; text-align: center; font-weight: 600;
}
.msg-actions { display: flex; gap: 8px; margin-bottom: 12px; }
.msg-batch { margin-bottom: 16px; }
.msg-batch-header {
  font-weight: 600; font-size: 13px; padding: 6px 0;
  border-bottom: 1px solid var(--border); margin-bottom: 8px;
}
.msg-record {
  display: flex; gap: 10px; padding: 6px; border-radius: var(--radius-pill);
  cursor: pointer;
}
.msg-record:hover { background: var(--bg-hover); }
.msg-thumb { width: 96px; height: 72px; object-fit: cover; border-radius: var(--radius); }
.msg-record-info { display: flex; flex-direction: column; gap: 2px; font-size: 12px; }
.msg-disease { font-weight: 600; font-size: 14px; }
.msg-preview {
  max-width: 90vw; max-height: 85vh; border-radius: var(--radius-pill);
  cursor: zoom-out;
}
```

- [ ] **Step 7: Commit**

```bash
git add src/sentry_mission/static_v2/
git commit -m "Add message center UI: bell button, batch modal, clear-all"
```

---

### Task 5: 验证、部署与 code review

- [ ] **Step 1: 本地全量测试**

Run: `D:/anaconda/python.exe -m pytest src/sentry_mission/tests/ -q`
Expected: 全过（test_preprocessing 若为既有失败则确认与本次无关）

- [ ] **Step 2: 推送 + 板端拉取部署**

```bash
git push -u origin feat/mission-message-center
ssh rdk "cd ~/dev_ws && git fetch origin && git checkout feat/mission-message-center && git pull --ff-only"
ssh rdk "bash -lc 'source /opt/ros/humble/setup.bash && cd ~/dev_ws && colcon build --packages-select sentry_mission'"
```

板端 checkout 前确认板端工作区干净（`git status`），有本地改动先 stash。

- [ ] **Step 3: 板端实测**

1. 前端预热 → 启动巡航
2. 巡航结束（完成或手动停）→ 铃铛出现红色角标
3. 点开消息 → 批次记录含快照（带绿框）+ 病害类别
4. 再跑一圈 → 出现第二个批次
5. 一键清理 → 列表清空、角标消失
6. 无检测框的固定点停车不产生记录

- [ ] **Step 4: code review**

按 `superpowers:requesting-code-review` 派遣 reviewer（BASE=分支起点，HEAD=最新），Critical/Important 必修。

- [ ] **Step 5: 文档与收尾**

- `docs/ROS2.md` 2.5 节 API 表追加 4 个 `/api/messages*` 端点
- `PLAN.md` 勾选本功能任务块
- 调用 `superpowers:finishing-a-development-branch` 呈现合并选项

## 验证

- `D:/anaconda/python.exe -m pytest src/sentry_mission/tests/test_batch_recorder.py src/sentry_mission/tests/test_web_remote_node.py -q` 全过
- 板端巡航 → 消息按钮角标 → 快照带检测框 → 清理生效
