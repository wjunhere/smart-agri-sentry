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
    rec.on_stop_trigger(None, 0.0, b'jpeg1')
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
    clock.t += 20.0
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
    column = decoded[300, 320]
    assert column[1] > 100  # G 通道明显抬升


def test_draw_bbox_on_jpeg_passes_through_undecodable():
    from sentry_mission.batch_recorder import draw_bbox_on_jpeg

    assert draw_bbox_on_jpeg(b'not-a-jpeg', [0, 0, 1, 1], 'x') == b'not-a-jpeg'
