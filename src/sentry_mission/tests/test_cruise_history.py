from sentry_mission.cruise_history import CruiseHistoryStore


class Clock:
    def __init__(self, value=1_000_000):
        self.value = value

    def __call__(self):
        return self.value


def test_finished_batch_persists_and_exposes_snapshot_url(tmp_path):
    clock = Clock()
    store = CruiseHistoryStore(tmp_path, now=clock)
    store.start('tomato')
    store.add_detection([1, 2, 3, 4], 0.9, b'jpeg')
    store.add_diagnosis('late_blight', 0.8)
    store.add_event('fusion', {'risk_score': 0.7, 'alert_level': 2})
    store.finish('manual')

    batch = store.query(crop_type='tomato', disease='late_blight')[0]
    assert batch['end_reason'] == 'manual'
    assert batch['records'][0]['snapshot_url'].endswith('/snapshot/0')
    assert store.snapshot(batch['id'], 0) == b'jpeg'
    assert batch['risk_points'][0]['risk_score'] == 0.7


def test_retention_keeps_newest_batches_and_limits_images(tmp_path):
    clock = Clock()
    store = CruiseHistoryStore(tmp_path, now=clock, max_batches=2, max_images=1)
    for i in range(3):
        store.start('tomato')
        store.add_detection([0, 0, 1, 1], 0.5, b'one')
        store.add_detection([0, 0, 1, 1], 0.5, b'two')
        store.finish('manual')
        clock.value += 1

    batches = store.query(limit=30)
    assert len(batches) == 2
    assert sum(r['snapshot_url'] is not None for r in batches[0]['records']) == 1
