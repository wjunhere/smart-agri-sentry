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


def test_active_batch_is_checkpointed_for_restart_recovery(tmp_path):
    clock = Clock()
    store = CruiseHistoryStore(tmp_path, now=clock)
    batch_id = store.start('tomato')
    store.add_detection([0, 0, 1, 1], 0.8, b'jpeg')
    store.add_diagnosis('early_blight', 0.7)

    # A new process can read the checkpoint even before the cruise is ended.
    recovered = CruiseHistoryStore(tmp_path, now=clock)
    batch = recovered.query(disease='early_blight')[0]
    assert batch['id'] == batch_id
    assert batch['ended_at'] is None
    assert recovered.snapshot(batch_id, 0) == b'jpeg'


class RosFloat:
    def __init__(self, value):
        self.value = value

    def __float__(self):
        return float(self.value)

    def item(self):
        return self.value


def test_ros_scalar_bbox_does_not_break_active_checkpoint(tmp_path):
    store = CruiseHistoryStore(tmp_path, now=Clock())
    store.start('tomato')
    store.add_detection([RosFloat(1.5), RosFloat(2.5), RosFloat(3.5), RosFloat(4.5)],
                        RosFloat(0.9), b'jpeg')

    batch = store.query()[0]
    assert batch['records'][0]['bbox'] == [1.5, 2.5, 3.5, 4.5]
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
