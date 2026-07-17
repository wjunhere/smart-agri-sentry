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
            stamp = time.strftime('%m-%d %H:%M', time.localtime(self._now()))
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
