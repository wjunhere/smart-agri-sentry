"""In-memory mission detection message recorder.

Collects per-cruise batches of plant-detection snapshots for the web
message center. Pure Python (no ROS) so it can be unit-tested off-board.
Data lives until the hosting node exits.

Thread-safe: mutated from ROS executor callbacks and Flask worker
threads, so every public method takes the internal RLock.
"""

import threading
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
    MAX_BATCHES = 10

    def __init__(self, now=time.time):
        self._now = now
        self._lock = threading.RLock()
        self.batches = []
        self.current = None
        self.unread = 0
        self._seq = 0
        self._mode = None

    def on_mode_change(self, new_mode):
        with self._lock:
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
                        del self.batches[:-self.MAX_BATCHES]
                        self.unread += 1

    def on_stop_trigger(self, bbox, plant_confidence, jpeg_bytes):
        with self._lock:
            if self.current is None or not bbox or len(bbox) != 4:
                return
            self.current.records.append(DetectionRecord(
                seq=len(self.current.records), timestamp=self._now(),
                bbox=list(bbox), plant_confidence=float(plant_confidence),
                jpeg_bytes=jpeg_bytes))

    def on_diagnosis(self, disease_class, confidence):
        with self._lock:
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
        with self._lock:
            self.unread = 0

    def clear(self):
        with self._lock:
            self.batches = []
            self.current = None
            self.unread = 0
            self._mode = None

    def get_snapshot(self, batch_id, seq):
        with self._lock:
            for batch in self.batches:
                if batch.id == batch_id and 0 <= seq < len(batch.records):
                    return batch.records[seq].jpeg_bytes
            return None

    def to_dict(self):
        with self._lock:
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
