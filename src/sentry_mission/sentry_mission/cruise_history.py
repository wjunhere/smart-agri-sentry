"""Persistent, filesystem-backed archive for completed AUTO cruises."""

import json
import shutil
import threading
import time
import uuid
from pathlib import Path


class CruiseHistoryStore:
    """Stores completed cruise batches without exposing filesystem paths."""

    def __init__(self, root, now=time.time, max_batches=30, max_age_days=90,
                 max_images=10):
        self.root = Path(root)
        self.batches_dir = self.root / 'batches'
        self.images_dir = self.root / 'images'
        self.now = now
        self.max_batches = max_batches
        self.max_age_s = max_age_days * 86400
        self.max_images = max_images
        self.lock = threading.RLock()
        self.active = None

    def start(self, crop_type):
        with self.lock:
            if self.active is not None:
                return self.active['id']
            batch_id = f"{int(self.now())}-{uuid.uuid4().hex[:8]}"
            self.active = {
                'id': batch_id, 'started_at': self.now(), 'ended_at': None,
                'end_reason': None, 'crop_type': crop_type or '',
                'records': [], 'fusion_results': [], 'advisories': [],
                'alerts': [], 'risk_points': [], 'dropped_images': 0,
            }
            return batch_id

    def finish(self, reason):
        with self.lock:
            batch, self.active = self.active, None
            if batch is None:
                return None
            batch['ended_at'] = self.now()
            batch['end_reason'] = reason or 'ended'
            self._write_batch(batch)
            self._prune()
            return batch['id']

    def add_detection(self, bbox, plant_confidence, jpeg_bytes):
        with self.lock:
            if self.active is None:
                return None
            seq = len(self.active['records'])
            record = {'seq': seq, 'timestamp': self.now(), 'bbox': list(bbox),
                      'plant_confidence': float(plant_confidence),
                      'disease_class': None, 'disease_confidence': None,
                      'snapshot_seq': None}
            if jpeg_bytes and sum(1 for r in self.active['records']
                                  if r['snapshot_seq'] is not None) < self.max_images:
                record['snapshot_seq'] = seq
                image_path = self.images_dir / self.active['id'] / f'{seq}.jpg'
                image_path.parent.mkdir(parents=True, exist_ok=True)
                image_path.write_bytes(jpeg_bytes)
            elif jpeg_bytes:
                self.active['dropped_images'] += 1
            self.active['records'].append(record)
            return seq

    def add_diagnosis(self, disease_class, confidence):
        with self.lock:
            if self.active is None:
                return
            for record in reversed(self.active['records']):
                if record['disease_class'] is None:
                    record['disease_class'] = disease_class
                    record['disease_confidence'] = float(confidence)
                    return

    def add_event(self, kind, payload):
        with self.lock:
            if self.active is None:
                return
            event = dict(payload)
            event['timestamp'] = self.now()
            if kind == 'fusion':
                self.active['fusion_results'].append(event)
                self.active['risk_points'].append({
                    'timestamp': event['timestamp'], 'risk_score': event.get('risk_score', 0),
                    'alert_level': event.get('alert_level', 0)})
            elif kind == 'advisory':
                self.active['advisories'].append(event)
            elif kind == 'alert':
                self.active['alerts'].append(event)

    def query(self, limit=10, start_at=None, end_at=None, crop_type='', disease=''):
        with self.lock:
            batches = [self._read(path) for path in self.batches_dir.glob('*.json')]
        batches = [b for b in batches if b]
        if start_at is not None:
            batches = [b for b in batches if b['started_at'] >= start_at]
        if end_at is not None:
            batches = [b for b in batches if b['started_at'] <= end_at]
        if crop_type:
            batches = [b for b in batches if b.get('crop_type') == crop_type]
        if disease:
            batches = [b for b in batches if any(r.get('disease_class') == disease
                                                 for r in b.get('records', []))]
        batches.sort(key=lambda b: b['started_at'], reverse=True)
        return [self._public(b) for b in batches[:max(1, min(int(limit), 30))]]

    def clear(self, start_at=None, end_at=None, crop_type='', disease=''):
        matches = self.query(30, start_at, end_at, crop_type, disease)
        with self.lock:
            for batch in matches:
                (self.batches_dir / f"{batch['id']}.json").unlink(missing_ok=True)
                shutil.rmtree(self.images_dir / batch['id'], ignore_errors=True)
        return len(matches)

    def snapshot(self, batch_id, seq):
        path = self.images_dir / batch_id / f'{int(seq)}.jpg'
        return path.read_bytes() if path.is_file() else None

    def _write_batch(self, batch):
        self.batches_dir.mkdir(parents=True, exist_ok=True)
        target = self.batches_dir / f"{batch['id']}.json"
        temp = target.with_suffix('.tmp')
        temp.write_text(json.dumps(batch, ensure_ascii=False), encoding='utf-8')
        temp.replace(target)

    def _prune(self):
        batches = [self._read(path) for path in self.batches_dir.glob('*.json')]
        batches = sorted((b for b in batches if b), key=lambda b: b['started_at'], reverse=True)
        cutoff = self.now() - self.max_age_s
        for index, batch in enumerate(batches):
            if index >= self.max_batches or batch['started_at'] < cutoff:
                (self.batches_dir / f"{batch['id']}.json").unlink(missing_ok=True)
                shutil.rmtree(self.images_dir / batch['id'], ignore_errors=True)

    @staticmethod
    def _read(path):
        try:
            return json.loads(path.read_text(encoding='utf-8'))
        except (OSError, ValueError):
            return None

    @staticmethod
    def _public(batch):
        out = dict(batch)
        for record in out.get('records', []):
            seq = record.get('snapshot_seq')
            record['snapshot_url'] = (
                f"/api/history/batches/{out['id']}/snapshot/{seq}" if seq is not None else None)
            record.pop('snapshot_seq', None)
        return out
