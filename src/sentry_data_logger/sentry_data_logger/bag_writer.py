import json
import os
import shutil
import threading
import time


def _topic_type_str(msg):
    cls = msg.__class__
    module = cls.__module__
    # Convert 'sentry_interfaces.msg.FusionResult' -> 'sentry_interfaces/msg/FusionResult'
    parts = module.split('.')
    return f"{'/'.join(parts)}/{cls.__name__}"


class BagWriter:
    """Wrapper around rosbag2_py.SequentialWriter with JSON fallback."""

    def __init__(self, base_dir, split_duration_sec=900, split_max_size_mb=1024):
        self.base_dir = base_dir
        self.split_duration_sec = split_duration_sec
        self.split_max_size_mb = split_max_size_mb
        self.split_max_size_bytes = split_max_size_mb * 1024 * 1024
        self._writer = None
        self._current_dir = None
        self._start_time = 0.0
        self._topics = set()
        self._lock = threading.Lock()
        self._json_fallback = False
        self._json_file = None

    def open(self):
        try:
            import rosbag2_py
            from rclpy.serialization import serialize_message
            self._rosbag2_py = rosbag2_py
            self._serialize_message = serialize_message
            self._new_bag()
        except Exception:
            self._json_fallback = True
            self._new_json()

    def _new_bag(self):
        self.close()
        timestamp = time.strftime('%Y%m%d_%H%M%S')
        self._current_dir = os.path.join(self.base_dir, timestamp)
        os.makedirs(self._current_dir, exist_ok=True)
        storage_options = self._rosbag2_py.StorageOptions(
            uri=self._current_dir,
            storage_id='sqlite3',
        )
        converter_options = self._rosbag2_py.ConverterOptions(
            input_serialization_format='cdr',
            output_serialization_format='cdr',
        )
        self._writer = self._rosbag2_py.SequentialWriter()
        self._writer.open(storage_options, converter_options)
        self._topics = set()
        self._start_time = time.time()

    def _new_json(self):
        self.close()
        timestamp = time.strftime('%Y%m%d_%H%M%S')
        self._current_dir = os.path.join(self.base_dir, timestamp)
        os.makedirs(self._current_dir, exist_ok=True)
        json_path = os.path.join(self._current_dir, 'events.jsonl')
        self._json_file = open(json_path, 'a')
        self._start_time = time.time()

    def _should_split(self):
        elapsed = time.time() - self._start_time
        if elapsed >= self.split_duration_sec:
            return True
        if self._current_dir and os.path.exists(self._current_dir):
            size = sum(
                os.path.getsize(os.path.join(dp, f))
                for dp, dn, filenames in os.walk(self._current_dir)
                for f in filenames
            )
            if size >= self.split_max_size_bytes:
                return True
        return False

    def write(self, topic, msg, timestamp_nanoseconds):
        with self._lock:
            if self._should_split():
                if self._json_fallback:
                    self._new_json()
                else:
                    self._new_bag()

            if self._json_fallback:
                record = {
                    'topic': topic,
                    'timestamp_ns': int(timestamp_nanoseconds),
                    'type': _topic_type_str(msg),
                }
                self._json_file.write(json.dumps(record) + '\n')
                self._json_file.flush()
                return

            topic_type = _topic_type_str(msg)
            if topic not in self._topics:
                self._writer.create_topic(
                    self._rosbag2_py.TopicMetadata(
                        name=topic,
                        type=topic_type,
                        serialization_format='cdr',
                    ))
                self._topics.add(topic)
            self._writer.write(
                topic,
                self._serialize_message(msg),
                int(timestamp_nanoseconds),
            )

    def snapshot_critical(self, target_dir, metadata=None):
        with self._lock:
            if not self._current_dir or not os.path.exists(self._current_dir):
                return
            os.makedirs(target_dir, exist_ok=True)
            for item in os.listdir(self._current_dir):
                src = os.path.join(self._current_dir, item)
                dst = os.path.join(target_dir, item)
                if os.path.isdir(src):
                    shutil.copytree(src, dst, dirs_exist_ok=True)
                else:
                    shutil.copy2(src, dst)
            if metadata:
                meta_path = os.path.join(target_dir, 'metadata.json')
                with open(meta_path, 'w') as f:
                    json.dump(metadata, f, indent=2)

    def close(self):
        with self._lock:
            if self._writer is not None:
                try:
                    self._writer.close()
                except Exception:
                    pass
                self._writer = None
            if self._json_file is not None:
                try:
                    self._json_file.close()
                except Exception:
                    pass
                self._json_file = None
