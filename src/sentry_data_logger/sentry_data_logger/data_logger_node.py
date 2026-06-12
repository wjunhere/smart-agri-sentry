import json
import os
import shutil
import time

import rclpy
from rclpy.node import Node

from sentry_interfaces.msg import (
    AdvisoryAction,
    Diagnosis,
    Environment,
    ForecastAlert,
    FusionResult,
    MissionStatus,
)
from .bag_writer import BagWriter


ALERT_CRITICAL = 3

_TOPIC_TYPES = {
    '/fusion/diagnosis': FusionResult,
    '/mission/status': MissionStatus,
    '/forecast/alert': ForecastAlert,
    '/advisory/action': AdvisoryAction,
    '/sensor/environment_mobile': Environment,
    '/vision/diagnosis': Diagnosis,
}


class DataLoggerNode(Node):
    def __init__(self):
        super().__init__('data_logger_node')
        self.declare_parameter('topics', [
            '/fusion/diagnosis',
            '/mission/status',
            '/forecast/alert',
            '/advisory/action',
            '/sensor/environment_mobile',
            '/vision/diagnosis',
        ])
        self.declare_parameter('bag_base_dir', 'bags')
        self.declare_parameter('split_duration_sec', 900)
        self.declare_parameter('split_max_size_mb', 1024)
        self.declare_parameter('retention_days', 7)
        self.declare_parameter('critical_retention_sec', 300)
        self.declare_parameter('record_metadata', True)

        topics = self.get_parameter('topics').value
        base_dir = self.get_parameter('bag_base_dir').value
        split_duration = self.get_parameter('split_duration_sec').value
        split_size = self.get_parameter('split_max_size_mb').value
        self.retention_days = self.get_parameter('retention_days').value
        self.critical_retention_sec = self.get_parameter(
            'critical_retention_sec').value
        self.record_metadata = self.get_parameter('record_metadata').value

        self.writer = BagWriter(
            base_dir=base_dir,
            split_duration_sec=split_duration,
            split_max_size_mb=split_size,
        )
        self.writer.open()

        self._latest = {}
        self._critical_keys = set()
        self._topic_subscriptions = []
        for topic in topics:
            msg_type = _TOPIC_TYPES.get(topic)
            if msg_type is None:
                self.get_logger().warn(f'Unknown topic type for {topic}, skipping')
                continue
            sub = self.create_subscription(
                msg_type,
                topic,
                lambda msg, t=topic: self._on_msg(t, msg),
                10,
            )
            self._topic_subscriptions.append(sub)

        self._cleanup_timer = self.create_timer(3600.0, self._cleanup_old_bags)
        self._cleanup_old_bags()

        self.get_logger().info(f'Data logger ready (base_dir={base_dir})')

    def _on_msg(self, topic, msg):
        now_ns = self.get_clock().now().nanoseconds
        self.writer.write(topic, msg, now_ns)
        self._latest[topic] = msg

        if topic == '/fusion/diagnosis':
            self._handle_fusion(msg)

    def _handle_fusion(self, msg: FusionResult):
        if msg.alert_level != ALERT_CRITICAL:
            return
        key = f'{msg.header.stamp.sec}_{msg.header.stamp.nanosec}'
        if key in self._critical_keys:
            return
        self._critical_keys.add(key)

        ts = time.strftime('%Y%m%d_%H%M%S')
        target_dir = os.path.join('records', 'critical', ts)
        metadata = {}
        if self.record_metadata:
            metadata = self._build_metadata(msg)
        self.writer.snapshot_critical(target_dir, metadata)
        self.get_logger().info(
            f'CRITICAL snapshot saved to {target_dir}')

    def _build_metadata(self, fusion_msg):
        now = self.get_clock().now().to_msg()
        return {
            'saved_at': {
                'sec': now.sec,
                'nanosec': now.nanosec,
            },
            'trigger': {
                'topic': '/fusion/diagnosis',
                'risk_score': float(fusion_msg.risk_score),
                'alert_level': int(fusion_msg.alert_level),
                'mode': str(fusion_msg.mode),
            },
            'context': self._latest_context(),
        }

    def _latest_context(self):
        ctx = {}
        env = self._latest.get('/sensor/environment_mobile')
        if env is not None:
            ctx['environment'] = {
                'air_temp': float(env.air_temp),
                'air_humidity': float(env.air_humidity),
                'air_co2': float(env.air_co2),
                'soil_temp': float(env.soil_temp),
                'soil_humidity': float(env.soil_humidity),
                'leaf_wetness': float(env.leaf_wetness),
                'data_source': str(env.data_source),
            }
        advisory = self._latest.get('/advisory/action')
        if advisory is not None:
            ctx['advisory'] = {
                'action_type': str(advisory.action_type),
                'priority': str(advisory.priority),
                'description': str(advisory.description),
            }
        forecast = self._latest.get('/forecast/alert')
        if forecast is not None:
            ctx['forecast'] = {
                'active': bool(forecast.active),
                'alert_type': str(forecast.alert_type),
                'probability': float(forecast.probability),
            }
        return ctx

    def _cleanup_old_bags(self):
        if not os.path.exists(self.writer.base_dir):
            return
        cutoff = time.time() - (self.retention_days * 86400)
        for name in os.listdir(self.writer.base_dir):
            path = os.path.join(self.writer.base_dir, name)
            if not os.path.isdir(path):
                continue
            try:
                mtime = os.path.getmtime(path)
            except OSError:
                continue
            if mtime < cutoff:
                try:
                    shutil.rmtree(path)
                    self.get_logger().info(f'Removed old bag dir: {path}')
                except Exception as e:
                    self.get_logger().warn(f'Failed to remove {path}: {e}')

    def destroy_node(self):
        self.writer.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = DataLoggerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
