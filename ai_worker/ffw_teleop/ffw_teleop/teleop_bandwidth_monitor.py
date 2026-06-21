import json
import os

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String


STREAM_ROWS = (
    ('zed', 'ZED'),
    ('wrist_left', 'L WRIST'),
    ('wrist_right', 'R WRIST'),
)


class TeleopBandwidthMonitor(Node):

    def __init__(self):
        super().__init__('teleop_bandwidth_monitor')

        self.declare_parameter('stream_stats_topic', '/teleop/stream_stats')
        self.declare_parameter('monitor_topic', '/teleop/bandwidth_monitor')
        self.declare_parameter('panel_topic', '/teleop/bandwidth_monitor/compressed')
        self.declare_parameter('available_mbps', 350.0)
        self.declare_parameter('window_s', 3.0)
        self.declare_parameter('stale_timeout_s', 1.5)
        self.declare_parameter('publish_hz', 2.0)
        self.declare_parameter('panel_width', 760)
        self.declare_parameter('panel_height', 300)
        self.declare_parameter('panel_jpeg_quality', 86)
        self.declare_parameter('network_interface', '')
        self.declare_parameter('network_tx_enabled', True)

        self.stream_stats_topic = str(
            self.get_parameter('stream_stats_topic').value).strip()
        self.monitor_topic = str(self.get_parameter('monitor_topic').value).strip()
        self.panel_topic = str(self.get_parameter('panel_topic').value).strip()
        self.available_mbps = max(float(self.get_parameter('available_mbps').value), 0.1)
        self.window_s = max(float(self.get_parameter('window_s').value), 0.5)
        self.stale_timeout_s = max(float(self.get_parameter('stale_timeout_s').value), 0.2)
        publish_hz = max(float(self.get_parameter('publish_hz').value), 0.2)
        self.panel_width = max(int(self.get_parameter('panel_width').value), 480)
        self.panel_height = max(int(self.get_parameter('panel_height').value), 220)
        self.panel_jpeg_quality = int(np.clip(
            int(self.get_parameter('panel_jpeg_quality').value), 1, 100))
        self.network_tx_enabled = self._as_bool(
            self.get_parameter('network_tx_enabled').value)
        self.network_interface = str(
            self.get_parameter('network_interface').value).strip()
        if self.network_tx_enabled and not self.network_interface:
            self.network_interface = self._detect_network_interface()
        self.last_tx_sample = None
        self.samples = {}

        self.create_subscription(
            String, self.stream_stats_topic, self._stream_stats_callback, 100)
        self.monitor_pub = self.create_publisher(String, self.monitor_topic, 10)
        self.panel_pub = self.create_publisher(CompressedImage, self.panel_topic, 1)
        self.timer = self.create_timer(1.0 / publish_hz, self._publish)

        self.get_logger().info(
            f'teleop bandwidth monitor: stats={self.stream_stats_topic}, '
            f'panel={self.panel_topic}, budget={self.available_mbps:.1f} Mbps, '
            f'net_if={self.network_interface or "disabled"}')

    def _as_bool(self, value):
        if isinstance(value, bool):
            return value
        if isinstance(value, str):
            return value.strip().lower() in ('1', 'true', 'yes', 'on')
        return bool(value)

    def _stream_stats_callback(self, msg):
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().warn('ignoring malformed stream stats JSON')
            return

        name = str(payload.get('name') or '').strip()
        if not name:
            return
        try:
            byte_count = int(payload.get('bytes', 0))
        except (TypeError, ValueError):
            return
        if byte_count <= 0:
            return

        now = self.get_clock().now().nanoseconds / 1e9
        sample = {
            't': now,
            'bytes': byte_count,
            'topic': str(payload.get('topic') or ''),
            'width': self._int_or_none(payload.get('width')),
            'height': self._int_or_none(payload.get('height')),
            'jpeg_quality': self._int_or_none(payload.get('jpeg_quality')),
        }
        samples = self.samples.setdefault(name, [])
        samples.append(sample)
        self.samples[name] = [
            item for item in samples
            if now - item['t'] <= max(self.window_s, self.stale_timeout_s)
        ]

    def _publish(self):
        now = self.get_clock().now().nanoseconds / 1e9
        streams = {}
        for name, _ in STREAM_ROWS:
            streams[name] = self._stream_snapshot(name, now)
        for name in sorted(set(self.samples.keys()) - set(streams.keys())):
            streams[name] = self._stream_snapshot(name, now)

        total_mbps = sum(
            stream['mbps'] for stream in streams.values()
            if stream.get('fresh')
        )
        usage_percent = 100.0 * total_mbps / self.available_mbps
        payload = {
            'stamp_sec': now,
            'available_mbps': self.available_mbps,
            'total_mbps': total_mbps,
            'usage_percent': usage_percent,
            'headroom_mbps': self.available_mbps - total_mbps,
            'net_tx_mbps': self._network_tx_mbps(now),
            'streams': streams,
        }

        msg = String()
        msg.data = json.dumps(payload, sort_keys=True)
        self.monitor_pub.publish(msg)
        self._publish_panel(payload)

    def _stream_snapshot(self, name, now):
        samples = [
            sample for sample in self.samples.get(name, [])
            if now - sample['t'] <= self.window_s
        ]
        self.samples[name] = samples
        if not samples or now - samples[-1]['t'] > self.stale_timeout_s:
            latest = samples[-1] if samples else {}
            return {
                'fresh': False,
                'fps': 0.0,
                'mbps': 0.0,
                'bytes_per_frame': int(latest.get('bytes', 0) or 0),
                'width': latest.get('width'),
                'height': latest.get('height'),
                'jpeg_quality': latest.get('jpeg_quality'),
                'topic': latest.get('topic', ''),
            }

        duration = max(min(self.window_s, now - samples[0]['t']), 0.2)
        total_bytes = sum(sample['bytes'] for sample in samples)
        latest = samples[-1]
        return {
            'fresh': True,
            'fps': len(samples) / duration,
            'mbps': total_bytes * 8.0 / duration / 1e6,
            'bytes_per_frame': total_bytes / max(len(samples), 1),
            'width': latest.get('width'),
            'height': latest.get('height'),
            'jpeg_quality': latest.get('jpeg_quality'),
            'topic': latest.get('topic', ''),
        }

    def _publish_panel(self, payload):
        if self.count_subscribers(self.panel_topic) <= 0:
            return
        image = self._render_panel(payload)
        ok, encoded = cv2.imencode(
            '.jpg', image, [int(cv2.IMWRITE_JPEG_QUALITY), self.panel_jpeg_quality])
        if not ok:
            self.get_logger().warn('failed to JPEG-encode bandwidth monitor panel')
            return
        msg = CompressedImage()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'teleop_bandwidth_monitor'
        msg.format = 'jpeg'
        msg.data = encoded.tobytes()
        self.panel_pub.publish(msg)

    def _render_panel(self, payload):
        width = self.panel_width
        height = self.panel_height
        image = np.full((height, width, 3), (30, 32, 36), dtype=np.uint8)
        total_mbps = float(payload.get('total_mbps') or 0.0)
        usage = float(payload.get('usage_percent') or 0.0)
        headroom = float(payload.get('headroom_mbps') or 0.0)
        header_color = self._usage_color(usage)
        cv2.rectangle(image, (0, 0), (width, 58), header_color, -1)
        self._put_text(image, 'BANDWIDTH MONITOR', (16, 38), 0.82, (255, 255, 255), 2)
        self._put_text(
            image,
            f'{total_mbps:5.1f} / {self.available_mbps:.0f} Mbps  {usage:4.0f}%',
            (390, 38),
            0.66,
            (255, 255, 255),
            2,
        )

        self._draw_usage_bar(image, 18, 76, width - 36, 18, usage)
        self._put_text(
            image,
            f'HEADROOM {headroom:5.1f} Mbps',
            (18, 116),
            0.52,
            (226, 232, 238),
            1,
        )
        net_tx = payload.get('net_tx_mbps')
        net_text = 'NET TX --' if net_tx is None else f'NET TX {float(net_tx):5.1f} Mbps'
        self._put_text(image, net_text, (420, 116), 0.52, (226, 232, 238), 1)

        streams = payload.get('streams') or {}
        y = 154
        for name, label in STREAM_ROWS:
            self._draw_stream_row(image, label, streams.get(name) or {}, y)
            y += 40

        self._put_text(
            image,
            f'Rolling window {self.window_s:.1f}s  Budget {self.available_mbps:.0f} Mbps',
            (18, height - 18),
            0.42,
            (172, 184, 194),
            1,
        )
        return image

    def _draw_usage_bar(self, image, x, y, width, height, usage_percent):
        cv2.rectangle(image, (x, y), (x + width, y + height), (64, 67, 74), -1)
        fill_width = int(width * min(max(usage_percent, 0.0), 100.0) / 100.0)
        if fill_width > 0:
            cv2.rectangle(
                image,
                (x, y),
                (x + fill_width, y + height),
                self._usage_color(usage_percent),
                -1,
            )
        cv2.rectangle(image, (x, y), (x + width, y + height), (160, 168, 176), 1)

    def _draw_stream_row(self, image, label, stream, y):
        fresh = bool(stream.get('fresh'))
        color = (226, 232, 238) if fresh else (126, 134, 144)
        if fresh:
            fps = float(stream.get('fps') or 0.0)
            mbps = float(stream.get('mbps') or 0.0)
            usage = 100.0 * mbps / self.available_mbps
            width = stream.get('width')
            height = stream.get('height')
            quality = stream.get('jpeg_quality')
            res = f'{width}x{height}' if width and height else '--'
            q_text = f'Q{int(quality)}' if quality is not None else 'Q--'
            text = f'{label:<8} {fps:4.1f} fps   {mbps:5.1f} Mbps   {res:<9} {q_text}'
            bar_color = self._usage_color(usage)
        else:
            text = f'{label:<8} -- fps     -- Mbps     STALE'
            usage = 0.0
            bar_color = (86, 90, 98)
        self._put_text(image, text, (18, y), 0.50, color, 1)
        x = self.panel_width - 170
        cv2.rectangle(image, (x, y - 15), (x + 138, y - 4), (58, 61, 68), -1)
        fill_width = int(138 * min(max(usage, 0.0), 100.0) / 100.0)
        if fill_width > 0:
            cv2.rectangle(image, (x, y - 15), (x + fill_width, y - 4), bar_color, -1)
        cv2.rectangle(image, (x, y - 15), (x + 138, y - 4), (130, 138, 148), 1)

    def _usage_color(self, usage_percent):
        if usage_percent >= 90.0:
            return (54, 68, 224)
        if usage_percent >= 70.0:
            return (42, 154, 232)
        return (65, 166, 92)

    def _put_text(self, image, text, origin, scale, color, thickness):
        cv2.putText(
            image, str(text), origin, cv2.FONT_HERSHEY_SIMPLEX, scale,
            (0, 0, 0), thickness + 2, cv2.LINE_AA)
        cv2.putText(
            image, str(text), origin, cv2.FONT_HERSHEY_SIMPLEX, scale,
            color, thickness, cv2.LINE_AA)

    def _network_tx_mbps(self, now):
        if not self.network_tx_enabled or not self.network_interface:
            return None
        tx_bytes = self._read_tx_bytes(self.network_interface)
        if tx_bytes is None:
            return None
        if self.last_tx_sample is None:
            self.last_tx_sample = (now, tx_bytes)
            return None
        prev_time, prev_bytes = self.last_tx_sample
        self.last_tx_sample = (now, tx_bytes)
        dt = now - prev_time
        if dt <= 0.0 or tx_bytes < prev_bytes:
            return None
        return (tx_bytes - prev_bytes) * 8.0 / dt / 1e6

    def _detect_network_interface(self):
        route_path = '/proc/net/route'
        try:
            with open(route_path, 'r', encoding='utf-8') as stream:
                for line in stream.readlines()[1:]:
                    parts = line.split()
                    if len(parts) >= 2 and parts[1] == '00000000':
                        return parts[0]
        except OSError:
            pass

        net_path = '/sys/class/net'
        try:
            for name in sorted(os.listdir(net_path)):
                if name == 'lo':
                    continue
                state_path = os.path.join(net_path, name, 'operstate')
                try:
                    with open(state_path, 'r', encoding='utf-8') as stream:
                        if stream.read().strip() == 'up':
                            return name
                except OSError:
                    continue
        except OSError:
            return ''
        return ''

    def _read_tx_bytes(self, interface):
        path = os.path.join('/sys/class/net', interface, 'statistics', 'tx_bytes')
        try:
            with open(path, 'r', encoding='utf-8') as stream:
                return int(stream.read().strip())
        except (OSError, ValueError):
            return None

    def _int_or_none(self, value):
        try:
            return int(value)
        except (TypeError, ValueError):
            return None


def main(args=None):
    rclpy.init(args=args)
    node = TeleopBandwidthMonitor()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
