import os
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from sensor_msgs.msg import CompressedImage


DEFAULT_STREAMS = [
    'STATUS|/teleop/operator_status/compressed',
    'MISSION|/teleop/mission_panel/compressed',
    'BANDWIDTH|/teleop/bandwidth_monitor/compressed',
    'ZED|/teleop/zed/depth_assist/compressed',
    'L WRIST|/teleop/wrist_left/depth_assist/compressed',
    'R WRIST|/teleop/wrist_right/depth_assist/compressed',
]


class OperatorImageViewer(Node):

    def __init__(self):
        super().__init__('operator_image_viewer')

        self.declare_parameter('window_title', 'Teleop Image Viewer')
        self.declare_parameter('streams', DEFAULT_STREAMS)
        self.declare_parameter('tile_width', 480)
        self.declare_parameter('tile_height', 270)
        self.declare_parameter('columns', 2)
        self.declare_parameter('show_hz', 20.0)
        self.declare_parameter('max_stale_sec', 2.0)
        self.declare_parameter('window_x', 1520)
        self.declare_parameter('window_y', 40)
        self.declare_parameter('headless_ok', True)

        self.window_title = str(self.get_parameter('window_title').value).strip()
        if not self.window_title:
            self.window_title = 'Teleop Image Viewer'
        self.tile_width = max(int(self.get_parameter('tile_width').value), 160)
        self.tile_height = max(int(self.get_parameter('tile_height').value), 120)
        self.columns = max(int(self.get_parameter('columns').value), 1)
        show_hz = max(float(self.get_parameter('show_hz').value), 1.0)
        self.max_stale_sec = max(float(self.get_parameter('max_stale_sec').value), 0.2)
        self.window_x = int(self.get_parameter('window_x').value)
        self.window_y = int(self.get_parameter('window_y').value)
        self.headless_ok = self._as_bool(self.get_parameter('headless_ok').value)

        self.streams = self._parse_streams(self.get_parameter('streams').value)
        self.frames = {
            stream['name']: {
                'image': None,
                'stamp': 0.0,
                'topic': stream['topic'],
            }
            for stream in self.streams
        }
        self.gui_available = False
        self._image_subscriptions = []

        qos = QoSProfile(depth=2)
        qos.reliability = ReliabilityPolicy.BEST_EFFORT
        qos.durability = DurabilityPolicy.VOLATILE
        for stream in self.streams:
            self._image_subscriptions.append(
                self.create_subscription(
                    CompressedImage,
                    stream['topic'],
                    self._make_callback(stream['name']),
                    qos,
                )
            )

        self._init_window()
        self.timer = self.create_timer(1.0 / show_hz, self._show)
        self.get_logger().info(
            f'operator image viewer active: window={self.window_title!r}, '
            f'streams={len(self.streams)}, gui={self.gui_available}')

    def destroy_node(self):
        if self.gui_available:
            try:
                cv2.destroyWindow(self.window_title)
            except cv2.error:
                pass
        super().destroy_node()

    def _as_bool(self, value):
        if isinstance(value, bool):
            return value
        if isinstance(value, str):
            return value.strip().lower() in ('1', 'true', 'yes', 'on')
        return bool(value)

    def _parse_streams(self, raw_streams):
        streams = []
        for raw in list(raw_streams or []):
            text = str(raw).strip()
            if not text:
                continue
            if '|' in text:
                name, topic = text.split('|', 1)
            else:
                topic = text
                name = topic.rsplit('/', 2)[0].rsplit('/', 1)[-1].upper()
            name = name.strip() or topic
            topic = topic.strip()
            if topic:
                streams.append({'name': name, 'topic': topic})
        return streams or [
            {'name': item.split('|', 1)[0], 'topic': item.split('|', 1)[1]}
            for item in DEFAULT_STREAMS
        ]

    def _init_window(self):
        if not os.environ.get('DISPLAY') and self.headless_ok:
            self.get_logger().warn('DISPLAY is not set; image viewer running headless')
            return
        try:
            rows = int(np.ceil(len(self.streams) / float(self.columns)))
            width = self.tile_width * self.columns
            height = self.tile_height * rows
            cv2.namedWindow(self.window_title, cv2.WINDOW_NORMAL)
            cv2.resizeWindow(self.window_title, width, height)
            if self.window_x >= 0 and self.window_y >= 0:
                cv2.moveWindow(self.window_title, self.window_x, self.window_y)
            self.gui_available = True
        except cv2.error as exc:
            self.get_logger().error(f'failed to create OpenCV viewer window: {exc}')
            self.gui_available = False

    def _make_callback(self, name):
        def callback(msg):
            data = np.frombuffer(msg.data, dtype=np.uint8)
            image = cv2.imdecode(data, cv2.IMREAD_COLOR)
            if image is None:
                self.get_logger().warn(f'failed to decode compressed image for {name}')
                return
            self.frames[name]['image'] = image
            self.frames[name]['stamp'] = time.time()
        return callback

    def _show(self):
        if not self.gui_available:
            return
        board = self._compose_board()
        try:
            cv2.imshow(self.window_title, board)
            key = cv2.waitKey(1) & 0xFF
        except cv2.error as exc:
            self.get_logger().error(f'OpenCV viewer failed; disabling GUI: {exc}')
            self.gui_available = False
            return
        if key in (27, ord('q'), ord('Q')):
            self.gui_available = False
            cv2.destroyWindow(self.window_title)

    def _compose_board(self):
        rows = int(np.ceil(len(self.streams) / float(self.columns)))
        board = np.full(
            (self.tile_height * rows, self.tile_width * self.columns, 3),
            (22, 24, 28),
            dtype=np.uint8,
        )
        now = time.time()
        for index, stream in enumerate(self.streams):
            row = index // self.columns
            col = index % self.columns
            x0 = col * self.tile_width
            y0 = row * self.tile_height
            tile = self._make_tile(stream, now)
            board[y0:y0 + self.tile_height, x0:x0 + self.tile_width] = tile
        return board

    def _make_tile(self, stream, now):
        name = stream['name']
        state = self.frames.get(name, {})
        image = state.get('image')
        stamp = float(state.get('stamp') or 0.0)
        stale = stamp <= 0.0 or (now - stamp) > self.max_stale_sec
        if image is None:
            tile = np.full((self.tile_height, self.tile_width, 3), (34, 36, 42), dtype=np.uint8)
            self._put_center(tile, 'waiting for image', (166, 172, 180), 0.58)
        else:
            tile = self._fit_image(image)
            if stale:
                overlay = np.full_like(tile, (20, 20, 20))
                tile = cv2.addWeighted(tile, 0.45, overlay, 0.55, 0)
                self._put_center(tile, 'stale', (80, 180, 255), 0.74)

        cv2.rectangle(tile, (0, 0), (self.tile_width - 1, self.tile_height - 1), (74, 78, 86), 1)
        cv2.rectangle(tile, (0, 0), (self.tile_width - 1, 30), (12, 14, 18), -1)
        cv2.putText(
            tile,
            name,
            (10, 21),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.56,
            (238, 240, 244),
            1,
            cv2.LINE_AA,
        )
        age = '--' if stamp <= 0.0 else f'{now - stamp:.1f}s'
        cv2.putText(
            tile,
            age,
            (self.tile_width - 62, 21),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.46,
            (190, 196, 204),
            1,
            cv2.LINE_AA,
        )
        return tile

    def _fit_image(self, image):
        src_h, src_w = image.shape[:2]
        if src_h <= 0 or src_w <= 0:
            return np.full((self.tile_height, self.tile_width, 3), (34, 36, 42), dtype=np.uint8)
        scale = min(self.tile_width / float(src_w), self.tile_height / float(src_h))
        new_w = max(int(src_w * scale), 1)
        new_h = max(int(src_h * scale), 1)
        resized = cv2.resize(image, (new_w, new_h), interpolation=cv2.INTER_AREA)
        tile = np.full((self.tile_height, self.tile_width, 3), (10, 10, 12), dtype=np.uint8)
        x0 = (self.tile_width - new_w) // 2
        y0 = (self.tile_height - new_h) // 2
        tile[y0:y0 + new_h, x0:x0 + new_w] = resized
        return tile

    def _put_center(self, image, text, color, scale):
        size, _ = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, scale, 1)
        x = max((image.shape[1] - size[0]) // 2, 4)
        y = max((image.shape[0] + size[1]) // 2, 20)
        cv2.putText(
            image,
            text,
            (x, y),
            cv2.FONT_HERSHEY_SIMPLEX,
            scale,
            color,
            1,
            cv2.LINE_AA,
        )


def main(args=None):
    rclpy.init(args=args)
    node = OperatorImageViewer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
