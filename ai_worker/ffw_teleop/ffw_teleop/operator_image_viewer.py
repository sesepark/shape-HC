import json
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

TOOLBAR_HEIGHT = 42
RESIZE_HANDLE_PX = 18
MIN_TILE_WIDTH = 180
MIN_TILE_HEIGHT = 120


class OperatorImageViewer(Node):

    def __init__(self):
        super().__init__('operator_image_viewer')

        self.declare_parameter('window_title', 'Teleop Image Viewer')
        self.declare_parameter('streams', DEFAULT_STREAMS)
        self.declare_parameter('tile_width', 640)
        self.declare_parameter('tile_height', 360)
        self.declare_parameter('columns', 2)
        self.declare_parameter('canvas_width', 1280)
        self.declare_parameter('canvas_height', 1080)
        self.declare_parameter(
            'layout_store_path',
            '~/.config/ffw_teleop/operator_image_viewer_layout.json')
        self.declare_parameter('show_toolbar', True)
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
        self.canvas_width = max(int(self.get_parameter('canvas_width').value), 640)
        self.canvas_height = max(int(self.get_parameter('canvas_height').value), 360)
        self.layout_store_path = os.path.expanduser(
            str(self.get_parameter('layout_store_path').value).strip())
        self.show_toolbar = self._as_bool(self.get_parameter('show_toolbar').value)
        self.toolbar_height = TOOLBAR_HEIGHT if self.show_toolbar else 0
        show_hz = max(float(self.get_parameter('show_hz').value), 1.0)
        self.max_stale_sec = max(float(self.get_parameter('max_stale_sec').value), 0.2)
        self.window_x = int(self.get_parameter('window_x').value)
        self.window_y = int(self.get_parameter('window_y').value)
        self.headless_ok = self._as_bool(self.get_parameter('headless_ok').value)

        self.streams = self._parse_streams(self.get_parameter('streams').value)
        self.stream_by_name = {stream['name']: stream for stream in self.streams}
        self.frames = {
            stream['name']: {
                'image': None,
                'stamp': 0.0,
                'topic': stream['topic'],
            }
            for stream in self.streams
        }
        self.layout_by_name = self._default_layout()
        self.z_order = [stream['name'] for stream in self.streams]
        self.selected_name = ''
        self.drag_mode = ''
        self.drag_start = None
        self.gui_available = False
        self._image_subscriptions = []

        self._load_layout()

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
            f'streams={len(self.streams)}, canvas={self.canvas_width}x{self.canvas_height}, '
            f'layout={self.layout_store_path or "disabled"}, gui={self.gui_available}')

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

    def _default_layout(self):
        layout = {}
        for index, stream in enumerate(self.streams):
            rect = {
                'x': (index % self.columns) * self.tile_width,
                'y': (index // self.columns) * self.tile_height,
                'width': self.tile_width,
                'height': self.tile_height,
            }
            layout[stream['name']] = self._sanitize_rect(rect)
        return layout

    def _load_layout(self):
        if not self.layout_store_path or not os.path.exists(self.layout_store_path):
            return
        try:
            with open(self.layout_store_path, 'r', encoding='utf-8') as stream:
                payload = json.load(stream)
        except (OSError, json.JSONDecodeError) as exc:
            self.get_logger().warn(f'failed to load image viewer layout: {exc}')
            return

        stored_layout = payload.get('layout') if isinstance(payload, dict) else None
        if not isinstance(stored_layout, dict):
            return
        for name, rect in stored_layout.items():
            if name in self.layout_by_name and isinstance(rect, dict):
                self.layout_by_name[name] = self._sanitize_rect(rect)

        stored_order = payload.get('z_order', [])
        if isinstance(stored_order, list):
            ordered = [str(name) for name in stored_order if str(name) in self.layout_by_name]
            ordered.extend(name for name in self.layout_by_name.keys() if name not in ordered)
            self.z_order = ordered
        self.get_logger().info(f'loaded image viewer layout: {self.layout_store_path}')

    def _save_layout(self):
        if not self.layout_store_path:
            self.get_logger().warn('image viewer layout save skipped: layout_store_path is empty')
            return
        payload = {
            'version': 1,
            'canvas_width': self.canvas_width,
            'canvas_height': self.canvas_height,
            'layout': self.layout_by_name,
            'z_order': self.z_order,
        }
        try:
            parent = os.path.dirname(self.layout_store_path)
            if parent:
                os.makedirs(parent, exist_ok=True)
            tmp_path = f'{self.layout_store_path}.tmp'
            with open(tmp_path, 'w', encoding='utf-8') as stream:
                json.dump(payload, stream, indent=2, sort_keys=True)
                stream.write('\n')
            os.replace(tmp_path, self.layout_store_path)
            self.get_logger().info(f'saved image viewer layout: {self.layout_store_path}')
        except OSError as exc:
            self.get_logger().warn(f'failed to save image viewer layout: {exc}')

    def _reset_layout(self, delete_saved):
        self.layout_by_name = self._default_layout()
        self.z_order = [stream['name'] for stream in self.streams]
        self.selected_name = ''
        self.drag_mode = ''
        self.drag_start = None
        if delete_saved and self.layout_store_path and os.path.exists(self.layout_store_path):
            try:
                os.remove(self.layout_store_path)
                self.get_logger().info(f'removed saved image viewer layout: {self.layout_store_path}')
            except OSError as exc:
                self.get_logger().warn(f'failed to remove saved image viewer layout: {exc}')

    def _sanitize_rect(self, rect):
        width = int(rect.get('width', self.tile_width))
        height = int(rect.get('height', self.tile_height))
        width = max(min(width, self.canvas_width), min(MIN_TILE_WIDTH, self.canvas_width))
        height = max(min(height, self.canvas_height), min(MIN_TILE_HEIGHT, self.canvas_height))
        max_x = max(self.canvas_width - width, 0)
        max_y = max(self.canvas_height - height, 0)
        x = int(np.clip(int(rect.get('x', 0)), 0, max_x))
        y = int(np.clip(int(rect.get('y', 0)), 0, max_y))
        return {'x': x, 'y': y, 'width': width, 'height': height}

    def _init_window(self):
        if not os.environ.get('DISPLAY') and self.headless_ok:
            self.get_logger().warn('DISPLAY is not set; image viewer running headless')
            return
        try:
            cv2.namedWindow(self.window_title, cv2.WINDOW_NORMAL)
            cv2.resizeWindow(
                self.window_title,
                self.canvas_width,
                self.canvas_height + self.toolbar_height,
            )
            cv2.setMouseCallback(self.window_title, self._mouse_callback)
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
        elif key in (ord('s'), ord('S')):
            self._save_layout()
        elif key in (ord('r'), ord('R')):
            self._reset_layout(delete_saved=True)
        elif key in (ord('g'), ord('G')):
            self._reset_layout(delete_saved=False)

    def _compose_board(self):
        board = np.full(
            (self.canvas_height + self.toolbar_height, self.canvas_width, 3),
            (22, 24, 28),
            dtype=np.uint8,
        )
        if self.show_toolbar:
            self._draw_toolbar(board)

        canvas = board[self.toolbar_height:self.toolbar_height + self.canvas_height, :]
        now = time.time()
        for name in self.z_order:
            stream = self.stream_by_name.get(name)
            rect = self.layout_by_name.get(name)
            if not stream or not rect:
                continue
            tile = self._make_tile(stream, now, rect['width'], rect['height'], name == self.selected_name)
            x0 = rect['x']
            y0 = rect['y']
            canvas[y0:y0 + rect['height'], x0:x0 + rect['width']] = tile
        return board

    def _draw_toolbar(self, board):
        cv2.rectangle(board, (0, 0), (self.canvas_width, self.toolbar_height), (14, 16, 20), -1)
        for action, label, rect in self._toolbar_buttons():
            x, y, width, height = rect
            color = (58, 64, 74) if action != 'reset' else (64, 54, 54)
            cv2.rectangle(board, (x, y), (x + width, y + height), color, -1)
            cv2.rectangle(board, (x, y), (x + width, y + height), (150, 158, 170), 1)
            self._put_text(board, label, (x + 12, y + 22), 0.48, (238, 240, 244), 1)
        hint = 'Drag tiles; drag bottom-right corner to resize; S/R/G shortcuts'
        self._put_text(board, hint, (296, 27), 0.42, (178, 186, 196), 1)

    def _toolbar_buttons(self):
        return [
            ('save', 'SAVE', (10, 7, 78, 28)),
            ('reset', 'RESET', (96, 7, 84, 28)),
            ('grid', 'GRID', (188, 7, 78, 28)),
        ]

    def _make_tile(self, stream, now, width, height, selected):
        name = stream['name']
        state = self.frames.get(name, {})
        image = state.get('image')
        stamp = float(state.get('stamp') or 0.0)
        stale = stamp <= 0.0 or (now - stamp) > self.max_stale_sec
        if image is None:
            tile = np.full((height, width, 3), (34, 36, 42), dtype=np.uint8)
            self._put_center(tile, 'waiting for image', (166, 172, 180), 0.58)
        else:
            tile = self._fit_image(image, width, height)
            if stale:
                overlay = np.full_like(tile, (20, 20, 20))
                tile = cv2.addWeighted(tile, 0.45, overlay, 0.55, 0)
                self._put_center(tile, 'stale', (80, 180, 255), 0.74)

        border = (30, 210, 255) if selected else (74, 78, 86)
        cv2.rectangle(tile, (0, 0), (width - 1, height - 1), border, 2 if selected else 1)
        cv2.rectangle(tile, (0, 0), (width - 1, 30), (12, 14, 18), -1)
        self._put_text(tile, name, (10, 21), 0.56, (238, 240, 244), 1)
        age = '--' if stamp <= 0.0 else f'{now - stamp:.1f}s'
        self._put_text(tile, age, (max(width - 62, 48), 21), 0.46, (190, 196, 204), 1)
        self._draw_resize_handle(tile)
        return tile

    def _fit_image(self, image, target_width, target_height):
        src_h, src_w = image.shape[:2]
        if src_h <= 0 or src_w <= 0:
            return np.full((target_height, target_width, 3), (34, 36, 42), dtype=np.uint8)
        scale = min(target_width / float(src_w), target_height / float(src_h))
        new_w = max(int(src_w * scale), 1)
        new_h = max(int(src_h * scale), 1)
        interpolation = cv2.INTER_AREA if scale < 1.0 else cv2.INTER_LINEAR
        resized = cv2.resize(image, (new_w, new_h), interpolation=interpolation)
        tile = np.full((target_height, target_width, 3), (10, 10, 12), dtype=np.uint8)
        x0 = (target_width - new_w) // 2
        y0 = (target_height - new_h) // 2
        tile[y0:y0 + new_h, x0:x0 + new_w] = resized
        return tile

    def _draw_resize_handle(self, tile):
        height, width = tile.shape[:2]
        points = np.array([
            [width - RESIZE_HANDLE_PX, height - 2],
            [width - 2, height - RESIZE_HANDLE_PX],
            [width - 2, height - 2],
        ], dtype=np.int32)
        cv2.fillConvexPoly(tile, points, (90, 96, 108))
        cv2.polylines(tile, [points], True, (190, 198, 208), 1, cv2.LINE_AA)

    def _mouse_callback(self, event, x, y, flags, param):
        del flags, param
        if event == cv2.EVENT_LBUTTONDOWN:
            self._mouse_down(x, y)
        elif event == cv2.EVENT_MOUSEMOVE and self.drag_mode:
            self._mouse_drag(x, y)
        elif event == cv2.EVENT_LBUTTONUP:
            self.drag_mode = ''
            self.drag_start = None

    def _mouse_down(self, x, y):
        if self.show_toolbar and y < self.toolbar_height:
            for action, _, rect in self._toolbar_buttons():
                if self._point_in_rect(x, y, rect):
                    self._run_toolbar_action(action)
                    return
            return

        point = self._canvas_point(x, y)
        if point is None:
            return
        canvas_x, canvas_y = point
        name = self._find_top_tile(canvas_x, canvas_y)
        if not name:
            self.selected_name = ''
            return

        self.selected_name = name
        self._bring_to_front(name)
        rect = self.layout_by_name[name]
        self.drag_mode = 'resize' if self._in_resize_handle(canvas_x, canvas_y, rect) else 'move'
        self.drag_start = {
            'mouse_x': canvas_x,
            'mouse_y': canvas_y,
            'rect': dict(rect),
        }

    def _mouse_drag(self, x, y):
        if not self.selected_name or not self.drag_start:
            return
        point = self._canvas_point(x, y)
        if point is None:
            return
        canvas_x, canvas_y = point
        dx = canvas_x - self.drag_start['mouse_x']
        dy = canvas_y - self.drag_start['mouse_y']
        start_rect = self.drag_start['rect']
        rect = dict(start_rect)
        if self.drag_mode == 'resize':
            rect['width'] = start_rect['width'] + dx
            rect['height'] = start_rect['height'] + dy
        else:
            rect['x'] = start_rect['x'] + dx
            rect['y'] = start_rect['y'] + dy
        self.layout_by_name[self.selected_name] = self._sanitize_rect(rect)

    def _canvas_point(self, x, y):
        canvas_y = y - self.toolbar_height
        if x < 0 or x >= self.canvas_width or canvas_y < 0 or canvas_y >= self.canvas_height:
            return None
        return x, canvas_y

    def _find_top_tile(self, x, y):
        for name in reversed(self.z_order):
            rect = self.layout_by_name.get(name)
            if not rect:
                continue
            if rect['x'] <= x < rect['x'] + rect['width'] and rect['y'] <= y < rect['y'] + rect['height']:
                return name
        return ''

    def _in_resize_handle(self, x, y, rect):
        return (
            x >= rect['x'] + rect['width'] - RESIZE_HANDLE_PX and
            y >= rect['y'] + rect['height'] - RESIZE_HANDLE_PX
        )

    def _bring_to_front(self, name):
        self.z_order = [item for item in self.z_order if item != name]
        self.z_order.append(name)

    def _run_toolbar_action(self, action):
        if action == 'save':
            self._save_layout()
        elif action == 'reset':
            self._reset_layout(delete_saved=True)
        elif action == 'grid':
            self._reset_layout(delete_saved=False)

    def _point_in_rect(self, x, y, rect):
        rx, ry, width, height = rect
        return rx <= x <= rx + width and ry <= y <= ry + height

    def _put_center(self, image, text, color, scale):
        size, _ = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, scale, 1)
        x = max((image.shape[1] - size[0]) // 2, 4)
        y = max((image.shape[0] + size[1]) // 2, 20)
        self._put_text(image, text, (x, y), scale, color, 1)

    def _put_text(self, image, text, origin, scale, color, thickness):
        cv2.putText(
            image,
            str(text),
            origin,
            cv2.FONT_HERSHEY_SIMPLEX,
            scale,
            color,
            thickness,
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
