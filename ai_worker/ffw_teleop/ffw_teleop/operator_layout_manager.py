import json
import os
import shutil
import subprocess
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


DEFAULT_TARGETS = [
    {
        'name': 'rviz',
        'enabled': True,
        'title_patterns': ['teleop_operator_rviz', 'RViz', 'RViz2'],
    },
    {
        'name': 'mission_control',
        'enabled': True,
        'title_patterns': ['Mission Control'],
    },
]


class OperatorLayoutManager(Node):

    def __init__(self):
        super().__init__('operator_layout_manager')
        self.declare_parameter('layout_config', '')
        self.declare_parameter('layout_store_path', '~/.config/ffw_teleop/operator_screen_layout.json')
        self.declare_parameter('action', 'restore')
        self.declare_parameter('command_topic', '/teleop/operator_layout/command')
        self.declare_parameter('status_topic', '/teleop/operator_layout/status')
        self.declare_parameter('initial_delay_s', 3.0)
        self.declare_parameter('retry_count', 24)
        self.declare_parameter('retry_interval_s', 0.5)
        self.declare_parameter('diagnostic_only', False)
        self.declare_parameter('dry_run', False)
        self.declare_parameter('wmctrl_path', 'wmctrl')

        self.layout_config = str(self.get_parameter('layout_config').value).strip()
        self.layout_store_path = os.path.expanduser(
            str(self.get_parameter('layout_store_path').value).strip())
        self.action = str(self.get_parameter('action').value).strip().lower() or 'restore'
        if self._as_bool(self.get_parameter('diagnostic_only').value):
            self.action = 'diagnostic'
        self.command_topic = str(self.get_parameter('command_topic').value).strip()
        self.status_topic = str(self.get_parameter('status_topic').value).strip()
        self.initial_delay_s = max(float(self.get_parameter('initial_delay_s').value), 0.0)
        self.retry_count = max(int(self.get_parameter('retry_count').value), 1)
        self.retry_interval_s = max(float(self.get_parameter('retry_interval_s').value), 0.1)
        self.dry_run = self._as_bool(self.get_parameter('dry_run').value)
        self.wmctrl_path = str(self.get_parameter('wmctrl_path').value).strip() or 'wmctrl'
        self.targets = self._load_targets(self.layout_config)
        self.status_pub = None
        if self.status_topic:
            self.status_pub = self.create_publisher(String, self.status_topic, 10)
        self.command_sub = None

    def run(self):
        if self.action == 'server':
            self._run_server()
            return
        if self.action == 'reset':
            self._reset_saved_layout()
            return
        wmctrl = self._resolve_wmctrl()
        if wmctrl is None:
            message = 'wmctrl is not available; install it to save/restore operator layout'
            self.get_logger().warn(message)
            self._publish_status('error', message, self.action)
            return

        if self.action == 'diagnostic':
            self._print_diagnostics(wmctrl)
            return
        if self.action == 'save':
            self._save_current_layout(wmctrl)
            return
        if self.action != 'restore':
            self.get_logger().warn(f'unknown layout action={self.action}; using restore')

        self._restore_saved_layout(wmctrl)

    def _run_server(self):
        if self.command_topic:
            self.command_sub = self.create_subscription(
                String, self.command_topic, self._command_callback, 10)
        self.get_logger().info(
            f'operator layout manager server active: command={self.command_topic}, '
            f'status={self.status_topic}')
        wmctrl = self._resolve_wmctrl()
        if wmctrl is None:
            message = 'wmctrl is not available; layout save/restore commands disabled'
            self.get_logger().warn(message)
            self._publish_status('error', message, 'server')
        else:
            self._restore_saved_layout(wmctrl)
            self._publish_status('ready', 'operator layout manager ready', 'server')
        rclpy.spin(self)

    def _command_callback(self, msg):
        action = str(msg.data or '').strip().lower()
        aliases = {
            'save_layout': 'save',
            'restore_layout': 'restore',
            'reset_layout': 'reset',
            'clear': 'reset',
        }
        action = aliases.get(action, action)
        if action == 'reset':
            self._reset_saved_layout()
            return
        if action in ('save', 'restore', 'diagnostic'):
            self._run_layout_action(action)
            return
        message = f'unknown layout command: {msg.data!r}'
        self.get_logger().warn(message)
        self._publish_status('error', message, action)

    def _run_layout_action(self, action):
        wmctrl = self._resolve_wmctrl()
        if wmctrl is None:
            message = 'wmctrl is not available; install it to save/restore operator layout'
            self.get_logger().warn(message)
            self._publish_status('error', message, action)
            return
        if action == 'save':
            self._save_current_layout(wmctrl)
        elif action == 'restore':
            self._restore_saved_layout(wmctrl)
        elif action == 'diagnostic':
            self._print_diagnostics(wmctrl)

    def _as_bool(self, value):
        if isinstance(value, bool):
            return value
        if isinstance(value, str):
            return value.strip().lower() in ('1', 'true', 'yes', 'on')
        return bool(value)

    def _resolve_wmctrl(self):
        if os.path.isabs(self.wmctrl_path) and os.path.exists(self.wmctrl_path):
            return self.wmctrl_path
        return shutil.which(self.wmctrl_path)

    def _publish_status(self, status, message, action='', details=None):
        if self.status_pub is None:
            return
        payload = {
            'stamp_sec': self.get_clock().now().nanoseconds / 1e9,
            'status': str(status),
            'action': str(action or ''),
            'message': str(message),
        }
        if details:
            payload['details'] = details
        msg = String()
        msg.data = json.dumps(payload, sort_keys=True)
        self.status_pub.publish(msg)

    def _load_targets(self, path):
        if not path:
            return DEFAULT_TARGETS
        try:
            import yaml
            with open(os.path.expanduser(path), 'r', encoding='utf-8') as stream:
                loaded = yaml.safe_load(stream) or {}
        except OSError as exc:
            self.get_logger().warn(f'failed to read layout target config {path}: {exc}')
            return DEFAULT_TARGETS
        except Exception as exc:
            self.get_logger().warn(f'failed to parse layout target config {path}: {exc}')
            return DEFAULT_TARGETS
        windows = loaded.get('windows')
        if not isinstance(windows, list):
            self.get_logger().warn(f'layout target config {path} has no windows list')
            return DEFAULT_TARGETS
        return windows

    def _enabled(self, window):
        return self._as_bool(window.get('enabled', True))

    def _list_windows(self, wmctrl):
        proc = subprocess.run(
            [wmctrl, '-lG'],
            check=False,
            text=True,
            capture_output=True,
        )
        if proc.returncode != 0:
            self.get_logger().warn(proc.stderr.strip() or 'wmctrl -lG failed')
            return []
        windows = []
        for line in proc.stdout.splitlines():
            parts = line.split(None, 7)
            if len(parts) < 8:
                continue
            try:
                geometry = {
                    'x': int(parts[2]),
                    'y': int(parts[3]),
                    'width': int(parts[4]),
                    'height': int(parts[5]),
                }
            except ValueError:
                continue
            windows.append({
                'id': parts[0],
                'desktop': parts[1],
                'host': parts[6],
                'title': parts[7],
                **geometry,
            })
        return windows

    def _find_window(self, target, windows):
        for pattern in self._target_patterns(target):
            for window in windows:
                if pattern and pattern in window.get('title', ''):
                    return window
        return None

    def _target_patterns(self, target):
        patterns = [str(item) for item in target.get('title_patterns') or []]
        if not patterns:
            name = str(target.get('name') or '').strip()
            patterns = [name] if name else []
        title = str(target.get('title') or '').strip()
        if title and title not in patterns:
            patterns.insert(0, title)
        return patterns

    def _save_current_layout(self, wmctrl):
        windows = self._list_windows(wmctrl)
        saved_windows = []
        missing_windows = []
        for target in self.targets:
            if not self._enabled(target):
                continue
            match = self._find_window(target, windows)
            if match is None:
                missing_windows.append(str(target.get('name') or 'unnamed'))
                self.get_logger().warn(
                    f'cannot save layout for missing window: {target.get("name", "unnamed")}')
                continue
            patterns = self._target_patterns(target)
            if match['title'] not in patterns:
                patterns.insert(0, match['title'])
            saved_windows.append({
                'name': str(target.get('name') or match['title']),
                'enabled': True,
                'title_patterns': patterns,
                'title': match['title'],
                'x': match['x'],
                'y': match['y'],
                'width': match['width'],
                'height': match['height'],
            })
            self.get_logger().info(
                f'saved {target.get("name", match["title"])}: '
                f'{match["x"]},{match["y"]} {match["width"]}x{match["height"]}')

        payload = {
            'version': 1,
            'saved_at_sec': time.time(),
            'windows': saved_windows,
        }
        if self.dry_run:
            self.get_logger().info(json.dumps(payload, sort_keys=True))
            self._publish_status(
                'ok',
                f'dry-run saved {len(saved_windows)} window layouts',
                'save',
                {'missing_windows': missing_windows},
            )
            return
        layout_dir = os.path.dirname(self.layout_store_path)
        if layout_dir:
            os.makedirs(layout_dir, exist_ok=True)
        with open(self.layout_store_path, 'w', encoding='utf-8') as stream:
            json.dump(payload, stream, indent=2, sort_keys=True)
            stream.write('\n')
        message = f'saved {len(saved_windows)} window layouts'
        self.get_logger().info(f'{message}: {self.layout_store_path}')
        self._publish_status(
            'ok',
            message,
            'save',
            {
                'path': self.layout_store_path,
                'missing_windows': missing_windows,
            },
        )

    def _restore_saved_layout(self, wmctrl):
        if not os.path.exists(self.layout_store_path):
            message = f'no saved operator layout at {self.layout_store_path}; skipping restore'
            self.get_logger().info(message)
            self._publish_status(
                'idle',
                'no saved operator layout; skipped restore',
                'restore',
                {'path': self.layout_store_path},
            )
            return
        try:
            with open(self.layout_store_path, 'r', encoding='utf-8') as stream:
                payload = json.load(stream)
        except (OSError, json.JSONDecodeError) as exc:
            message = f'failed to read saved operator layout: {exc}'
            self.get_logger().warn(message)
            self._publish_status('error', message, 'restore')
            return
        saved_windows = payload.get('windows') or []
        pending = [window for window in saved_windows if self._enabled(window)]
        if not pending:
            message = 'saved operator layout has no enabled windows'
            self.get_logger().info(message)
            self._publish_status('idle', message, 'restore')
            return

        if self.initial_delay_s > 0.0:
            time.sleep(self.initial_delay_s)

        restored_count = 0
        for _ in range(self.retry_count):
            if not pending:
                break
            current = self._list_windows(wmctrl)
            next_pending = []
            for window in pending:
                match = self._find_window(window, current)
                if match is None:
                    next_pending.append(window)
                    continue
                self._apply_geometry(wmctrl, match, window)
                restored_count += 1
            pending = next_pending
            if pending:
                time.sleep(self.retry_interval_s)

        for window in pending:
            self.get_logger().warn(
                f'window not found for saved layout: {window.get("name", "unnamed")}')
        if pending:
            self._publish_status(
                'warn',
                f'restored {restored_count} windows; {len(pending)} not found',
                'restore',
                {
                    'missing_windows': [
                        str(window.get('name') or 'unnamed') for window in pending
                    ],
                },
            )
        else:
            self._publish_status(
                'ok',
                f'restored {restored_count} windows',
                'restore',
                {'path': self.layout_store_path},
            )

    def _apply_geometry(self, wmctrl, match, target):
        x = int(target.get('x', 0))
        y = int(target.get('y', 0))
        width = int(target.get('width', 800))
        height = int(target.get('height', 600))
        command_remove_max = [
            wmctrl, '-ir', match['id'], '-b', 'remove,maximized_vert,maximized_horz']
        command_geometry = [
            wmctrl, '-ir', match['id'], '-e', f'0,{x},{y},{width},{height}']
        self.get_logger().info(
            f'restore {target.get("name", "window")} -> '
            f'{x},{y} {width}x{height} ({match["title"]})')
        if self.dry_run:
            return
        subprocess.run(command_remove_max, check=False)
        subprocess.run(command_geometry, check=False)

    def _print_diagnostics(self, wmctrl):
        windows = self._list_windows(wmctrl)
        if not windows:
            self.get_logger().info('no X11 windows reported by wmctrl')
            self._publish_status('idle', 'no X11 windows reported by wmctrl', 'diagnostic')
            return
        for window in windows:
            self.get_logger().info(
                f'{window["id"]} x={window["x"]} y={window["y"]} '
                f'w={window["width"]} h={window["height"]} title={window["title"]}')
        self._publish_status(
            'ok',
            f'reported {len(windows)} X11 windows',
            'diagnostic',
        )

    def _reset_saved_layout(self):
        if not self.layout_store_path:
            message = 'layout store path is empty'
            self.get_logger().warn(message)
            self._publish_status('error', message, 'reset')
            return
        if not os.path.exists(self.layout_store_path):
            message = 'no saved operator layout to reset'
            self.get_logger().info(message)
            self._publish_status(
                'idle',
                message,
                'reset',
                {'path': self.layout_store_path},
            )
            return
        try:
            os.remove(self.layout_store_path)
        except OSError as exc:
            message = f'failed to reset saved operator layout: {exc}'
            self.get_logger().warn(message)
            self._publish_status('error', message, 'reset')
            return
        message = 'deleted saved operator layout'
        self.get_logger().info(f'{message}: {self.layout_store_path}')
        self._publish_status(
            'ok',
            message,
            'reset',
            {'path': self.layout_store_path},
        )


def main(args=None):
    rclpy.init(args=args)
    node = OperatorLayoutManager()
    try:
        node.run()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
