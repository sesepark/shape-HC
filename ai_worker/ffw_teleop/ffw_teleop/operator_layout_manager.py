import os
import shutil
import subprocess
import time

import rclpy
from rclpy.node import Node


DEFAULT_WINDOWS = [
    {
        'name': 'rviz',
        'enabled': True,
        'title_patterns': ['teleop_operator_rviz', 'RViz', 'RViz2'],
        'x': 60,
        'y': 40,
        'width': 1440,
        'height': 900,
    },
    {
        'name': 'mission_control',
        'enabled': True,
        'title_patterns': ['Mission Control'],
        'x': 1520,
        'y': 40,
        'width': 380,
        'height': 360,
    },
]


class OperatorLayoutManager(Node):

    def __init__(self):
        super().__init__('operator_layout_manager')
        self.declare_parameter('layout_config', '')
        self.declare_parameter('initial_delay_s', 3.0)
        self.declare_parameter('retry_count', 24)
        self.declare_parameter('retry_interval_s', 0.5)
        self.declare_parameter('diagnostic_only', False)
        self.declare_parameter('dry_run', False)
        self.declare_parameter('wmctrl_path', 'wmctrl')

        self.layout_config = str(self.get_parameter('layout_config').value).strip()
        self.initial_delay_s = max(float(self.get_parameter('initial_delay_s').value), 0.0)
        self.retry_count = max(int(self.get_parameter('retry_count').value), 1)
        self.retry_interval_s = max(float(self.get_parameter('retry_interval_s').value), 0.1)
        self.diagnostic_only = self._as_bool(self.get_parameter('diagnostic_only').value)
        self.dry_run = self._as_bool(self.get_parameter('dry_run').value)
        self.wmctrl_path = str(self.get_parameter('wmctrl_path').value).strip() or 'wmctrl'
        self.windows = self._load_windows(self.layout_config)

    def run(self):
        wmctrl = self._resolve_wmctrl()
        if wmctrl is None:
            self.get_logger().warn(
                'wmctrl is not available; install it to enable fixed operator layout')
            return
        if self.diagnostic_only:
            self._print_diagnostics(wmctrl)
            return

        if self.initial_delay_s > 0.0:
            time.sleep(self.initial_delay_s)

        pending = [window for window in self.windows if self._enabled(window)]
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
            pending = next_pending
            if pending:
                time.sleep(self.retry_interval_s)

        for window in pending:
            self.get_logger().warn(
                f'window not found for layout: {window.get("name", "unnamed")}')

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

    def _load_windows(self, path):
        if not path:
            return DEFAULT_WINDOWS
        try:
            import yaml
            with open(os.path.expanduser(path), 'r', encoding='utf-8') as stream:
                loaded = yaml.safe_load(stream) or {}
        except OSError as exc:
            self.get_logger().warn(f'failed to read layout config {path}: {exc}')
            return DEFAULT_WINDOWS
        except Exception as exc:
            self.get_logger().warn(f'failed to parse layout config {path}: {exc}')
            return DEFAULT_WINDOWS
        windows = loaded.get('windows')
        if not isinstance(windows, list):
            self.get_logger().warn(f'layout config {path} has no windows list')
            return DEFAULT_WINDOWS
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
            windows.append({
                'id': parts[0],
                'desktop': parts[1],
                'x': parts[2],
                'y': parts[3],
                'width': parts[4],
                'height': parts[5],
                'host': parts[6],
                'title': parts[7],
            })
        return windows

    def _find_window(self, target, windows):
        patterns = [str(item) for item in target.get('title_patterns') or []]
        if not patterns:
            name = str(target.get('name') or '').strip()
            patterns = [name] if name else []
        for pattern in patterns:
            for window in windows:
                if pattern and pattern in window.get('title', ''):
                    return window
        return None

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
            f'layout {target.get("name", "window")} -> '
            f'{x},{y} {width}x{height} ({match["title"]})')
        if self.dry_run:
            return
        subprocess.run(command_remove_max, check=False)
        subprocess.run(command_geometry, check=False)

    def _print_diagnostics(self, wmctrl):
        windows = self._list_windows(wmctrl)
        if not windows:
            self.get_logger().info('no X11 windows reported by wmctrl')
            return
        for window in windows:
            self.get_logger().info(
                f'{window["id"]} x={window["x"]} y={window["y"]} '
                f'w={window["width"]} h={window["height"]} title={window["title"]}')


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
