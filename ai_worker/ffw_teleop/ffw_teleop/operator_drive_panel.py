import json
import os
import time

from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from std_msgs.msg import Bool
from std_msgs.msg import String


PANEL_WIDTH = 420
PANEL_HEIGHT = 520
KEY_RELEASE_DEBOUNCE_MS = 60
MOVE_ACTIONS = ('forward', 'backward', 'left', 'right', 'rot_left', 'rot_right')
ACTION_LABELS = {
    'forward': 'FORWARD',
    'backward': 'BACK',
    'left': 'LEFT',
    'right': 'RIGHT',
    'rot_left': 'ROT L',
    'rot_right': 'ROT R',
}
KEY_ACTIONS = {
    'w': 'forward',
    's': 'backward',
    'a': 'left',
    'd': 'right',
    'q': 'rot_left',
    'e': 'rot_right',
}


class OperatorDrivePanel(Node):

    def __init__(self):
        super().__init__('operator_drive_panel')

        self.declare_parameter('window_title', 'Teleop Drive Control')
        self.declare_parameter('window_width', PANEL_WIDTH)
        self.declare_parameter('window_height', PANEL_HEIGHT)
        self.declare_parameter('window_x', 80)
        self.declare_parameter('window_y', 560)
        self.declare_parameter('show_hz', 30.0)
        self.declare_parameter('keyboard_cmd_vel_topic', '/teleop/keyboard_cmd_vel')
        self.declare_parameter('keyboard_enabled_topic', '/teleop/keyboard_drive/enabled')
        self.declare_parameter('cmd_vel_mux_status_topic', '/teleop/cmd_vel_mux/status')
        self.declare_parameter('keyboard_linear_x_mps', 0.08)
        self.declare_parameter('keyboard_linear_y_mps', 0.08)
        self.declare_parameter('keyboard_angular_z_radps', 0.20)
        self.declare_parameter('click_jog_duration_s', 0.25)
        self.declare_parameter('mouse_hold_timeout_s', 0.75)
        self.declare_parameter('mouse_max_hold_s', 8.0)
        self.declare_parameter('operator_ok_topic', '/teleop/operator_ok')
        self.declare_parameter('ok_overlay_duration_s', 3.0)
        self.declare_parameter('headless_ok', True)

        self.window_title = str(self.get_parameter('window_title').value).strip()
        self.window_width = max(int(self.get_parameter('window_width').value), 320)
        self.window_height = max(int(self.get_parameter('window_height').value), 360)
        self.window_x = int(self.get_parameter('window_x').value)
        self.window_y = int(self.get_parameter('window_y').value)
        show_hz = max(float(self.get_parameter('show_hz').value), 5.0)
        self.tick_period_s = 1.0 / show_hz
        self.tick_period_ms = max(int(round(1000.0 / show_hz)), 1)
        self.keyboard_cmd_vel_topic = str(
            self.get_parameter('keyboard_cmd_vel_topic').value).strip()
        self.keyboard_enabled_topic = str(
            self.get_parameter('keyboard_enabled_topic').value).strip()
        self.cmd_vel_mux_status_topic = str(
            self.get_parameter('cmd_vel_mux_status_topic').value).strip()
        self.keyboard_linear_x_mps = max(
            float(self.get_parameter('keyboard_linear_x_mps').value), 0.0)
        self.keyboard_linear_y_mps = max(
            float(self.get_parameter('keyboard_linear_y_mps').value), 0.0)
        self.keyboard_angular_z_radps = max(
            float(self.get_parameter('keyboard_angular_z_radps').value), 0.0)
        self.click_jog_duration_s = max(
            float(self.get_parameter('click_jog_duration_s').value), 0.05)
        self.mouse_hold_timeout_s = max(
            float(self.get_parameter('mouse_hold_timeout_s').value), 0.5)
        self.mouse_max_hold_s = max(
            float(self.get_parameter('mouse_max_hold_s').value), 0.0)
        self.operator_ok_topic = str(self.get_parameter('operator_ok_topic').value).strip()
        self.ok_overlay_duration_s = max(
            float(self.get_parameter('ok_overlay_duration_s').value), 0.1)
        self.headless_ok = self._as_bool(self.get_parameter('headless_ok').value)

        self.drive_enabled = False
        self.focus_active = False
        self.pressed_actions = set()
        self.release_jobs = {}
        self.mouse_action = ''
        self.mouse_hold_started_s = 0.0
        self.status_text = 'LOCKED'
        self.active_text = '--'
        self.last_mux_status = {}
        self.last_mux_status_time_s = 0.0
        self.gui_available = False
        self.root = None
        self.tk = None
        self.headless_timer = None
        self.closing = False

        drive_qos = QoSProfile(depth=1)
        drive_qos.reliability = ReliabilityPolicy.BEST_EFFORT
        drive_qos.durability = DurabilityPolicy.VOLATILE
        enabled_qos = QoSProfile(depth=1)
        enabled_qos.reliability = ReliabilityPolicy.RELIABLE
        enabled_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.cmd_pub = self.create_publisher(Twist, self.keyboard_cmd_vel_topic, drive_qos)
        self.enabled_pub = self.create_publisher(Bool, self.keyboard_enabled_topic, enabled_qos)
        self.ok_pub = self.create_publisher(String, self.operator_ok_topic, 10)
        self.mux_status_sub = None
        if self.cmd_vel_mux_status_topic:
            self.mux_status_sub = self.create_subscription(
                String, self.cmd_vel_mux_status_topic, self._mux_status_callback, 10)

        self._init_window()
        self._publish_enabled()
        if not self.gui_available:
            self.headless_timer = self.create_timer(self.tick_period_s, self._headless_tick)
        self.get_logger().info(
            f'operator drive panel active: window={self.window_title!r}, '
            f'cmd={self.keyboard_cmd_vel_topic}, enabled={self.keyboard_enabled_topic}, '
            f'gui={self.gui_available}')

    def destroy_node(self):
        self.closing = True
        self.drive_enabled = False
        self._clear_motion('SHUTDOWN - STOPPED')
        self._publish_enabled()
        self._destroy_window()
        super().destroy_node()

    def run_gui(self):
        if self.root is None:
            return
        self.root.after(self.tick_period_ms, self._gui_tick)
        self.root.mainloop()

    def _as_bool(self, value):
        if isinstance(value, bool):
            return value
        if isinstance(value, str):
            return value.strip().lower() in ('1', 'true', 'yes', 'on')
        return bool(value)

    def _mux_connected(self):
        return (
            self.count_subscribers(self.keyboard_cmd_vel_topic) > 0 and
            self.count_subscribers(self.keyboard_enabled_topic) > 0
        )

    def _now_s(self):
        return self.get_clock().now().nanoseconds / 1e9

    def _mux_status_callback(self, msg):
        try:
            status = json.loads(msg.data)
        except (TypeError, ValueError):
            status = {}
        self.last_mux_status = status if isinstance(status, dict) else {}
        self.last_mux_status_time_s = self._now_s()

    def _mux_status_fresh(self):
        return (
            self.last_mux_status_time_s > 0.0 and
            (self._now_s() - self.last_mux_status_time_s) <= 1.0
        )

    def _owner_display(self, mux_connected):
        if not mux_connected:
            return 'OWNER: MUX OFF / NOT CONNECTED', '#6d5228'
        if not self._mux_status_fresh():
            return 'OWNER: WAITING FOR MUX STATUS', '#6d5228'

        owner = str(self.last_mux_status.get('owner', '')).strip().lower()
        if owner == 'monitor':
            return 'OWNER: MONITOR', '#2d7d4c'
        if owner == 'leader':
            return 'OWNER: LEADER', '#385d8a'
        if owner == 'monitor_stale':
            return 'OWNER: MONITOR STALE - STOPPED', '#8a5b2f'
        if owner == 'leader_stale':
            return 'OWNER: LEADER STALE - STOPPED', '#7a3232'
        return 'OWNER: UNKNOWN', '#6d5228'

    def _mux_text(self, mux_connected):
        if not mux_connected:
            return 'OFF - launch start_cmd_vel_mux:=true'
        if not self._mux_status_fresh():
            return 'connected, waiting status'
        owner = self.last_mux_status.get('owner', '--')
        output_topic = self.last_mux_status.get('output_topic', '--')
        return f'{owner} -> {output_topic}'

    def _init_window(self):
        if not os.environ.get('DISPLAY') and self.headless_ok:
            self.get_logger().warn('DISPLAY is not set; drive panel running headless')
            return
        try:
            import tkinter as tk
        except Exception as exc:
            self.get_logger().error(f'tkinter is unavailable; drive panel GUI disabled: {exc}')
            return

        try:
            root = tk.Tk()
        except Exception as exc:
            self.get_logger().error(f'failed to create drive panel window: {exc}')
            return

        self.tk = tk
        self.root = root
        self.gui_available = True
        root.title(self.window_title)
        root.configure(bg='#202226')
        root.minsize(PANEL_WIDTH, PANEL_HEIGHT)
        if self.window_x >= 0 and self.window_y >= 0:
            root.geometry(
                f'{self.window_width}x{self.window_height}+{self.window_x}+{self.window_y}')
        else:
            root.geometry(f'{self.window_width}x{self.window_height}')
        root.protocol('WM_DELETE_WINDOW', self._on_close)

        self.focus_var = tk.StringVar(value='NOT FOCUSED - CLICK PANEL')
        self.owner_var = tk.StringVar(value='OWNER: MUX OFF / NOT CONNECTED')
        self.drive_var = tk.StringVar(value='DRIVE LOCKED')
        self.status_var = tk.StringVar(value='status: LOCKED')
        self.active_var = tk.StringVar(value='active: --    mux: --')
        self.speed_var = tk.StringVar(value=self._speed_text())

        outer = tk.Frame(root, bg='#202226')
        outer.pack(fill='both', expand=True, padx=16, pady=14)

        self.focus_label = tk.Label(
            outer,
            textvariable=self.focus_var,
            bg='#4b5159',
            fg='#f6f8fa',
            font=('Helvetica', 13, 'bold'),
            height=2,
        )
        self.focus_label.pack(fill='x')

        self.owner_label = tk.Label(
            outer,
            textvariable=self.owner_var,
            bg='#6d5228',
            fg='#ffffff',
            font=('Helvetica', 14, 'bold'),
            height=2,
        )
        self.owner_label.pack(fill='x', pady=(10, 0))

        self.drive_button = tk.Button(
            outer,
            textvariable=self.drive_var,
            command=lambda: self._set_drive_enabled(not self.drive_enabled),
            font=('Helvetica', 15, 'bold'),
            height=2,
            relief='raised',
        )
        self.drive_button.pack(fill='x', pady=(12, 6))

        self.status_label = tk.Label(
            outer,
            textvariable=self.status_var,
            bg='#202226',
            fg='#e6edf3',
            anchor='w',
            font=('Helvetica', 11),
        )
        self.status_label.pack(fill='x', pady=(4, 0))

        self.active_label = tk.Label(
            outer,
            textvariable=self.active_var,
            bg='#202226',
            fg='#b7c2cc',
            anchor='w',
            font=('Helvetica', 10),
        )
        self.active_label.pack(fill='x', pady=(2, 0))

        self.speed_label = tk.Label(
            outer,
            textvariable=self.speed_var,
            bg='#202226',
            fg='#9ca6af',
            anchor='w',
            font=('Helvetica', 10),
        )
        self.speed_label.pack(fill='x', pady=(2, 12))

        grid = tk.Frame(outer, bg='#202226')
        grid.pack(fill='both', expand=True)
        for col in range(3):
            grid.grid_columnconfigure(col, weight=1)
        for row in range(4):
            grid.grid_rowconfigure(row, weight=1)

        self.action_buttons = {}
        self._make_motion_button(grid, 'FORWARD', 'forward', 0, 1)
        self._make_motion_button(grid, 'LEFT', 'left', 1, 0)
        stop_button = tk.Button(
            grid,
            text='STOP',
            command=lambda: self._clear_motion('STOP - STOPPED'),
            font=('Helvetica', 13, 'bold'),
            bg='#7a3232',
            fg='#ffffff',
            activebackground='#9b3f3f',
            activeforeground='#ffffff',
        )
        stop_button.grid(row=1, column=1, sticky='nsew', padx=5, pady=5)
        self._make_motion_button(grid, 'RIGHT', 'right', 1, 2)
        self._make_motion_button(grid, 'BACK', 'backward', 2, 1)
        self._make_motion_button(grid, 'ROT L', 'rot_left', 3, 0)
        self._make_motion_button(grid, 'ROT R', 'rot_right', 3, 2)

        ok_button = tk.Button(
            outer,
            text='OK SIGN',
            command=self._send_ok,
            font=('Helvetica', 12, 'bold'),
            bg='#2d7d4c',
            fg='#ffffff',
            activebackground='#35945a',
            activeforeground='#ffffff',
            height=2,
        )
        ok_button.pack(fill='x', pady=(12, 0))

        root.bind('<FocusIn>', self._on_focus_in)
        root.bind('<FocusOut>', self._on_focus_out)
        root.bind('<KeyPress>', self._on_key_press)
        root.bind('<KeyRelease>', self._on_key_release)
        root.bind('<Button-1>', self._focus_panel, add='+')
        root.bind_all('<ButtonRelease-1>', self._on_any_mouse_release, add='+')
        root.after(200, self._request_initial_focus)
        self._update_display()

    def _make_motion_button(self, parent, label, action, row, col):
        button = self.tk.Button(
            parent,
            text=label,
            font=('Helvetica', 13, 'bold'),
            bg='#344258',
            fg='#ffffff',
            activebackground='#415a78',
            activeforeground='#ffffff',
        )
        button.grid(row=row, column=col, sticky='nsew', padx=5, pady=5)
        button.bind('<ButtonPress-1>', lambda event, action=action: self._mouse_press(action))
        button.bind('<ButtonRelease-1>', lambda event, action=action: self._mouse_release(action))
        self.action_buttons[action] = button

    def _request_initial_focus(self):
        if self.root is None or self.closing:
            return
        try:
            self.root.focus_force()
        except Exception:
            self.root.focus_set()
        self._poll_focus()

    def _focus_panel(self, event=None):
        del event
        if self.root is not None:
            self.root.focus_set()

    def _on_focus_in(self, event=None):
        del event
        self.focus_active = True
        self._update_display()

    def _on_focus_out(self, event=None):
        del event
        if self.root is not None:
            self.root.after(20, self._poll_focus)

    def _poll_focus(self):
        if self.root is None or self.closing:
            return
        focused = self.root.focus_get() is not None
        if self.focus_active and not focused:
            self.focus_active = False
            self._clear_motion('FOCUS LOST - STOPPED')
        elif focused:
            self.focus_active = True
        self._update_display()

    def _on_key_press(self, event):
        key = str(event.keysym).lower()
        if key == 'k':
            self._set_drive_enabled(not self.drive_enabled)
            return
        if key == 'space':
            self._clear_motion('SPACE - STOPPED')
            return
        if key == 'o':
            self._send_ok()
            return

        action = KEY_ACTIONS.get(key)
        if action is None:
            return
        self._cancel_release_job(action)
        if not self._ready_for_manual_drive():
            return
        self.pressed_actions.add(action)
        self.status_text = f'KEY {ACTION_LABELS[action]}'
        self._publish_current_command()
        self._update_display()

    def _on_key_release(self, event):
        action = KEY_ACTIONS.get(str(event.keysym).lower())
        if action is None or self.root is None:
            return
        self._cancel_release_job(action)
        self.release_jobs[action] = self.root.after(
            KEY_RELEASE_DEBOUNCE_MS,
            lambda action=action: self._release_key_action(action),
        )

    def _release_key_action(self, action):
        self.release_jobs.pop(action, None)
        if action in self.pressed_actions:
            self.pressed_actions.discard(action)
            self.status_text = 'DRIVE ON - STOPPED' if not self._active_actions() else 'DRIVE ON'
            self._publish_current_command()
            self._update_display()

    def _cancel_release_job(self, action):
        job = self.release_jobs.pop(action, None)
        if job is not None and self.root is not None:
            try:
                self.root.after_cancel(job)
            except Exception:
                pass

    def _cancel_all_release_jobs(self):
        for action in list(self.release_jobs):
            self._cancel_release_job(action)

    def _mouse_press(self, action):
        if action not in MOVE_ACTIONS:
            return
        self._focus_panel()
        self.focus_active = True
        if not self._ready_for_manual_drive(require_focus=False):
            return
        self.mouse_action = action
        self.mouse_hold_started_s = time.time()
        self.status_text = f'MOUSE {ACTION_LABELS[action]}'
        self._publish_current_command()
        self._update_display()

    def _mouse_release(self, action):
        if self.mouse_action == action:
            self.mouse_action = ''
            self.mouse_hold_started_s = 0.0
            self.status_text = 'DRIVE ON - STOPPED' if not self._active_actions() else 'DRIVE ON'
            self._publish_current_command()
            self._update_display()

    def _on_any_mouse_release(self, event=None):
        del event
        if self.mouse_action:
            self.mouse_action = ''
            self.mouse_hold_started_s = 0.0
            self.status_text = 'DRIVE ON - STOPPED'
            self._publish_current_command()
            self._update_display()

    def _ready_for_manual_drive(self, require_focus=True):
        if not self.drive_enabled:
            self.status_text = 'LOCKED - CLICK DRIVE ON FIRST'
            self._publish_zero()
            self._update_display()
            return False
        if require_focus and not self.focus_active:
            self.status_text = 'NOT FOCUSED - CLICK PANEL'
            self._clear_motion('NOT FOCUSED - STOPPED')
            return False
        if not self._mux_connected():
            self.status_text = 'DRIVE ON - WAITING FOR MUX'
            self._update_display()
        return True

    def _set_drive_enabled(self, enabled):
        self.drive_enabled = bool(enabled)
        if self.drive_enabled and not self._mux_connected():
            status_text = 'DRIVE ON - WAITING FOR MUX'
        else:
            status_text = 'DRIVE ON' if self.drive_enabled else 'LOCKED'
        self._clear_motion(status_text)
        self._publish_enabled()
        self._update_display()

    def _clear_motion(self, status_text=None):
        self._cancel_all_release_jobs()
        self.pressed_actions.clear()
        self.mouse_action = ''
        self.mouse_hold_started_s = 0.0
        if status_text:
            self.status_text = status_text
        elif self.drive_enabled:
            self.status_text = 'DRIVE ON - STOPPED'
        self._publish_zero()
        self.active_text = '--'

    def _headless_tick(self):
        if self.drive_enabled:
            self.drive_enabled = False
            self._clear_motion('HEADLESS - DRIVE DISABLED')
            self._publish_enabled()

    def _gui_tick(self):
        if self.closing or self.root is None:
            return
        try:
            rclpy.spin_once(self, timeout_sec=0.0)
        except Exception as exc:
            self.get_logger().warn(f'ROS spin_once failed in drive panel: {exc}')
        self._poll_focus()
        self._publish_drive_tick()
        self._update_display()
        if not self.closing and self.root is not None:
            self.root.after(self.tick_period_ms, self._gui_tick)

    def _publish_drive_tick(self):
        if not self.drive_enabled:
            return
        if self.mouse_action and self.mouse_max_hold_s > 0.0:
            hold_age_s = time.time() - self.mouse_hold_started_s
            if hold_age_s > self.mouse_max_hold_s:
                self.mouse_action = ''
                self.mouse_hold_started_s = 0.0
                self.status_text = 'MOUSE HOLD TIMEOUT - STOPPED'
        self._publish_current_command()

    def _publish_current_command(self):
        twist, active_text = self._make_twist()
        self.active_text = active_text
        if self.drive_enabled:
            self.cmd_pub.publish(twist)

    def _publish_zero(self):
        self.cmd_pub.publish(Twist())

    def _publish_enabled(self):
        msg = Bool()
        msg.data = bool(self.drive_enabled)
        self.enabled_pub.publish(msg)

    def _active_actions(self):
        actions = set(self.pressed_actions)
        if self.mouse_action:
            actions.add(self.mouse_action)
        return actions

    def _make_twist(self):
        actions = self._active_actions()
        twist = Twist()
        twist.linear.x = self._axis_value(actions, 'forward', 'backward',
                                          self.keyboard_linear_x_mps)
        twist.linear.y = self._axis_value(actions, 'left', 'right',
                                          self.keyboard_linear_y_mps)
        twist.angular.z = self._axis_value(actions, 'rot_left', 'rot_right',
                                           self.keyboard_angular_z_radps)
        active = [ACTION_LABELS[action] for action in MOVE_ACTIONS if action in actions]
        return twist, '+'.join(active) if active else '--'

    def _axis_value(self, actions, positive_action, negative_action, magnitude):
        positive = positive_action in actions
        negative = negative_action in actions
        if positive and not negative:
            return magnitude
        if negative and not positive:
            return -magnitude
        return 0.0

    def _speed_text(self):
        return (
            f'speed: x/y {self.keyboard_linear_x_mps:.2f} m/s, '
            f'rot {self.keyboard_angular_z_radps:.2f} rad/s'
        )

    def _update_display(self):
        if not self.gui_available or self.root is None:
            return
        mux_connected = self._mux_connected()
        owner_text, owner_bg = self._owner_display(mux_connected)
        self.owner_var.set(owner_text)
        self.owner_label.configure(bg=owner_bg)
        if self.focus_active:
            self.focus_var.set('FOCUSED - WASD ACTIVE')
            self.focus_label.configure(bg='#2d7d4c')
        else:
            self.focus_var.set('NOT FOCUSED - CLICK PANEL')
            self.focus_label.configure(bg='#7a5a2d')

        if self.drive_enabled and not mux_connected:
            drive_text = 'DRIVE ON - WAITING FOR MUX'
            drive_bg = '#6d5228'
        elif self.drive_enabled:
            drive_text = 'DRIVE ON - MONITOR CONTROL'
            drive_bg = '#2d7d4c'
        elif mux_connected:
            drive_text = 'DRIVE LOCKED - LEADER CONTROL'
            drive_bg = '#5b4444'
        else:
            drive_text = 'DRIVE LOCKED / MUX OFF'
            drive_bg = '#6d5228'
        self.drive_var.set(drive_text)
        self.drive_button.configure(bg=drive_bg, fg='#ffffff', activebackground=drive_bg)
        self.status_var.set(f'status: {self.status_text}')
        self.active_var.set(f'active: {self.active_text}    mux: {self._mux_text(mux_connected)}')
        self.speed_var.set(self._speed_text())

        button_bg = '#344258' if self.drive_enabled else '#33363d'
        active_bg = '#4f6f91'
        for action, button in self.action_buttons.items():
            bg = active_bg if action in self._active_actions() else button_bg
            button.configure(bg=bg, activebackground=active_bg)

    def _send_ok(self):
        payload = {
            'stamp_sec': self.get_clock().now().nanoseconds / 1e9,
            'event': 'ok',
            'source': 'drive_panel',
            'duration_s': self.ok_overlay_duration_s,
        }
        msg = String()
        msg.data = json.dumps(payload, sort_keys=True)
        self.ok_pub.publish(msg)
        self.status_text = 'OK SIGN SENT'
        self._update_display()

    def _on_close(self):
        self.closing = True
        self.drive_enabled = False
        self._clear_motion('CLOSED - STOPPED')
        self._publish_enabled()
        self._destroy_window()

    def _destroy_window(self):
        if self.root is None:
            return
        root = self.root
        self.root = None
        self.gui_available = False
        try:
            root.destroy()
        except Exception:
            pass


def main(args=None):
    rclpy.init(args=args)
    node = OperatorDrivePanel()
    try:
        if node.gui_available:
            node.run_gui()
        else:
            rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
