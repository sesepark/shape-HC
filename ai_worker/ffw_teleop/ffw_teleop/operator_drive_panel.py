import json
import os
import time

import cv2
from geometry_msgs.msg import Twist
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from std_msgs.msg import Bool
from std_msgs.msg import String


PANEL_WIDTH = 420
PANEL_HEIGHT = 520


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
        self.declare_parameter('keyboard_linear_x_mps', 0.10)
        self.declare_parameter('keyboard_linear_y_mps', 0.10)
        self.declare_parameter('keyboard_angular_z_radps', 0.25)
        self.declare_parameter('click_jog_duration_s', 0.25)
        self.declare_parameter('mouse_hold_timeout_s', 0.75)
        self.declare_parameter('operator_ok_topic', '/teleop/operator_ok')
        self.declare_parameter('ok_overlay_duration_s', 3.0)
        self.declare_parameter('headless_ok', True)

        self.window_title = str(self.get_parameter('window_title').value).strip()
        self.window_width = max(int(self.get_parameter('window_width').value), 320)
        self.window_height = max(int(self.get_parameter('window_height').value), 360)
        self.window_x = int(self.get_parameter('window_x').value)
        self.window_y = int(self.get_parameter('window_y').value)
        show_hz = max(float(self.get_parameter('show_hz').value), 5.0)
        self.keyboard_cmd_vel_topic = str(
            self.get_parameter('keyboard_cmd_vel_topic').value).strip()
        self.keyboard_enabled_topic = str(
            self.get_parameter('keyboard_enabled_topic').value).strip()
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
        self.operator_ok_topic = str(self.get_parameter('operator_ok_topic').value).strip()
        self.ok_overlay_duration_s = max(
            float(self.get_parameter('ok_overlay_duration_s').value), 0.1)
        self.headless_ok = self._as_bool(self.get_parameter('headless_ok').value)

        self.drive_enabled = False
        self.active_action = ''
        self.active_until_s = 0.0
        self.mouse_hold_action = ''
        self.mouse_is_down = False
        self.status_text = 'LOCKED'
        self.gui_available = False
        self.buttons = []
        self.draw_width = self.window_width
        self.draw_height = self.window_height

        drive_qos = QoSProfile(depth=1)
        drive_qos.reliability = ReliabilityPolicy.BEST_EFFORT
        drive_qos.durability = DurabilityPolicy.VOLATILE
        enabled_qos = QoSProfile(depth=1)
        enabled_qos.reliability = ReliabilityPolicy.RELIABLE
        enabled_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.cmd_pub = self.create_publisher(Twist, self.keyboard_cmd_vel_topic, drive_qos)
        self.enabled_pub = self.create_publisher(Bool, self.keyboard_enabled_topic, enabled_qos)
        self.ok_pub = self.create_publisher(String, self.operator_ok_topic, 10)

        self._init_window()
        self._publish_enabled()
        self.timer = self.create_timer(1.0 / show_hz, self._tick)
        self.get_logger().info(
            f'operator drive panel active: window={self.window_title!r}, '
            f'cmd={self.keyboard_cmd_vel_topic}, enabled={self.keyboard_enabled_topic}')

    def destroy_node(self):
        self.drive_enabled = False
        self._publish_enabled()
        self._stop_motion()
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

    def _mux_connected(self):
        return (
            self.count_subscribers(self.keyboard_cmd_vel_topic) > 0 and
            self.count_subscribers(self.keyboard_enabled_topic) > 0
        )

    def _init_window(self):
        if not os.environ.get('DISPLAY') and self.headless_ok:
            self.get_logger().warn('DISPLAY is not set; drive panel running headless')
            return
        try:
            cv2.namedWindow(self.window_title, cv2.WINDOW_NORMAL)
            cv2.resizeWindow(self.window_title, self.window_width, self.window_height)
            cv2.setMouseCallback(self.window_title, self._mouse_callback)
            if self.window_x >= 0 and self.window_y >= 0:
                cv2.moveWindow(self.window_title, self.window_x, self.window_y)
            self.gui_available = True
        except cv2.error as exc:
            self.get_logger().error(f'failed to create drive panel window: {exc}')
            self.gui_available = False

    def _tick(self):
        now_s = time.time()
        if self.mouse_is_down and self.mouse_hold_action and self.drive_enabled:
            self.active_until_s = now_s + self.mouse_hold_timeout_s
        if self.active_action and now_s > self.active_until_s:
            self._stop_motion()
        if self.drive_enabled and self.active_action:
            self.cmd_pub.publish(self._make_twist(self.active_action))
        if not self.drive_enabled:
            self.active_action = ''
            self.active_until_s = 0.0
            self.mouse_hold_action = ''

        if not self.gui_available:
            return
        self._update_draw_size()
        frame = self._draw_panel()
        try:
            cv2.imshow(self.window_title, frame)
            key = cv2.waitKeyEx(1)
        except cv2.error as exc:
            self.get_logger().error(f'drive panel failed; disabling GUI: {exc}')
            self.gui_available = False
            return
        if key == 27:
            self.gui_available = False
            cv2.destroyWindow(self.window_title)
        elif key != -1:
            self._handle_key(key)

    def _update_draw_size(self):
        if not hasattr(cv2, 'getWindowImageRect'):
            self.draw_width = self.window_width
            self.draw_height = self.window_height
            return
        try:
            _, _, width, height = cv2.getWindowImageRect(self.window_title)
        except cv2.error:
            width = self.window_width
            height = self.window_height
        self.draw_width = max(int(width), PANEL_WIDTH)
        self.draw_height = max(int(height), PANEL_HEIGHT)

    def _draw_panel(self):
        width = self.draw_width
        height = self.draw_height
        scale = min(width / PANEL_WIDTH, height / PANEL_HEIGHT)
        image = np.full((height, width, 3), (24, 26, 30), dtype=np.uint8)
        self.buttons = []
        mux_connected = self._mux_connected()
        enabled_color = (42, 138, 72) if self.drive_enabled else (70, 58, 58)
        if self.drive_enabled:
            drive_label = 'DRIVE ON'
        elif mux_connected:
            drive_label = 'DRIVE LOCKED'
        else:
            drive_label = 'DRIVE LOCKED / MUX OFF'
        self._button(image, 'toggle', drive_label, (24, 24, width - 48, 56),
                     enabled_color, scale)
        self._put_text(
            image,
            f'status: {self.status_text}',
            (28, 104),
            0.56 * scale,
            (220, 226, 232),
            1,
        )
        mux_text = 'connected' if mux_connected else 'OFF - launch start_cmd_vel_mux:=true'
        self._put_text(
            image,
            f'active: {self.active_action or "--"}    mux: {mux_text}',
            (28, 130),
            0.44 * scale,
            (120, 205, 145) if mux_connected else (95, 170, 240),
            1,
        )
        self._put_text(
            image,
            f'jog {self.keyboard_linear_x_mps:.2f}m/s {self.keyboard_angular_z_radps:.2f}rad/s',
            (28, 154),
            0.44 * scale,
            (178, 186, 196),
            1,
        )

        cx = width // 2
        button_color = (54, 68, 88) if self.drive_enabled else (48, 50, 56)
        self._button(image, 'forward', 'FORWARD', (cx - 70, 176, 140, 58), button_color, scale)
        self._button(image, 'left', 'LEFT', (cx - 164, 248, 116, 58), button_color, scale)
        self._button(image, 'stop', 'STOP', (cx - 58, 244, 116, 66), (60, 46, 46), scale)
        self._button(image, 'right', 'RIGHT', (cx + 48, 248, 116, 58), button_color, scale)
        self._button(image, 'backward', 'BACK', (cx - 70, 322, 140, 58), button_color, scale)
        self._button(image, 'rot_left', 'ROT L', (42, 402, 130, 52),
                     (48, 74, 72) if self.drive_enabled else (48, 50, 56), scale)
        self._button(image, 'rot_right', 'ROT R', (width - 172, 402, 130, 52),
                     (48, 74, 72) if self.drive_enabled else (48, 50, 56), scale)
        self._button(image, 'ok', 'OK SIGN', (42, height - 68, width - 84, 48),
                     (42, 122, 72), scale)
        return image

    def _button(self, image, action, label, rect, color, scale=1.0):
        x, y, width, height = rect
        x = int(np.clip(x, 4, image.shape[1] - 8))
        y = int(np.clip(y, 4, image.shape[0] - 8))
        width = int(np.clip(width, 16, image.shape[1] - x - 4))
        height = int(np.clip(height, 16, image.shape[0] - y - 4))
        if action == self.active_action:
            color = tuple(min(int(channel) + 42, 255) for channel in color)
        cv2.rectangle(image, (x, y), (x + width, y + height), color, -1)
        cv2.rectangle(image, (x, y), (x + width, y + height), (160, 168, 180), 1)
        text_scale = max(0.46, 0.62 * scale)
        size, _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, text_scale, 2)
        tx = x + max((width - size[0]) // 2, 4)
        ty = y + (height + size[1]) // 2
        self._put_text(image, label, (tx, ty), text_scale, (244, 246, 248), 2)
        self.buttons.append({'action': action, 'rect': (x, y, width, height)})

    def _put_text(self, image, text, origin, scale, color, thickness):
        cv2.putText(
            image, str(text), origin, cv2.FONT_HERSHEY_SIMPLEX, scale,
            color, thickness, cv2.LINE_AA)

    def _mouse_callback(self, event, x, y, flags, param):
        del param
        if event == cv2.EVENT_LBUTTONDOWN:
            action = self._button_at(x, y)
            if action in ('forward', 'backward', 'left', 'right', 'rot_left', 'rot_right'):
                self.mouse_hold_action = action
                self.mouse_is_down = True
                self._start_motion(action, 'HOLD', self.mouse_hold_timeout_s)
            else:
                self._handle_click(action)
        elif event == cv2.EVENT_MOUSEMOVE and self.mouse_hold_action:
            if flags & cv2.EVENT_FLAG_LBUTTON:
                self.mouse_is_down = True
                self.active_until_s = time.time() + self.mouse_hold_timeout_s
            else:
                self._stop_motion()
        elif event == cv2.EVENT_LBUTTONUP and self.mouse_hold_action:
            self._stop_motion()

    def _button_at(self, x, y):
        for button in self.buttons:
            bx, by, width, height = button['rect']
            if bx <= x <= bx + width and by <= y <= by + height:
                return button['action']
        return ''

    def _handle_click(self, action):
        if not action:
            return
        if action == 'toggle':
            self._set_drive_enabled(not self.drive_enabled)
        elif action == 'stop':
            self._stop_motion()
        elif action == 'ok':
            self._send_ok()
        elif action in ('forward', 'backward', 'left', 'right', 'rot_left', 'rot_right'):
            self._start_motion(action, 'CLICK', self.click_jog_duration_s)

    def _handle_key(self, key):
        key_low = key & 0xFF
        if key_low in (ord('k'), ord('K')):
            self._set_drive_enabled(not self.drive_enabled)
            return
        if key_low == ord(' '):
            self._stop_motion()
            return
        if key_low in (ord('o'), ord('O')):
            self._send_ok()
            return

        exact_key_actions = {
            82: 'forward',
            65362: 'forward',
            2490368: 'forward',
            84: 'backward',
            65364: 'backward',
            2621440: 'backward',
            81: 'left',
            65361: 'left',
            2424832: 'left',
            83: 'right',
            65363: 'right',
            2555904: 'right',
        }
        key_low_actions = {
            82: 'forward',
            84: 'backward',
            81: 'left',
            83: 'right',
            ord('w'): 'forward',
            ord('W'): 'forward',
            ord('s'): 'backward',
            ord('S'): 'backward',
            ord('a'): 'left',
            ord('A'): 'left',
            ord('d'): 'right',
            ord('D'): 'right',
            ord('q'): 'rot_left',
            ord('Q'): 'rot_left',
            ord('e'): 'rot_right',
            ord('E'): 'rot_right',
        }
        action = exact_key_actions.get(key) or key_low_actions.get(key_low)
        if action is not None:
            self._start_motion(action, 'KEY', self.click_jog_duration_s)

    def _start_motion(self, action, source, duration_s):
        if not self.drive_enabled:
            self.status_text = 'LOCKED - CLICK DRIVE LOCKED FIRST'
            self._stop_motion()
            return
        if not self._mux_connected():
            self.status_text = 'LOCKED / MUX OFF'
            self._set_drive_enabled(False)
            return
        self.active_action = action
        self.active_until_s = time.time() + max(float(duration_s), 0.05)
        self.status_text = f'{source} {action.upper()}'
        self.cmd_pub.publish(self._make_twist(action))

    def _set_drive_enabled(self, enabled):
        if enabled and not self._mux_connected():
            self.drive_enabled = False
            self._stop_motion()
            self._publish_enabled()
            self.status_text = 'LOCKED / MUX OFF'
            return
        self.drive_enabled = bool(enabled)
        self._stop_motion()
        self._publish_enabled()
        self.status_text = 'DRIVE ON' if self.drive_enabled else 'LOCKED'

    def _publish_enabled(self):
        msg = Bool()
        msg.data = bool(self.drive_enabled)
        self.enabled_pub.publish(msg)

    def _stop_motion(self):
        self.active_action = ''
        self.active_until_s = 0.0
        self.mouse_hold_action = ''
        self.mouse_is_down = False
        if self.drive_enabled:
            self.status_text = 'DRIVE ON - STOPPED'
        self.cmd_pub.publish(Twist())

    def _make_twist(self, action):
        twist = Twist()
        if action == 'forward':
            twist.linear.x = self.keyboard_linear_x_mps
        elif action == 'backward':
            twist.linear.x = -self.keyboard_linear_x_mps
        elif action == 'left':
            twist.linear.y = self.keyboard_linear_y_mps
        elif action == 'right':
            twist.linear.y = -self.keyboard_linear_y_mps
        elif action == 'rot_left':
            twist.angular.z = self.keyboard_angular_z_radps
        elif action == 'rot_right':
            twist.angular.z = -self.keyboard_angular_z_radps
        return twist

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


def main(args=None):
    rclpy.init(args=args)
    node = OperatorDrivePanel()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
