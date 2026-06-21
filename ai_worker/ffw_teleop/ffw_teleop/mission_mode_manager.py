import copy
import json
import os

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String


DEFAULT_MISSIONS = {
    'A': {
        'title': 'Mission A - Part Sorting',
        'summary': 'Identify requested parts and place the selected parts into the tray.',
        'specific_features': [
            'Monitor command parser placeholder',
            'Part detection overlay placeholder',
            'Tray placement assist placeholder',
        ],
    },
    'B': {
        'title': 'Mission B - Box Transport',
        'summary': 'Grasp boxes, move to the target table, and place them at the marked area.',
        'specific_features': [
            'Dual-arm box grasp assist placeholder',
            'Stop line and target table assist placeholder',
            'Transport cycle counter placeholder',
        ],
    },
    'C': {
        'title': 'Mission C - Sequential Assembly',
        'summary': 'Follow the monitor order and insert the requested nuts onto the peg set.',
        'specific_features': [
            'Assembly order parser placeholder',
            'Peg diameter and position assist placeholder',
            'Completion button assist placeholder',
        ],
    },
    'D': {
        'title': 'Mission D - Wheel Mounting',
        'summary': 'Mount the wheel, insert the bolt, and complete tightening with the drill.',
        'specific_features': [
            'Wheel and hub alignment assist placeholder',
            'Bolt and drill grasp assist placeholder',
            'Fastening alignment assist placeholder',
        ],
    },
}

MISSION_COLORS_BGR = {
    'A': (64, 128, 236),
    'B': (54, 166, 92),
    'C': (192, 132, 52),
    'D': (92, 92, 220),
}

MISSION_COLORS_TK = {
    'A': '#ec8040',
    'B': '#5ca636',
    'C': '#3484c0',
    'D': '#dc5c5c',
}


class MissionModeManager(Node):

    def __init__(self):
        super().__init__('mission_mode_manager')

        self.declare_parameter('profiles_config', '')
        self.declare_parameter('select_topic', '/teleop/mission/select')
        self.declare_parameter('mission_id_topic', '/teleop/mission/id')
        self.declare_parameter('mission_state_topic', '/teleop/mission/state')
        self.declare_parameter('mission_panel_topic', '/teleop/mission_panel/compressed')
        self.declare_parameter('publish_hz', 2.0)
        self.declare_parameter('panel_width', 900)
        self.declare_parameter('panel_height', 360)
        self.declare_parameter('initial_mission', '')
        self.declare_parameter('gui_enabled', True)

        self.profiles_config = str(self.get_parameter('profiles_config').value).strip()
        self.select_topic = str(self.get_parameter('select_topic').value).strip()
        self.mission_id_topic = str(self.get_parameter('mission_id_topic').value).strip()
        self.mission_state_topic = str(self.get_parameter('mission_state_topic').value).strip()
        self.mission_panel_topic = str(self.get_parameter('mission_panel_topic').value).strip()
        publish_hz = max(float(self.get_parameter('publish_hz').value), 0.2)
        self.panel_width = max(int(self.get_parameter('panel_width').value), 320)
        self.panel_height = max(int(self.get_parameter('panel_height').value), 180)
        self.gui_enabled = self._as_bool(self.get_parameter('gui_enabled').value)

        profile_data = self._load_profiles(self.profiles_config)
        self.missions = profile_data['missions']
        initial = str(self.get_parameter('initial_mission').value).strip().upper()
        self.active_mission = self._normalize_mission(
            initial or profile_data.get('default_mission') or 'A') or 'A'
        self.updated_by = 'startup'
        self.root = None
        self.tk = None
        self.gui_widgets = {}

        state_qos = QoSProfile(depth=1)
        state_qos.reliability = ReliabilityPolicy.RELIABLE
        state_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.mission_id_pub = self.create_publisher(
            String, self.mission_id_topic, state_qos)
        self.mission_state_pub = self.create_publisher(
            String, self.mission_state_topic, state_qos)
        self.panel_pub = self.create_publisher(
            CompressedImage, self.mission_panel_topic, 1)
        self.select_sub = self.create_subscription(
            String, self.select_topic, self._select_callback, 10)
        self.timer = self.create_timer(1.0 / publish_hz, self._publish_state)

        self._init_gui()
        self._publish_state()
        self.get_logger().info(
            f'mission mode manager active: mission={self.active_mission}, '
            f'select={self.select_topic}, state={self.mission_state_topic}, '
            f'panel={self.mission_panel_topic}')

    def run(self):
        if self.root is None:
            rclpy.spin(self)
            return
        self._schedule_ros_spin()
        self.root.mainloop()

    def _as_bool(self, value):
        if isinstance(value, bool):
            return value
        if isinstance(value, str):
            return value.strip().lower() in ('1', 'true', 'yes', 'on')
        return bool(value)

    def _load_profiles(self, path):
        missions = copy.deepcopy(DEFAULT_MISSIONS)
        data = {
            'default_mission': 'A',
            'missions': missions,
        }
        if not path:
            return data
        try:
            import yaml
            with open(os.path.expanduser(path), 'r', encoding='utf-8') as stream:
                loaded = yaml.safe_load(stream) or {}
        except OSError as exc:
            self.get_logger().warn(f'failed to read mission profiles {path}: {exc}')
            return data
        except Exception as exc:
            self.get_logger().warn(f'failed to parse mission profiles {path}: {exc}')
            return data

        loaded_missions = loaded.get('missions') or {}
        for mission_id, profile in loaded_missions.items():
            key = self._normalize_mission(mission_id)
            if key is None or not isinstance(profile, dict):
                continue
            missions[key] = {
                'title': str(profile.get('title') or missions.get(key, {}).get('title') or key),
                'summary': str(profile.get('summary') or ''),
                'specific_features': [
                    str(item) for item in profile.get('specific_features') or []
                ],
            }
        default = self._normalize_mission(loaded.get('default_mission')) or 'A'
        return {
            'default_mission': default,
            'missions': missions,
        }

    def _normalize_mission(self, value):
        text = str(value or '').strip().upper()
        if not text:
            return None
        if text.startswith('MISSION '):
            text = text.split()[-1]
        key = text[0]
        if key in ('A', 'B', 'C', 'D'):
            return key
        return None

    def _select_callback(self, msg):
        mission = self._normalize_mission(msg.data)
        if mission is None:
            self.get_logger().warn(f'ignoring unknown mission selection: {msg.data!r}')
            return
        self._set_mission(mission, 'topic')

    def _set_mission(self, mission, source):
        mission = self._normalize_mission(mission)
        if mission is None:
            return
        changed = mission != self.active_mission
        self.active_mission = mission
        self.updated_by = source
        self._update_gui()
        self._publish_state()
        if changed:
            self.get_logger().info(f'mission switched to {mission} by {source}')

    def _state_payload(self):
        profile = self.missions.get(self.active_mission, {})
        return {
            'stamp_sec': self.get_clock().now().nanoseconds / 1e9,
            'mission_id': self.active_mission,
            'title': str(profile.get('title') or self.active_mission),
            'summary': str(profile.get('summary') or ''),
            'specific_features': [
                str(item) for item in profile.get('specific_features') or []
            ],
            'updated_by': self.updated_by,
            'available_missions': sorted(self.missions.keys()),
        }

    def _publish_state(self):
        payload = self._state_payload()
        mission_msg = String()
        mission_msg.data = self.active_mission
        self.mission_id_pub.publish(mission_msg)

        state_msg = String()
        state_msg.data = json.dumps(payload, sort_keys=True)
        self.mission_state_pub.publish(state_msg)

        panel = self._render_panel(payload)
        ok, encoded = cv2.imencode('.jpg', panel, [int(cv2.IMWRITE_JPEG_QUALITY), 86])
        if ok:
            image_msg = CompressedImage()
            image_msg.header.stamp = self.get_clock().now().to_msg()
            image_msg.header.frame_id = 'teleop_mission_panel'
            image_msg.format = 'jpeg'
            image_msg.data = encoded.tobytes()
            self.panel_pub.publish(image_msg)

    def _render_panel(self, payload):
        width = self.panel_width
        height = self.panel_height
        mission_id = str(payload.get('mission_id') or 'A')
        color = MISSION_COLORS_BGR.get(mission_id, (80, 80, 80))
        image = np.full((height, width, 3), (30, 32, 36), dtype=np.uint8)
        cv2.rectangle(image, (0, 0), (width - 1, 72), color, -1)
        self._put_text(
            image,
            f'{mission_id}  {payload.get("title", "")}',
            (20, 46),
            0.95,
            (255, 255, 255),
            2,
        )
        self._put_text(
            image,
            'MISSION MODE',
            (width - 190, 46),
            0.55,
            (255, 255, 255),
            1,
        )

        y = 112
        y = self._draw_wrapped(
            image,
            str(payload.get('summary') or ''),
            (20, y),
            width - 40,
            0.58,
            (234, 238, 242),
        )

        y += 18
        self._put_text(image, 'Specific feature slots', (20, y), 0.56, (210, 216, 222), 1)
        y += 30
        for feature in payload.get('specific_features') or []:
            y = self._draw_wrapped(
                image,
                f'- {feature}',
                (36, y),
                width - 70,
                0.50,
                (235, 238, 240),
            ) + 4
            if y > height - 55:
                break

        button_y = height - 38
        for index, key in enumerate(('A', 'B', 'C', 'D')):
            x0 = 20 + index * 70
            is_active = key == mission_id
            fill = MISSION_COLORS_BGR.get(key, (80, 80, 80)) if is_active else (54, 56, 62)
            cv2.rectangle(image, (x0, button_y - 23), (x0 + 52, button_y + 8), fill, -1)
            cv2.rectangle(image, (x0, button_y - 23), (x0 + 52, button_y + 8), (190, 196, 204), 1)
            self._put_text(image, key, (x0 + 18, button_y), 0.58, (255, 255, 255), 1)
        self._put_text(
            image,
            f'Updated by {payload.get("updated_by", "--")}',
            (width - 245, height - 16),
            0.45,
            (200, 206, 214),
            1,
        )
        return image

    def _draw_wrapped(self, image, text, origin, max_width, scale, color):
        words = str(text).split()
        if not words:
            return origin[1]
        x, y = origin
        line = ''
        for word in words:
            candidate = word if not line else f'{line} {word}'
            size, _ = cv2.getTextSize(candidate, cv2.FONT_HERSHEY_SIMPLEX, scale, 1)
            if size[0] > max_width and line:
                self._put_text(image, line, (x, y), scale, color, 1)
                y += int(28 * scale) + 12
                line = word
            else:
                line = candidate
        if line:
            self._put_text(image, line, (x, y), scale, color, 1)
            y += int(28 * scale) + 12
        return y

    def _put_text(self, image, text, origin, scale, color, thickness):
        cv2.putText(
            image, str(text), origin, cv2.FONT_HERSHEY_SIMPLEX, scale,
            (0, 0, 0), thickness + 2, cv2.LINE_AA)
        cv2.putText(
            image, str(text), origin, cv2.FONT_HERSHEY_SIMPLEX, scale,
            color, thickness, cv2.LINE_AA)

    def _init_gui(self):
        if not self.gui_enabled:
            return
        if not os.environ.get('DISPLAY'):
            self.get_logger().warn('DISPLAY is not set; mission control GUI disabled')
            return
        try:
            import tkinter as tk
            root = tk.Tk()
        except Exception as exc:
            self.get_logger().warn(f'mission control GUI disabled: {exc}')
            return

        self.tk = tk
        self.root = root
        root.title('Mission Control')
        root.geometry('380x360+80+80')
        root.configure(bg='#202226')
        root.protocol('WM_DELETE_WINDOW', root.iconify)
        root.bind('<Key>', self._on_key_press)

        title = tk.Label(
            root,
            text='Mission Control',
            bg='#202226',
            fg='#f2f5f7',
            font=('Helvetica', 18, 'bold'),
            anchor='w',
        )
        title.pack(fill='x', padx=16, pady=(14, 4))

        active = tk.Label(
            root,
            text='',
            bg='#202226',
            fg='#f2f5f7',
            font=('Helvetica', 14, 'bold'),
            anchor='w',
        )
        active.pack(fill='x', padx=16, pady=(0, 10))
        self.gui_widgets['active'] = active

        button_frame = tk.Frame(root, bg='#202226')
        button_frame.pack(fill='x', padx=14, pady=4)
        self.gui_widgets['buttons'] = {}
        for mission_id in ('A', 'B', 'C', 'D'):
            button = tk.Button(
                button_frame,
                text=mission_id,
                width=5,
                height=2,
                command=lambda key=mission_id: self._set_mission(key, 'gui'),
                font=('Helvetica', 14, 'bold'),
            )
            button.pack(side='left', padx=4)
            self.gui_widgets['buttons'][mission_id] = button

        summary = tk.Label(
            root,
            text='',
            bg='#202226',
            fg='#d7dde2',
            justify='left',
            wraplength=340,
            anchor='nw',
            font=('Helvetica', 11),
        )
        summary.pack(fill='both', expand=True, padx=16, pady=(12, 4))
        self.gui_widgets['summary'] = summary

        hint = tk.Label(
            root,
            text='Keys: A / B / C / D',
            bg='#202226',
            fg='#9ca6af',
            font=('Helvetica', 10),
            anchor='w',
        )
        hint.pack(fill='x', padx=16, pady=(0, 12))
        self._update_gui()

    def _on_key_press(self, event):
        mission = self._normalize_mission(getattr(event, 'char', ''))
        if mission is not None:
            self._set_mission(mission, 'keyboard')

    def _update_gui(self):
        if self.root is None:
            return
        profile = self.missions.get(self.active_mission, {})
        active = self.gui_widgets.get('active')
        if active is not None:
            active.configure(text=f'Active: {self.active_mission}  {profile.get("title", "")}')
        summary = self.gui_widgets.get('summary')
        if summary is not None:
            features = '\n'.join(
                f'- {item}' for item in profile.get('specific_features') or [])
            summary.configure(text=f'{profile.get("summary", "")}\n\n{features}')
        for mission_id, button in (self.gui_widgets.get('buttons') or {}).items():
            if mission_id == self.active_mission:
                button.configure(
                    bg=MISSION_COLORS_TK.get(mission_id, '#555555'),
                    fg='#ffffff',
                    relief='sunken',
                    activebackground=MISSION_COLORS_TK.get(mission_id, '#555555'),
                )
            else:
                button.configure(
                    bg='#3a3d43',
                    fg='#ffffff',
                    relief='raised',
                    activebackground='#555a62',
                )

    def _schedule_ros_spin(self):
        if self.root is None:
            return
        if rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.0)
            self.root.after(20, self._schedule_ros_spin)
        else:
            self.root.quit()


def main(args=None):
    rclpy.init(args=args)
    node = MissionModeManager()
    try:
        node.run()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
