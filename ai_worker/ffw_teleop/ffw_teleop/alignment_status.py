import json
import math
import os

import cv2
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
from std_msgs.msg import Float32
from std_msgs.msg import String


class AlignmentStatus(Node):

    def __init__(self):
        super().__init__('teleop_alignment_status')

        self.declare_parameter('right_goal_topic', '/r_goal_pose')
        self.declare_parameter('left_goal_topic', '/l_goal_pose')
        self.declare_parameter('right_current_topic', '/r_gripper_pose')
        self.declare_parameter('left_current_topic', '/l_gripper_pose')
        self.declare_parameter('status_topic', '/teleop/alignment_status')
        self.declare_parameter('ok_topic', '/teleop/alignment_ok')
        self.declare_parameter('publish_hz', 10.0)
        self.declare_parameter('pos_threshold_m', 0.30)
        self.declare_parameter('ori_threshold_deg', 120.0)
        self.declare_parameter('stale_timeout_s', 1.0)
        self.declare_parameter('record_practice_events', True)
        self.declare_parameter('practice_event_log_path', '~/teleop_practice_events.jsonl')
        self.declare_parameter('practice_event_input_topic', '/teleop/practice_event/mark')
        self.declare_parameter('practice_event_output_topic', '/teleop/practice_event')
        self.declare_parameter('tact_trigger_topic', '/leader/joystick_controller/tact_trigger')
        self.declare_parameter(
            'joystick_mode_topic', '/leader/joystick_controller_right/joystick_mode')
        self.declare_parameter('joint_state_topic', '/joint_states')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter(
            'right_center_distance_topic', '/teleop/wrist_right/center_distance_m')
        self.declare_parameter(
            'left_center_distance_topic', '/teleop/wrist_left/center_distance_m')
        self.declare_parameter(
            'right_depth_metrics_topic', '/teleop/wrist_right/depth_metrics')
        self.declare_parameter(
            'left_depth_metrics_topic', '/teleop/wrist_left/depth_metrics')
        self.declare_parameter('table_reference_enabled', False)
        self.declare_parameter('table_x_m', 0.0)
        self.declare_parameter('table_y_m', 0.0)
        self.declare_parameter('table_yaw_deg', 0.0)
        self.declare_parameter('status_panel_topic', '/teleop/operator_status/compressed')
        self.declare_parameter('status_panel_jpeg_quality', 85)

        self.pos_threshold_m = float(self.get_parameter('pos_threshold_m').value)
        self.ori_threshold_deg = float(self.get_parameter('ori_threshold_deg').value)
        self.stale_timeout_s = float(self.get_parameter('stale_timeout_s').value)
        publish_hz = max(float(self.get_parameter('publish_hz').value), 0.1)
        self.record_practice_events = self._as_bool(
            self.get_parameter('record_practice_events').value)
        self.practice_event_log_path = os.path.expanduser(
            str(self.get_parameter('practice_event_log_path').value))
        self.table_reference_enabled = self._as_bool(
            self.get_parameter('table_reference_enabled').value)
        self.table_x_m = float(self.get_parameter('table_x_m').value)
        self.table_y_m = float(self.get_parameter('table_y_m').value)
        self.table_yaw_rad = math.radians(float(self.get_parameter('table_yaw_deg').value))
        self.status_panel_topic = str(self.get_parameter('status_panel_topic').value).strip()
        self.status_panel_jpeg_quality = int(min(
            max(int(self.get_parameter('status_panel_jpeg_quality').value), 1), 100))

        self.latest = {}
        self.latest_alignment_status = None
        self.latest_joint_state = None
        self.latest_odom = None
        self.latest_cmd_vel = None
        self.latest_mode = 'arm_control'
        self.latest_center_distance = {}
        self.latest_depth_metrics = {}
        self.last_event_warn_time = None
        self._subscribe_pose('right_goal', self.get_parameter('right_goal_topic').value)
        self._subscribe_pose('left_goal', self.get_parameter('left_goal_topic').value)
        self._subscribe_pose('right_current', self.get_parameter('right_current_topic').value)
        self._subscribe_pose('left_current', self.get_parameter('left_current_topic').value)

        self.status_pub = self.create_publisher(
            String, self.get_parameter('status_topic').value, 1)
        self.ok_pub = self.create_publisher(
            Bool, self.get_parameter('ok_topic').value, 1)
        self.practice_event_pub = self.create_publisher(
            String, self.get_parameter('practice_event_output_topic').value, 10)
        self.status_panel_pub = None
        if self.status_panel_topic:
            self.status_panel_pub = self.create_publisher(
                CompressedImage, self.status_panel_topic, 1)
        self._init_practice_event_inputs()
        self.timer = self.create_timer(1.0 / publish_hz, self._publish_status)

    def _subscribe_pose(self, key, topic):
        self.create_subscription(
            PoseStamped, topic, lambda msg, key=key: self._pose_callback(key, msg), 1)

    def _init_practice_event_inputs(self):
        if not self.record_practice_events:
            return
        self._subscribe_if_topic(
            String, self.get_parameter('practice_event_input_topic').value,
            self._manual_practice_event_callback)
        self._subscribe_if_topic(
            String, self.get_parameter('tact_trigger_topic').value,
            self._tact_trigger_callback)
        self._subscribe_if_topic(
            String, self.get_parameter('joystick_mode_topic').value,
            self._joystick_mode_callback)
        self._subscribe_if_topic(
            JointState, self.get_parameter('joint_state_topic').value,
            self._joint_state_callback)
        self._subscribe_if_topic(
            Odometry, self.get_parameter('odom_topic').value,
            self._odom_callback)
        self._subscribe_if_topic(
            Twist, self.get_parameter('cmd_vel_topic').value,
            self._cmd_vel_callback)
        self._subscribe_if_topic(
            Float32, self.get_parameter('right_center_distance_topic').value,
            lambda msg: self._center_distance_callback('right', msg))
        self._subscribe_if_topic(
            Float32, self.get_parameter('left_center_distance_topic').value,
            lambda msg: self._center_distance_callback('left', msg))
        self._subscribe_if_topic(
            String, self.get_parameter('right_depth_metrics_topic').value,
            lambda msg: self._depth_metrics_callback('right', msg))
        self._subscribe_if_topic(
            String, self.get_parameter('left_depth_metrics_topic').value,
            lambda msg: self._depth_metrics_callback('left', msg))

        self.get_logger().info(
            f'teleop practice event recording enabled: {self.practice_event_log_path}')

    def _subscribe_if_topic(self, msg_type, topic, callback):
        topic = str(topic).strip()
        if topic:
            self.create_subscription(msg_type, topic, callback, 10)

    def _as_bool(self, value):
        if isinstance(value, bool):
            return value
        if isinstance(value, str):
            return value.strip().lower() in ('1', 'true', 'yes', 'on')
        return bool(value)

    def _pose_callback(self, key, msg):
        self.latest[key] = {
            'msg': msg,
            'received_time': self.get_clock().now(),
        }

    def _manual_practice_event_callback(self, msg):
        event = msg.data.strip() or 'mark'
        self._record_practice_event(event, 'manual')

    def _tact_trigger_callback(self, msg):
        trigger = msg.data.strip()
        event_by_trigger = {
            'left': 'retry',
            'right': 'next',
            'left_long_time': 'good_pose',
            'right_long_time': 'success',
        }
        self._record_practice_event(
            event_by_trigger.get(trigger, trigger or 'tact'), f'tact:{trigger}')

    def _joystick_mode_callback(self, msg):
        if msg.data.strip():
            self.latest_mode = msg.data.strip()

    def _joint_state_callback(self, msg):
        self.latest_joint_state = msg

    def _odom_callback(self, msg):
        self.latest_odom = msg

    def _cmd_vel_callback(self, msg):
        self.latest_cmd_vel = msg

    def _center_distance_callback(self, side, msg):
        self.latest_center_distance[side] = float(msg.data)

    def _depth_metrics_callback(self, side, msg):
        try:
            self.latest_depth_metrics[side] = json.loads(msg.data)
        except json.JSONDecodeError:
            self._warn_event_throttled(f'failed to parse {side} depth metrics')

    def _publish_status(self):
        right = self._arm_status('right')
        left = self._arm_status('left')
        all_ok = bool(right['ok'] and left['ok'])

        payload = {
            'stamp_sec': self.get_clock().now().nanoseconds / 1e9,
            'mode': self.latest_mode,
            'thresholds': {
                'pos_m': self.pos_threshold_m,
                'ori_deg': self.ori_threshold_deg,
            },
            'right': right,
            'left': left,
            'all_ok': all_ok,
            'center_distance_m': self._center_distance_snapshot(),
            'depth_metrics': self._depth_metrics_snapshot(),
            'table_relative': self._table_relative_snapshot(),
        }
        self.latest_alignment_status = payload

        status_msg = String()
        status_msg.data = json.dumps(payload, sort_keys=True)
        self.status_pub.publish(status_msg)

        ok_msg = Bool()
        ok_msg.data = all_ok
        self.ok_pub.publish(ok_msg)
        self._publish_status_panel(payload)

    def _record_practice_event(self, event, source):
        if not self.record_practice_events:
            return
        payload = {
            'stamp_sec': self.get_clock().now().nanoseconds / 1e9,
            'event': event,
            'source': source,
            'mode': self.latest_mode,
            'joint_state': self._joint_state_snapshot(),
            'odom': self._odom_snapshot(),
            'cmd_vel': self._cmd_vel_snapshot(),
            'center_distance_m': self._center_distance_snapshot(),
            'depth_metrics': self._depth_metrics_snapshot(),
            'table_relative': self._table_relative_snapshot(),
            'alignment': self.latest_alignment_status,
        }
        data = json.dumps(payload, sort_keys=True)

        msg = String()
        msg.data = data
        self.practice_event_pub.publish(msg)

        if not self.practice_event_log_path:
            return
        try:
            parent = os.path.dirname(self.practice_event_log_path)
            if parent:
                os.makedirs(parent, exist_ok=True)
            with open(self.practice_event_log_path, 'a', encoding='utf-8') as stream:
                stream.write(data + '\n')
        except OSError as exc:
            self._warn_event_throttled(f'failed to write practice event log: {exc}')

    def _warn_event_throttled(self, message):
        now = self.get_clock().now()
        if (self.last_event_warn_time is None or
                (now - self.last_event_warn_time).nanoseconds > 5_000_000_000):
            self.get_logger().warn(message)
            self.last_event_warn_time = now

    def _publish_status_panel(self, payload):
        if (
            self.status_panel_pub is None or
            self.count_subscribers(self.status_panel_topic) <= 0
        ):
            return

        width = 560
        height = 196
        image = np.full((height, width, 3), (34, 36, 40), dtype=np.uint8)
        mode = str(payload.get('mode') or 'unknown').strip()
        mode_key = mode.lower()
        if mode_key == 'swerve':
            mode_label = 'BASE MOVE'
            header_color = (32, 113, 239)
        else:
            mode_label = 'ZED/HEAD + LIFT'
            header_color = (67, 161, 92)

        cv2.rectangle(image, (0, 0), (width, 58), header_color, -1)
        self._put_panel_text(
            image, f'MODE: {mode.upper()}', (16, 38), 0.86, (255, 255, 255), 2)
        self._put_panel_text(
            image, mode_label, (350, 38), 0.70, (255, 255, 255), 2)

        depth_metrics = payload.get('depth_metrics') or {}
        distances = payload.get('center_distance_m') or {}
        left_metric = depth_metrics.get('left') or {}
        right_metric = depth_metrics.get('right') or {}
        left_distance = self._format_distance(left_metric.get('center_m', distances.get('left')))
        right_distance = self._format_distance(right_metric.get('center_m', distances.get('right')))
        left_hint = str(left_metric.get('hint') or '--')
        right_hint = str(right_metric.get('hint') or '--')
        self._put_panel_text(
            image, f'L DEPTH: {left_distance} {left_hint}', (16, 91), 0.54, (236, 241, 245), 1)
        self._put_panel_text(
            image, f'R DEPTH: {right_distance} {right_hint}', (292, 91), 0.54, (236, 241, 245), 1)

        left_offset = self._format_offset(left_metric)
        right_offset = self._format_offset(right_metric)
        self._put_panel_text(
            image, f'L OFFSET: {left_offset}', (16, 125), 0.54, (236, 241, 245), 1)
        self._put_panel_text(
            image, f'R OFFSET: {right_offset}', (292, 125), 0.54, (236, 241, 245), 1)

        table = payload.get('table_relative')
        if table:
            table_text = (
                f'TABLE: {self._format_distance(table.get("distance_m"))} '
                f'HEAD ERR: {self._format_angle(table.get("heading_to_table_error_deg"))}'
            )
        else:
            table_text = 'TABLE: --'
        self._put_panel_text(image, table_text, (16, 162), 0.56, (203, 211, 218), 1)

        ok, encoded = cv2.imencode(
            '.jpg', image, [int(cv2.IMWRITE_JPEG_QUALITY), self.status_panel_jpeg_quality])
        if not ok:
            self._warn_event_throttled('failed to JPEG-encode operator status panel')
            return
        msg = CompressedImage()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'teleop_operator_status'
        msg.format = 'jpeg'
        msg.data = encoded.tobytes()
        self.status_panel_pub.publish(msg)

    def _put_panel_text(self, image, text, origin, scale, color, thickness):
        cv2.putText(
            image, text, origin, cv2.FONT_HERSHEY_SIMPLEX, scale,
            (0, 0, 0), thickness + 2, cv2.LINE_AA)
        cv2.putText(
            image, text, origin, cv2.FONT_HERSHEY_SIMPLEX, scale,
            color, thickness, cv2.LINE_AA)

    def _format_distance(self, value):
        if value is None:
            return '-- m'
        return f'{float(value):.2f} m'

    def _format_angle(self, value):
        if value is None:
            return '-- deg'
        return f'{float(value):+.1f} deg'

    def _format_offset(self, metrics):
        offset_x = metrics.get('offset_x_px')
        offset_y = metrics.get('offset_y_px')
        axis = metrics.get('axis_angle_deg')
        if offset_x is None or offset_y is None:
            return '--'
        if axis is None:
            return f'{int(offset_x):+d},{int(offset_y):+d}px'
        return f'{int(offset_x):+d},{int(offset_y):+d}px {float(axis):+.0f}deg'

    def _format_arm_status(self, status):
        if not status.get('available'):
            return '--'
        if not status.get('fresh'):
            return 'STALE'
        return 'OK' if status.get('ok') else 'CHECK'

    def _joint_state_snapshot(self):
        msg = self.latest_joint_state
        if msg is None:
            return None
        return {
            'stamp_sec': self._stamp_sec(msg.header.stamp),
            'name': list(msg.name),
            'position': self._float_list(msg.position),
            'velocity': self._float_list(msg.velocity),
            'effort': self._float_list(msg.effort),
        }

    def _odom_snapshot(self):
        msg = self.latest_odom
        if msg is None:
            return None
        pose = msg.pose.pose
        twist = msg.twist.twist
        yaw = self._yaw_from_quaternion(pose.orientation)
        return {
            'stamp_sec': self._stamp_sec(msg.header.stamp),
            'frame_id': msg.header.frame_id,
            'child_frame_id': msg.child_frame_id,
            'x_m': self._clean_float(pose.position.x),
            'y_m': self._clean_float(pose.position.y),
            'z_m': self._clean_float(pose.position.z),
            'yaw_deg': self._clean_float(math.degrees(yaw)),
            'linear_x_mps': self._clean_float(twist.linear.x),
            'linear_y_mps': self._clean_float(twist.linear.y),
            'angular_z_radps': self._clean_float(twist.angular.z),
        }

    def _cmd_vel_snapshot(self):
        msg = self.latest_cmd_vel
        if msg is None:
            return None
        return {
            'linear_x_mps': self._clean_float(msg.linear.x),
            'linear_y_mps': self._clean_float(msg.linear.y),
            'angular_z_radps': self._clean_float(msg.angular.z),
        }

    def _center_distance_snapshot(self):
        if not self.latest_center_distance:
            return None
        return {
            side: self._clean_float(distance)
            for side, distance in self.latest_center_distance.items()
        }

    def _depth_metrics_snapshot(self):
        if not self.latest_depth_metrics:
            return None
        return self.latest_depth_metrics.copy()

    def _table_relative_snapshot(self):
        msg = self.latest_odom
        if not self.table_reference_enabled or msg is None:
            return None
        pose = msg.pose.pose
        base_yaw = self._yaw_from_quaternion(pose.orientation)
        dx = self.table_x_m - pose.position.x
        dy = self.table_y_m - pose.position.y
        bearing = math.atan2(dy, dx)
        return {
            'table_x_m': self._clean_float(self.table_x_m),
            'table_y_m': self._clean_float(self.table_y_m),
            'table_yaw_deg': self._clean_float(math.degrees(self.table_yaw_rad)),
            'dx_m': self._clean_float(dx),
            'dy_m': self._clean_float(dy),
            'distance_m': self._clean_float(math.hypot(dx, dy)),
            'bearing_to_table_deg': self._clean_float(math.degrees(bearing)),
            'heading_to_table_error_deg': self._clean_float(
                math.degrees(self._normalize_angle(bearing - base_yaw))),
            'base_yaw_minus_table_yaw_deg': self._clean_float(
                math.degrees(self._normalize_angle(base_yaw - self.table_yaw_rad))),
        }

    def _stamp_sec(self, stamp):
        return self._clean_float(stamp.sec + stamp.nanosec / 1e9)

    def _float_list(self, values):
        return [self._clean_float(value) for value in values]

    def _clean_float(self, value):
        value = float(value)
        if not math.isfinite(value):
            return None
        return value

    def _yaw_from_quaternion(self, quat):
        siny_cosp = 2.0 * (quat.w * quat.z + quat.x * quat.y)
        cosy_cosp = 1.0 - 2.0 * (quat.y * quat.y + quat.z * quat.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def _normalize_angle(self, angle):
        return math.atan2(math.sin(angle), math.cos(angle))

    def _arm_status(self, arm):
        goal = self.latest.get(f'{arm}_goal')
        current = self.latest.get(f'{arm}_current')
        if goal is None or current is None:
            return {
                'available': False,
                'fresh': False,
                'ok': False,
                'pos_m': None,
                'ori_deg': None,
                'goal_frame': None,
                'current_frame': None,
            }

        now = self.get_clock().now()
        fresh = (
            (now - goal['received_time']).nanoseconds <= self.stale_timeout_s * 1e9 and
            (now - current['received_time']).nanoseconds <= self.stale_timeout_s * 1e9
        )
        goal_msg = goal['msg']
        current_msg = current['msg']
        pos_m = self._position_error(current_msg.pose, goal_msg.pose)
        ori_deg = self._orientation_error_deg(current_msg.pose, goal_msg.pose)
        ok = bool(
            fresh and
            pos_m <= self.pos_threshold_m and
            ori_deg <= self.ori_threshold_deg
        )
        return {
            'available': True,
            'fresh': bool(fresh),
            'ok': ok,
            'pos_m': round(pos_m, 4),
            'ori_deg': round(ori_deg, 2),
            'goal_frame': goal_msg.header.frame_id,
            'current_frame': current_msg.header.frame_id,
        }

    def _position_error(self, pose_a, pose_b):
        dx = pose_a.position.x - pose_b.position.x
        dy = pose_a.position.y - pose_b.position.y
        dz = pose_a.position.z - pose_b.position.z
        return math.sqrt(dx * dx + dy * dy + dz * dz)

    def _orientation_error_deg(self, pose_a, pose_b):
        qa = pose_a.orientation
        qb = pose_b.orientation
        dot = qa.x * qb.x + qa.y * qb.y + qa.z * qb.z + qa.w * qb.w
        norm_a = math.sqrt(qa.x * qa.x + qa.y * qa.y + qa.z * qa.z + qa.w * qa.w)
        norm_b = math.sqrt(qb.x * qb.x + qb.y * qb.y + qb.z * qb.z + qb.w * qb.w)
        if norm_a <= 1e-9 or norm_b <= 1e-9:
            return 180.0
        dot = abs(dot / (norm_a * norm_b))
        dot = min(max(dot, 0.0), 1.0)
        return math.degrees(2.0 * math.acos(dot))


def main(args=None):
    rclpy.init(args=args)
    node = AlignmentStatus()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
