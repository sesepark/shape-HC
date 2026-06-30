import copy
import json
import math

from control_msgs.msg import DynamicJointState
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory


YM080_CURRENT_SCALE = 0.07692307692
DXL_CURRENT_INTERFACE = 'Present Current'
PROTECTED_JOINTS = {
    'left': (
        {'joint': 'arm_l_joint2', 'dxl': 'dxl32', 'label': 'L J2'},
        {'joint': 'arm_l_joint3', 'dxl': 'dxl33', 'label': 'L J3'},
    ),
    'right': (
        {'joint': 'arm_r_joint2', 'dxl': 'dxl2', 'label': 'R J2'},
        {'joint': 'arm_r_joint3', 'dxl': 'dxl3', 'label': 'R J3'},
    ),
}
PROTECTED_DXL_NAMES = {
    item['dxl']
    for side_items in PROTECTED_JOINTS.values()
    for item in side_items
}


class MissionBGraspGuard(Node):

    def __init__(self):
        super().__init__('teleop_mission_b_grasp_guard')

        self.declare_parameter(
            'left_input_topic',
            '/leader/joint_trajectory_command_broadcaster_left/joint_trajectory')
        self.declare_parameter(
            'right_input_topic',
            '/leader/joint_trajectory_command_broadcaster_right/joint_trajectory')
        self.declare_parameter(
            'left_output_topic',
            '/teleop/mission_b_grasp_guard/left/joint_trajectory')
        self.declare_parameter(
            'right_output_topic',
            '/teleop/mission_b_grasp_guard/right/joint_trajectory')
        self.declare_parameter(
            'enabled_topic',
            '/teleop/mission_b_grasp_guard/enabled')
        self.declare_parameter(
            'status_topic',
            '/teleop/mission_b_grasp_guard/status')
        self.declare_parameter('dynamic_joint_state_topic', '/dynamic_joint_states')
        self.declare_parameter('joint_state_topic', '/joint_states')
        self.declare_parameter('threshold_raw', 1950.0)
        self.declare_parameter('release_raw', 1700.0)
        self.declare_parameter('stale_timeout_s', 0.5)
        self.declare_parameter('direction_deadband_rad', 0.002)
        self.declare_parameter('status_publish_hz', 5.0)

        self.left_input_topic = str(self.get_parameter('left_input_topic').value).strip()
        self.right_input_topic = str(self.get_parameter('right_input_topic').value).strip()
        self.left_output_topic = str(self.get_parameter('left_output_topic').value).strip()
        self.right_output_topic = str(self.get_parameter('right_output_topic').value).strip()
        self.enabled_topic = str(self.get_parameter('enabled_topic').value).strip()
        self.status_topic = str(self.get_parameter('status_topic').value).strip()
        self.dynamic_joint_state_topic = str(
            self.get_parameter('dynamic_joint_state_topic').value).strip()
        self.joint_state_topic = str(self.get_parameter('joint_state_topic').value).strip()
        self.threshold_raw = abs(float(self.get_parameter('threshold_raw').value))
        self.release_raw = abs(float(self.get_parameter('release_raw').value))
        if self.release_raw > self.threshold_raw:
            self.release_raw = self.threshold_raw
        self.stale_timeout_s = max(float(self.get_parameter('stale_timeout_s').value), 0.05)
        self.direction_deadband_rad = max(
            float(self.get_parameter('direction_deadband_rad').value), 0.0)
        status_publish_hz = max(float(self.get_parameter('status_publish_hz').value), 0.5)

        self.enabled = False
        self.current_raw_by_dxl = {}
        self.error_code_by_dxl = {}
        self.hardware_error_by_dxl = {}
        self.current_update_time = None
        self.joint_positions = {}
        self.joint_state_update_time = None
        self.latches = {
            item['joint']: {
                'active': False,
                'armed': False,
                'position': None,
                'direction': 0.0,
                'direction_source': '',
                'raw_current': None,
            }
            for side_items in PROTECTED_JOINTS.values()
            for item in side_items
        }
        self.last_status = {
            'reason': 'startup',
            'frozen': [],
            'warnings': [],
            'pass_through': True,
        }

        enabled_qos = QoSProfile(depth=1)
        enabled_qos.reliability = ReliabilityPolicy.RELIABLE
        enabled_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        status_qos = QoSProfile(depth=1)
        status_qos.reliability = ReliabilityPolicy.RELIABLE
        status_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        self.left_pub = self.create_publisher(
            JointTrajectory, self.left_output_topic, 10)
        self.right_pub = self.create_publisher(
            JointTrajectory, self.right_output_topic, 10)
        self.status_pub = self.create_publisher(String, self.status_topic, status_qos)

        self.create_subscription(
            Bool, self.enabled_topic, self._enabled_callback, enabled_qos)
        self.create_subscription(
            DynamicJointState, self.dynamic_joint_state_topic,
            self._dynamic_joint_state_callback, 10)
        self.create_subscription(
            JointState, self.joint_state_topic, self._joint_state_callback, 10)
        self.create_subscription(
            JointTrajectory, self.left_input_topic,
            lambda msg: self._trajectory_callback('left', msg), 10)
        self.create_subscription(
            JointTrajectory, self.right_input_topic,
            lambda msg: self._trajectory_callback('right', msg), 10)
        self.status_timer = self.create_timer(
            1.0 / status_publish_hz, self._publish_status)

        self._publish_status()
        self.get_logger().info(
            'Mission B grasp guard ready: '
            f'enabled_topic={self.enabled_topic}, threshold_raw={self.threshold_raw:.0f}, '
            f'release_raw={self.release_raw:.0f}')

    def _enabled_callback(self, msg):
        self.enabled = bool(msg.data)
        if not self.enabled:
            self._clear_latches()
        self.last_status = {
            'reason': 'enabled' if self.enabled else 'disabled',
            'frozen': [],
            'warnings': [],
            'pass_through': not self.enabled,
        }
        self._publish_status()

    def _dynamic_joint_state_callback(self, msg):
        latest_currents = {}
        latest_error = {}
        latest_hardware_error = {}
        for index, joint_name in enumerate(msg.joint_names):
            if joint_name not in PROTECTED_DXL_NAMES or index >= len(msg.interface_values):
                continue
            current_raw = None
            error_code = None
            hardware_error = None
            interface_values = msg.interface_values[index]
            for interface_name, value in zip(
                interface_values.interface_names,
                interface_values.values
            ):
                cleaned_value = self._safe_float(value)
                if cleaned_value is None:
                    continue
                if interface_name == DXL_CURRENT_INTERFACE:
                    current_raw = int(round(cleaned_value / YM080_CURRENT_SCALE))
                elif interface_name == 'Error Code':
                    error_code = self._nonzero_int(cleaned_value)
                elif interface_name == 'Hardware Error Status':
                    hardware_error = self._nonzero_int(cleaned_value)
            if current_raw is not None:
                latest_currents[joint_name] = current_raw
            if error_code is not None:
                latest_error[joint_name] = error_code
            if hardware_error is not None:
                latest_hardware_error[joint_name] = hardware_error

        if latest_currents or latest_error or latest_hardware_error:
            self.current_raw_by_dxl.update(latest_currents)
            self.error_code_by_dxl = latest_error
            self.hardware_error_by_dxl = latest_hardware_error
            self.current_update_time = self.get_clock().now()

    def _joint_state_callback(self, msg):
        positions = {}
        for index, joint_name in enumerate(msg.name):
            if index < len(msg.position):
                value = self._safe_float(msg.position[index])
                if value is not None:
                    positions[joint_name] = value
        if positions:
            self.joint_positions.update(positions)
            self.joint_state_update_time = self.get_clock().now()

    def _trajectory_callback(self, side, msg):
        publisher = self.left_pub if side == 'left' else self.right_pub
        if not self.enabled:
            publisher.publish(msg)
            self.last_status = {
                'side': side,
                'reason': 'disabled',
                'frozen': [],
                'warnings': [],
                'pass_through': True,
            }
            return

        fresh, stale_reasons = self._inputs_fresh()
        if not fresh:
            publisher.publish(msg)
            self.last_status = {
                'side': side,
                'reason': 'stale_inputs',
                'frozen': [],
                'warnings': stale_reasons,
                'pass_through': True,
            }
            return

        guarded_msg = copy.deepcopy(msg)
        frozen = []
        warnings = []
        for item in PROTECTED_JOINTS.get(side, ()):
            joint_name = item['joint']
            if joint_name not in guarded_msg.joint_names:
                continue
            joint_index = guarded_msg.joint_names.index(joint_name)
            result = self._guard_joint(guarded_msg, joint_index, item)
            frozen.extend(result['frozen'])
            warnings.extend(result['warnings'])

        publisher.publish(guarded_msg)
        self.last_status = {
            'side': side,
            'reason': 'clamped' if frozen else 'pass_through',
            'frozen': frozen,
            'warnings': warnings,
            'pass_through': not frozen,
        }

    def _guard_joint(self, msg, joint_index, item):
        joint_name = item['joint']
        dxl_name = item['dxl']
        label = item['label']
        result = {
            'frozen': [],
            'warnings': [],
        }
        current_raw = self.current_raw_by_dxl.get(dxl_name)
        current_position = self.joint_positions.get(joint_name)
        if current_raw is None:
            result['warnings'].append(f'{label}:missing_current')
            return result
        if current_position is None:
            result['warnings'].append(f'{label}:missing_joint_state')
            return result

        latch = self.latches[joint_name]
        abs_current = abs(float(current_raw))
        if abs_current <= self.release_raw:
            if latch['active']:
                self._clear_latch(joint_name)
            latch['armed'] = True
            return result
        if abs_current < self.threshold_raw:
            if not latch['active']:
                latch['armed'] = True
            return result

        if abs_current >= self.threshold_raw and not latch['active']:
            if not latch.get('armed'):
                result['warnings'].append(f'{label}:not_armed_high_current')
                return result
            direction = self._direction_from_target(msg, joint_index, current_position)
            if direction == 0.0:
                result['warnings'].append(f'{label}:uncertain_direction')
                return result
            latch['active'] = True
            latch['armed'] = False
            latch['position'] = float(current_position)
            latch['direction'] = direction
            latch['direction_source'] = 'target_delta_at_threshold'
            latch['raw_current'] = int(current_raw)

        if not latch['active']:
            return result

        latch_position = latch.get('position')
        if latch_position is None:
            result['warnings'].append(f'{label}:missing_latch_position')
            return result

        clamped_any = False
        for point in msg.points:
            if joint_index >= len(point.positions):
                continue
            target_position = self._safe_float(point.positions[joint_index])
            if target_position is None:
                continue
            direction = self._latch_direction(latch)
            if direction == 0.0:
                result['warnings'].append(f'{label}:uncertain_direction')
                continue
            if (target_position - float(latch_position)) * direction <= self.direction_deadband_rad:
                continue

            point.positions[joint_index] = float(latch_position)
            if joint_index < len(point.velocities):
                point.velocities[joint_index] = 0.0
            if joint_index < len(point.accelerations):
                point.accelerations[joint_index] = 0.0
            clamped_any = True

        if clamped_any:
            result['frozen'].append({
                'joint': joint_name,
                'dxl': dxl_name,
                'label': label,
                'current_raw': int(current_raw),
                'latch_position': self._clean_float(latch_position),
                'direction': latch.get('direction'),
            })
        return result

    def _latch_direction(self, latch):
        direction = float(latch.get('direction') or 0.0)
        if direction < 0.0:
            return -1.0
        if direction > 0.0:
            return 1.0
        return 0.0

    def _direction_from_target(self, msg, joint_index, current_position):
        for point in reversed(msg.points):
            if joint_index >= len(point.positions):
                continue
            target_position = self._safe_float(point.positions[joint_index])
            if target_position is None:
                continue
            delta = target_position - float(current_position)
            if abs(delta) <= self.direction_deadband_rad:
                continue
            return 1.0 if delta > 0.0 else -1.0
        return 0.0

    def _inputs_fresh(self):
        now = self.get_clock().now()
        reasons = []
        if self.current_update_time is None:
            reasons.append('current_missing')
        else:
            age_s = (now - self.current_update_time).nanoseconds / 1e9
            if age_s > self.stale_timeout_s:
                reasons.append(f'current_stale:{age_s:.2f}s')

        if self.joint_state_update_time is None:
            reasons.append('joint_state_missing')
        else:
            age_s = (now - self.joint_state_update_time).nanoseconds / 1e9
            if age_s > self.stale_timeout_s:
                reasons.append(f'joint_state_stale:{age_s:.2f}s')
        return not reasons, reasons

    def _clear_latches(self):
        for joint_name in self.latches:
            self._clear_latch(joint_name)

    def _clear_latch(self, joint_name):
        latch = self.latches[joint_name]
        latch['active'] = False
        latch['armed'] = False
        latch['position'] = None
        latch['direction'] = 0.0
        latch['direction_source'] = ''
        latch['raw_current'] = None

    def _publish_status(self):
        msg = String()
        msg.data = json.dumps(self._status_payload(), sort_keys=True)
        self.status_pub.publish(msg)

    def _status_payload(self):
        now = self.get_clock().now()
        current_age_s = None
        joint_state_age_s = None
        if self.current_update_time is not None:
            current_age_s = (now - self.current_update_time).nanoseconds / 1e9
        if self.joint_state_update_time is not None:
            joint_state_age_s = (now - self.joint_state_update_time).nanoseconds / 1e9
        return {
            'stamp_sec': self._clean_float(now.nanoseconds / 1e9),
            'enabled': bool(self.enabled),
            'threshold_raw': self._clean_float(self.threshold_raw),
            'release_raw': self._clean_float(self.release_raw),
            'stale_timeout_s': self._clean_float(self.stale_timeout_s),
            'current_age_s': self._clean_float(current_age_s),
            'joint_state_age_s': self._clean_float(joint_state_age_s),
            'currents_raw': self.current_raw_by_dxl.copy(),
            'error_codes': self.error_code_by_dxl.copy(),
            'hardware_error_status': self.hardware_error_by_dxl.copy(),
            'latches': self._latch_snapshot(),
            'last': self.last_status.copy(),
        }

    def _latch_snapshot(self):
        snapshot = {}
        for joint_name, latch in self.latches.items():
            snapshot[joint_name] = {
                'active': bool(latch.get('active')),
                'armed': bool(latch.get('armed')),
                'position': self._clean_float(latch.get('position')),
                'direction': self._clean_float(latch.get('direction')),
                'direction_source': latch.get('direction_source') or '',
                'raw_current': latch.get('raw_current'),
            }
        return snapshot

    def _safe_float(self, value):
        try:
            value = float(value)
        except (TypeError, ValueError):
            return None
        if not math.isfinite(value):
            return None
        return value

    def _clean_float(self, value):
        if value is None:
            return None
        return self._safe_float(value)

    def _nonzero_int(self, value):
        value = int(round(float(value)))
        return value if value != 0 else None


def main(args=None):
    rclpy.init(args=args)
    node = MissionBGraspGuard()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
