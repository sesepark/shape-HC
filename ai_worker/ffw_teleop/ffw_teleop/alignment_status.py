import json
import math

from geometry_msgs.msg import PoseStamped
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
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

        self.pos_threshold_m = float(self.get_parameter('pos_threshold_m').value)
        self.ori_threshold_deg = float(self.get_parameter('ori_threshold_deg').value)
        self.stale_timeout_s = float(self.get_parameter('stale_timeout_s').value)
        publish_hz = max(float(self.get_parameter('publish_hz').value), 0.1)

        self.latest = {}
        self._subscribe_pose('right_goal', self.get_parameter('right_goal_topic').value)
        self._subscribe_pose('left_goal', self.get_parameter('left_goal_topic').value)
        self._subscribe_pose('right_current', self.get_parameter('right_current_topic').value)
        self._subscribe_pose('left_current', self.get_parameter('left_current_topic').value)

        self.status_pub = self.create_publisher(
            String, self.get_parameter('status_topic').value, 1)
        self.ok_pub = self.create_publisher(
            Bool, self.get_parameter('ok_topic').value, 1)
        self.timer = self.create_timer(1.0 / publish_hz, self._publish_status)

    def _subscribe_pose(self, key, topic):
        self.create_subscription(
            PoseStamped, topic, lambda msg, key=key: self._pose_callback(key, msg), 1)

    def _pose_callback(self, key, msg):
        self.latest[key] = {
            'msg': msg,
            'received_time': self.get_clock().now(),
        }

    def _publish_status(self):
        right = self._arm_status('right')
        left = self._arm_status('left')
        all_ok = bool(right['ok'] and left['ok'])

        payload = {
            'stamp_sec': self.get_clock().now().nanoseconds / 1e9,
            'thresholds': {
                'pos_m': self.pos_threshold_m,
                'ori_deg': self.ori_threshold_deg,
            },
            'right': right,
            'left': left,
            'all_ok': all_ok,
        }

        status_msg = String()
        status_msg.data = json.dumps(payload, sort_keys=True)
        self.status_pub.publish(status_msg)

        ok_msg = Bool()
        ok_msg.data = all_ok
        self.ok_pub.publish(ok_msg)

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
