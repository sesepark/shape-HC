import json

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


RIGHT_LONG_TIME_TRIGGER = 'right_long_time'


class LeaderTactOkBridge(Node):

    def __init__(self):
        super().__init__('leader_tact_ok_bridge')

        self.declare_parameter(
            'tact_trigger_topic', '/leader/joystick_controller/tact_trigger')
        self.declare_parameter('operator_ok_topic', '/teleop/operator_ok')
        self.declare_parameter('ok_overlay_duration_s', 3.0)

        self.tact_trigger_topic = str(
            self.get_parameter('tact_trigger_topic').value).strip()
        self.operator_ok_topic = str(
            self.get_parameter('operator_ok_topic').value).strip()
        self.ok_overlay_duration_s = max(
            float(self.get_parameter('ok_overlay_duration_s').value), 0.1)

        self.ok_pub = self.create_publisher(String, self.operator_ok_topic, 10)
        self.create_subscription(
            String, self.tact_trigger_topic, self._tact_trigger_callback, 10)

        self.get_logger().info(
            f'leader tact OK bridge active: {self.tact_trigger_topic} '
            f'right_long_time -> {self.operator_ok_topic}')

    def _tact_trigger_callback(self, msg):
        trigger = str(msg.data or '').strip()
        if trigger != RIGHT_LONG_TIME_TRIGGER:
            return

        payload = {
            'event': 'ok',
            'source': 'leader_right_long_time',
            'trigger': trigger,
            'duration_s': self.ok_overlay_duration_s,
        }
        out = String()
        out.data = json.dumps(payload, sort_keys=True)
        self.ok_pub.publish(out)
        self.get_logger().info('OK sign triggered by leader right long press')


def main(args=None):
    rclpy.init(args=args)
    node = LeaderTactOkBridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
