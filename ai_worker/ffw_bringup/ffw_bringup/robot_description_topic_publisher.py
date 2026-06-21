#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String


class RobotDescriptionTopicPublisher(Node):

    def __init__(self):
        super().__init__('robot_description_topic_publisher')
        self.declare_parameter('robot_description', '')
        self.declare_parameter('publish_period_sec', 1.0)

        description = self.get_parameter('robot_description').value
        publish_period_sec = self.get_parameter('publish_period_sec').value

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.publisher = self.create_publisher(String, 'robot_description', qos)
        self.message = String(data=description)

        self.timer = self.create_timer(float(publish_period_sec), self.publish_description)
        self.publish_description()
        self.get_logger().info('Publishing robot_description topic')

    def publish_description(self):
        self.publisher.publish(self.message)


def main(args=None):
    rclpy.init(args=args)
    node = RobotDescriptionTopicPublisher()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
