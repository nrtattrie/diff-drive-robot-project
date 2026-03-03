#!/usr/bin/env python3
"""
Relay: geometry_msgs/Twist  →  geometry_msgs/TwistStamped

diff_drive_controller in ROS 2 Jazzy requires TwistStamped, but
teleop_twist_keyboard publishes plain Twist.  This node bridges the gap.

  /cmd_vel  (Twist)  →  /diffbot_base_controller/cmd_vel  (TwistStamped)
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped


class TwistRelay(Node):
    def __init__(self):
        super().__init__('twist_relay')
        self.pub = self.create_publisher(
            TwistStamped, '/diffbot_base_controller/cmd_vel', 10)
        self.sub = self.create_subscription(
            Twist, '/cmd_vel', self.callback, 10)
        self.get_logger().info(
            'twist_relay ready: /cmd_vel (Twist) → /diffbot_base_controller/cmd_vel (TwistStamped)')

    def callback(self, msg: Twist):
        stamped = TwistStamped()
        stamped.header.stamp = self.get_clock().now().to_msg()
        stamped.twist = msg
        self.pub.publish(stamped)


def main():
    rclpy.init()
    node = TwistRelay()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
