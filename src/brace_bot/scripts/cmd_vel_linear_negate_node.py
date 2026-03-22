#!/usr/bin/env python3
"""订阅 /cmd_vel，原样转发到 /cmd_vel_to_base，供底盘控制器使用。Nav2 的“前进”(linear.x>0) 与底盘“前进”一致，不取负。"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class CmdVelLinearNegateNode(Node):
    def __init__(self):
        super().__init__("cmd_vel_linear_negate")
        self.sub = self.create_subscription(Twist, "/cmd_vel", self.cb, 10)
        self.pub = self.create_publisher(Twist, "/cmd_vel_to_base", 10)

    def cb(self, msg: Twist):
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = CmdVelLinearNegateNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
