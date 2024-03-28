#! /usr/bin/python3

import rclpy
from rclpy.node import Node
from builtin_interfaces.msg import Time as MsgTime
import time

class ClockNode(Node):
    def __init__(self) -> None:
        super().__init__("battery_cell_clock_node", parameter_overrides=[])

        self.clock_pub_ = self.create_publisher(MsgTime, "/clock", 1)

        self.timer = self.create_timer(0.0002, self.timer_callback)

        self.msg = MsgTime()

    def timer_callback(self) -> None:
        self.clock_pub_.publish(self.get_clock().now().to_msg())

if __name__ == '__main__':
    rclpy.init()

    node = ClockNode()

    rclpy.spin(node)

    rclpy.shutdown()
