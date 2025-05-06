#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from std_msgs.msg import Int16

class TestTimer(Node):
    def __init__(self):
        super().__init__("test_timer")
        self.get_logger().info("Test Timer Node has been started")
        self.test_period = self.declare_parameter("test_period", 90.0)

        self.timer_pub = self.create_publisher(Int16, "test_timer", 10)

        self.timer = self.create_timer(float(self.test_period.value), self.timer_callback)
        self.count = 0

    def timer_callback(self):
        self.get_logger().info(f"Test time is over!")
        msg = Int16()
        msg.data = 1
        self.timer_pub.publish(msg)

def main():
    rclpy.init()
    node = TestTimer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()