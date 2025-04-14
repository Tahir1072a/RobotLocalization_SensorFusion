#!/usr/bin/env python3
import rclpy
import time
from rclpy.node import Node
from sensor_msgs.msg import Imu

imu_pub_1 = None

def imu_callback(msg):
    global imu_pub_1
    msg.header.frame_id = "base_footprint_ekf"
    imu_pub_1.publish(msg)

def main():
    global imu_pub_1
    rclpy.init()
    node = Node("imu_republisher_node")
    time.sleep(1)

    imu_pub_1 = node.create_publisher(Imu, "imu_ekf_1", 10)
    imu_sub = node.create_subscription(Imu, "/imu1", imu_callback ,10)
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()