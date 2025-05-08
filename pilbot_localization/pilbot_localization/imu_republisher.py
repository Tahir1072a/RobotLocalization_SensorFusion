#!/usr/bin/env python3
import rclpy
import time
from rclpy.node import Node
from sensor_msgs.msg import Imu

imu_pub_1 = None
imu_pub_2 = None
imu_pub_3 = None

def imu_callback(msg):
    global imu_pub_1
    global imu_pub_2
    global imu_pub_3

    old_frame_id = msg.header.frame_id
    msg.header.frame_id = "base_footprint_ekf"

    if old_frame_id == "imu1_link":
        imu_pub_1.publish(msg)
    elif old_frame_id == "imu2_link":
        imu_pub_2.publish(msg)
    elif old_frame_id == "imu3_link":
        imu_pub_3.publish(msg)

def main():
    global imu_pub_1
    global imu_pub_2
    global imu_pub_3
    rclpy.init()
    node = Node("imu_republisher_node")
    time.sleep(1)

    imu_pub_1 = node.create_publisher(Imu, "imu_ekf_1", 10)
    node.create_subscription(Imu, "/imu1", imu_callback ,10)
    imu_pub_2 = node.create_publisher(Imu, "imu_ekf_2", 10)
    node.create_subscription(Imu, "/imu2", imu_callback ,10)
    imu_pub_3 = node.create_publisher(Imu, "imu_ekf_3", 10)
    node.create_subscription(Imu, "/imu3", imu_callback ,10)

    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()