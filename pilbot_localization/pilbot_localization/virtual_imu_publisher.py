#!/usr/bin/env python3
import rclpy
from threading import Lock
from rclpy.node import Node
from sensor_msgs.msg import Imu


class VirtualIMUPublisher(Node):
    def __init__(self):
        super().__init__("virtual_imu_publisher")
        
        self.virtual_imu = self.create_publisher(Imu, "/virtual_imu", 10)

        self.create_subscription(Imu, "/imu1", lambda msg: self.imu_callback("imu1", msg), 10)
        self.create_subscription(Imu, "/imu2", lambda msg: self.imu_callback("imu2", msg), 10)
        self.create_subscription(Imu, "/imu3", lambda msg: self.imu_callback("imu3", msg), 10)
    
        self.buffer = {}
        self.lock = Lock()

    def imu_callback(self, imu_id, msg):
        timestamp = self.get_timestamp(msg.header)

        if self.buffer.get(timestamp) is None:
            self.buffer[timestamp] = {}
        
        with self.lock:
            self.buffer[timestamp][imu_id] = msg

        if len(self.buffer[timestamp]) == 3:
            self.combine_and_publish(timestamp)
            with self.lock:
                del self.buffer[timestamp]

    def combine_and_publish(self, timestamp):
        data = self.buffer[timestamp]
        imu1 = data["imu1"]
        imu2 = data["imu2"]
        imu3 = data["imu3"]

        # Combine the IMU data from imu1, imu2, and imu3
        combined_msg = Imu()

        combined_msg.header = imu1.header
        combined_msg.header.frame_id = "base_footprint_ekf"
        combined_msg.orientation.x = (imu1.orientation.x + imu2.orientation.x + imu3.orientation.x) / 3.0
        combined_msg.orientation.y = (imu1.orientation.y + imu2.orientation.y + imu3.orientation.y) / 3.0
        combined_msg.orientation.z = (imu1.orientation.z + imu2.orientation.z + imu3.orientation.z) / 3.0
        combined_msg.orientation.w = (imu1.orientation.w + imu2.orientation.w + imu3.orientation.w) / 3.0

        norm = (combined_msg.orientation.x ** 2 + combined_msg.orientation.y ** 2 + combined_msg.orientation.z ** 2 + combined_msg.orientation.w ** 2) ** 0.5 # Vektörün büyüklüğü

        if norm > 1e-9:
            combined_msg.orientation.x /= norm
            combined_msg.orientation.y /= norm
            combined_msg.orientation.z /= norm
            combined_msg.orientation.w /= norm
        else:
            combined_msg.orientation.x = 0.0
            combined_msg.orientation.y = 0.0
            combined_msg.orientation.z = 0.0
            combined_msg.orientation.w = 1.0

        combined_msg.angular_velocity.x = (imu1.angular_velocity.x + imu2.angular_velocity.x + imu3.angular_velocity.x) / 3.0
        combined_msg.angular_velocity.y = (imu1.angular_velocity.y + imu2.angular_velocity.y + imu3.angular_velocity.y) / 3.0
        combined_msg.angular_velocity.z = (imu1.angular_velocity.z + imu2.angular_velocity.z + imu3.angular_velocity.z) / 3.0
        
        combined_msg.linear_acceleration.x = (imu1.linear_acceleration.x + imu2.linear_acceleration.x + imu3.linear_acceleration.x) / 3.0
        combined_msg.linear_acceleration.y = (imu1.linear_acceleration.y + imu2.linear_acceleration.y + imu3.linear_acceleration.y) / 3.0
        combined_msg.linear_acceleration.z = (imu1.linear_acceleration.z + imu2.linear_acceleration.z + imu3.linear_acceleration.z) / 3.0

        self.virtual_imu.publish(combined_msg)


    def get_timestamp(self, header):
        return float(header.stamp.sec + header.stamp.nanosec * 1e-9)

def main(args=None):
    rclpy.init()
    node = VirtualIMUPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()