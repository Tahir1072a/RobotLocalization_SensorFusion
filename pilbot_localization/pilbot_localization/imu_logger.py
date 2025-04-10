#!/usr/bin/env python3
import rclpy
import pandas as pd
from rclpy.node import Node
from nav_msgs.msg import Odometry

from sensor_msgs.msg import Imu
from threading import Lock

class ImuLogger(Node):
    def __init__(self):
        super().__init__("imu_logger")

        self.imu_sub = self.create_subscription(Imu, "imu1", lambda msg: self.imu_callback(msg=msg, imu_id="imu1"), 10)
        self.imu_sub = self.create_subscription(Imu, "imu2", lambda msg: self.imu_callback(msg=msg, imu_id="imu2"), 10)
        self.imu_sub = self.create_subscription(Imu, "imu3", lambda msg: self.imu_callback(msg=msg, imu_id="imu3"), 10)

        self.pilbot_pose_sub = self.create_subscription(Odometry, "pilbot/real_pose", self.pose_callback, 10)

        self.df = pd.DataFrame(columns=["time"])

        self.buffer = {}
        self.buffer_lock = Lock()

    # pozisyon mesajı birleştirilecek
    def pose_callback(self, msg):
        self.get_logger().info(str(msg.pose.pose.position))

    def imu_callback(self, imu_id, msg):
        timestamp = self.get_timestamp(msg.header)
        angular_velocity = msg.angular_velocity
        linear_accleration = msg.linear_acceleration

        if self.buffer.get(timestamp) is None:
            self.buffer[str(timestamp)] = {}

        with self.buffer_lock:
            self.buffer[timestamp][imu_id] = {
                "ax": linear_accleration.x,
                "ay": linear_accleration.y,
                "az": linear_accleration.z,
                "gx": angular_velocity.x,
                "gy": angular_velocity.y,
                "gz": angular_velocity.z
            }

        if len(self.buffer[timestamp]) == 3:
            self.prepare_data(timestamp)
    
    def get_timestamp(self, header):
        timestamp_sec = header.stamp.sec
        timestamp_nano = header.stamp.nanosec
        return f"{timestamp_sec}.{timestamp_nano:09d}"

    def prepare_data(self, timestamp):
        data = self.buffer[timestamp]

        row = {
            "time": timestamp
        }

        for imu_id in ["imu1", "imu2", "imu3"]:
            imu_data = data[imu_id]
            row.update({
                f"ax{imu_id[-1]}": imu_data["ax"],
                f"ay{imu_id[-1]}": imu_data["ay"],
                f"az{imu_id[-1]}": imu_data["az"],
                f"gx{imu_id[-1]}": imu_data["gx"],
                f"gy{imu_id[-1]}": imu_data["gy"],
                f"gz{imu_id[-1]}": imu_data["gz"],
            })
        
        with self.buffer_lock:
            self.df = pd.concat([self.df, pd.DataFrame([row])], ignore_index=True)
            del self.buffer[timestamp]
        
        # Burası daha dinamik hale getirilmelidir!!
        if len(self.df) == 300:
            with self.buffer_lock:
                self.df.to_excel("imu_saved_data.xlsx", index=False)
                self.df.drop(self.df.index)

def main():
    rclpy.init()
    node = ImuLogger()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()