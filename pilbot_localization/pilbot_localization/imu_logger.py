#!/usr/bin/env python3
import rclpy
import pandas as pd
from rclpy.node import Node
from nav_msgs.msg import Odometry
from pilbot_msgs.msg import MovementCycles
from sensor_msgs.msg import Imu
from threading import Lock

# Estimate DEğerleri eklenecek. Ayrıca bir tane launch file yazılıp tüm launc file'lar aynı anda başlaması gerekiyor.
# Not: Imu tanımları daha dinamik hale gitrilebilir, urdf dosyalarına bakmayı unutma xacro ifadeler kullanabilirsin. (refactor önerisi)
class ImuLogger(Node):
    def __init__(self):
        super().__init__("imu_logger")

        self.imu_sub = self.create_subscription(Imu, "imu1", lambda msg: self.imu_callback(msg=msg, imu_id="imu1"), 10)
        self.imu_sub = self.create_subscription(Imu, "imu2", lambda msg: self.imu_callback(msg=msg, imu_id="imu2"), 10)
        self.imu_sub = self.create_subscription(Imu, "imu3", lambda msg: self.imu_callback(msg=msg, imu_id="imu3"), 10)

        self.pilbot_pose_sub = self.create_subscription(Odometry, "pilbot/real_pose", self.pose_callback, 10)
        self.movement_cycle_sub = self.create_subscription(MovementCycles, "movement_cycles", self.cycles_callback, 10)

        self.df = pd.DataFrame(columns=["time"])

        self.buffer = {}
        self.buffer_lock = Lock()

    def pose_callback(self, msg):
        timestamp = self.get_timestamp(msg.header)
        position =  msg.pose.pose.position
        
        if self.buffer.get(timestamp) is None:
            self.buffer[str(timestamp)] = {}

        with self.buffer_lock:
            self.buffer[timestamp]["real_poses"] = {
                "pose_x": position.x,
                "pose_y": position.y,
                "pose_z": position.z
            }
        
        if len(self.buffer[timestamp]) == 4:
            self.prepare_data(timestamp)


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
        
        if len(self.buffer[timestamp]) == 4:
            self.prepare_data(timestamp)
        
    
    def get_timestamp(self, header):
        timestamp_sec = header.stamp.sec
        timestamp_nano = header.stamp.nanosec
        return f"{timestamp_sec}.{timestamp_nano:09d}"

    def prepare_data(self, timestamp):
        data = self.buffer[timestamp]

        row = {
            "time": timestamp,
            "real_pose_x": data["real_poses"]["pose_x"],
            "real_pose_y": data["real_poses"]["pose_y"],
            "real_pose_z": data["real_poses"]["pose_z"]
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
    
    def cycles_callback(self, cycles):
        line, rectangle, circle = cycles.line, cycles.rectangle, cycles.circle
        
        # Dosya daha önceden varsa yeni dosya açılabilir. Burayı dinamikleştir..
        if line == 1:
            self.df.to_excel("imu_saved_data.xlsx", index=False)
            self.get_logger().info("Döngü bitmiştir...")
            rclpy.shutdown()

def main():
    rclpy.init()
    node = ImuLogger()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()