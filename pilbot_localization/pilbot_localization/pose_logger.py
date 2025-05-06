#!/usr/bin/env python3
import rclpy
import pandas as pd
from rclpy.node import Node

from nav_msgs.msg import Odometry
from threading import Lock
from pilbot_msgs.msg import MovementCycles
from std_msgs.msg import Int16

# Odometry fused data'dan gelen verileri alıp bunları real pose data'dan gelen veriler ile kıyaslayacak ve hata değerlerini pose_logger.xlns dosyasına loglayacak.
class PoseLogger(Node):
    def __init__(self):
        super().__init__("pose_logger")
        self.get_logger().info("Pose Logger Node has been started")

        self.df = pd.DataFrame(columns=["time"])

        self.buffer = []
        self.buffer_lock = Lock()
        self.buffer_window = 0.01
        self.max_buffer_age = 1.0

        self.create_subscription(Odometry, "/odometry/filtered_fused", lambda msg: self.odometry_callback_("fused", msg), 20)
        self.create_subscription(Odometry, "/pilbot/real_pose", lambda msg: self.odometry_callback_("real", msg), 20)

        self.movement_cycle_sub = self.create_subscription(MovementCycles, "movement_cycles", self.cycles_callback, 10)
        self.test_period_sub = self.create_subscription(Int16, "test_timer", self.test_period_callback, 10)

        self.timer = self.create_timer(1.0, self.cleanup_old_entries)

    def timestamp_to_float(self, header):
        return header.stamp.sec + header.stamp.nanosec * 1e-9

    def odometry_callback_(self, odometry_id, msg):
        timestamp = self.timestamp_to_float(msg.header)

        found_entry = None
        with self.buffer_lock:
            for entry in self.buffer:
                if abs(float(entry["timestamp"]) - timestamp) < self.buffer_window:
                    found_entry = entry
                    break

            if found_entry:
                found_entry["data"][odometry_id] = msg
                #self.get_logger().info(f"Added to existing entry with timestamp {found_entry['timestamp']}")

                if len(found_entry["data"]) == 2:
                    self.prepare_data(found_entry)
                    self.buffer.remove(found_entry)

            else:
                new_entry = {
                    "timestamp": timestamp,
                    "data": {odometry_id: msg}
                }
                self.buffer.append(new_entry)
    
    def prepare_data(self, entry):
        datas = entry["data"]

        real_pose = datas["real"]
        fused_pose = datas["fused"]
        
        fused_pose_x = fused_pose.pose.pose.position.x
        fused_pose_y = fused_pose.pose.pose.position.y
        fused_pose_yaw = fused_pose.pose.pose.orientation.z

        estimated_error_x = abs(float(real_pose.pose.pose.position.x) - float(fused_pose_x)) ** 2
        estimated_error_y = abs(float(real_pose.pose.pose.position.y) - float(fused_pose_y)) ** 2
        estimated_yaw_error = abs(float(real_pose.pose.pose.orientation.z) - float(fused_pose_yaw)) ** 2
        
        t1 = self.timestamp_to_float(real_pose.header)
        t2 = self.timestamp_to_float(fused_pose.header)
        avg_time = (t1 + t2) / 2
        avg_sec = int(avg_time)
        avg_nsec = int((avg_time - avg_sec) * 1e9)
        
        row = {
            "time": f"{avg_sec}.{avg_nsec:09d}",
            "real_pose_x": real_pose.pose.pose.position.x,
            "real_pose_y": real_pose.pose.pose.position.y,
            "real_yaw": real_pose.pose.pose.orientation.z,
            "fused_pose_x": fused_pose_x,
            "fused_pose_y": fused_pose_y,
            "fused_pose_yaw": fused_pose_yaw,
            "estimated_error_x": estimated_error_x,
            "estimated_error_y": estimated_error_y,
            "estimated_yaw_error": estimated_yaw_error,
        }

        self.df = pd.concat([self.df, pd.DataFrame([row])], ignore_index=True)

    def cycles_callback(self, cycles):
        line, rectangle, circle = cycles.line, cycles.rectangle, cycles.circle
        
        # Dosya daha önceden varsa yeni dosya açılabilir. Burayı dinamikleştir..
        if line == 1 or rectangle == 1 or circle == 1:
            self.df.to_excel("pose_saved_data.xlsx", index=False)
            self.get_logger().info("Döngü bitmiştir...")
            rclpy.shutdown()

    def cleanup_old_entries(self):
        current_time = self.get_clock().now().nanoseconds / 1e9
        with self.buffer_lock:
            self.buffer = [entry for entry in self.buffer if (current_time - entry["timestamp"]) < self.max_buffer_age]

    def test_period_callback(self, msg):
        period = msg.data

        if period == 1:
            self.df.to_excel("pose_saved_data.xlsx", index=False)
            self.get_logger().info("Test time is over!")
            rclpy.shutdown()

def main():
    rclpy.init()
    node = PoseLogger()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()