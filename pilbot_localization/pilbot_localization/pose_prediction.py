#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from threading import Lock
from nav_msgs.msg import Odometry

# 3 Ayrı odometri verisinden alınan bilgilerden tek bir pose değeri üreti ve bunu belli bir topic altında yayınlar.
class PosePrediction(Node):
    def __init__(self):
        super().__init__('pose_prediction')
        self.get_logger().info("Pose Prediction Node has been started")

        self.buffer = []
        self.buffer_lock = Lock()
        self.buffer_window = 0.01
        self.max_buffer_age = 1.0 # şimdiki zamandan en fazla 1 saniye geçene kadar odometry verileri buffer da saklanır.

        self.create_subscription(Odometry, "odometry/filtered_1", lambda msg: self.odometry_callback("odometry_1", msg), 10)
        self.create_subscription(Odometry, "odometry/filtered_2", lambda msg: self.odometry_callback("odometry_2", msg), 10)
        self.create_subscription(Odometry, "odometry/filtered_3", lambda msg: self.odometry_callback("odometry_3", msg), 10)

        self.odometry_fusion_pub = self.create_publisher(Odometry, "odometry/filtered_fused", 10)

        self.cleanup_timer = self.create_timer(1.0, self.cleanup_old_entries)

    def timestamp_to_float(self, header):
        return header.stamp.sec + header.stamp.nanosec * 1e-9

    def odometry_callback(self, odometry_id, msg):
        timestamp = self.timestamp_to_float(msg.header)

        found_entry = None
        with self.buffer_lock:
            for entry in self.buffer:
                if abs(float(entry["timestamp"]) - timestamp) < self.buffer_window:
                    found_entry = entry
                    break
            
            if found_entry:
                found_entry["data"][odometry_id] = msg
                
                if len(found_entry["data"]) == 3:
                    self.fuse_and_publish_odometry(found_entry)
                    self.buffer.remove(found_entry)

            else:
                new_entry = {
                    "timestamp": timestamp,
                    "data": {odometry_id: msg}
                }
                self.buffer.append(new_entry)

    def fuse_and_publish_odometry(self, entry):
        datas = entry["data"]

        odometry_1 = datas["odometry_1"]
        odometry_2 = datas["odometry_2"]
        odometry_3 = datas["odometry_3"]

        # Avarage timestamp
        t1 = self.timestamp_to_float(odometry_1.header)
        t2 = self.timestamp_to_float(odometry_2.header)
        t3 = self.timestamp_to_float(odometry_3.header)
        avg_timestamp = (t1 + t2 + t3) / 3
        avg_sec = int(avg_timestamp)
        avg_nano = int((avg_timestamp - avg_sec) * 1e9)

        fused_odometry = Odometry()
        fused_odometry.header.stamp.sec = avg_sec
        fused_odometry.header.stamp.nanosec = avg_nano
        fused_odometry.header.frame_id = odometry_1.header.frame_id
        fused_odometry.child_frame_id = odometry_1.child_frame_id

        # Odometry verileri basit ortalama ile birleştiriliyor.
        fused_odometry.pose.pose.position.x = (odometry_1.pose.pose.position.x + odometry_2.pose.pose.position.x + odometry_3.pose.pose.position.x) / 3
        fused_odometry.pose.pose.position.y = (odometry_1.pose.pose.position.y + odometry_2.pose.pose.position.y + odometry_3.pose.pose.position.y) / 3
        fused_odometry.pose.pose.position.z = (odometry_1.pose.pose.position.z + odometry_2.pose.pose.position.z + odometry_3.pose.pose.position.z) / 3
        fused_odometry.pose.pose.orientation.x = (odometry_1.pose.pose.orientation.x + odometry_2.pose.pose.orientation.x + odometry_3.pose.pose.orientation.x) / 3
        fused_odometry.pose.pose.orientation.y = (odometry_1.pose.pose.orientation.y + odometry_2.pose.pose.orientation.y + odometry_3.pose.pose.orientation.y) / 3
        fused_odometry.pose.pose.orientation.z = (odometry_1.pose.pose.orientation.z + odometry_2.pose.pose.orientation.z + odometry_3.pose.pose.orientation.z) / 3
        fused_odometry.pose.pose.orientation.w = (odometry_1.pose.pose.orientation.w + odometry_2.pose.pose.orientation.w + odometry_3.pose.pose.orientation.w) / 3
        fused_odometry.twist.twist.linear.x = (odometry_1.twist.twist.linear.x + odometry_2.twist.twist.linear.x + odometry_3.twist.twist.linear.x) / 3
        fused_odometry.twist.twist.linear.y = (odometry_1.twist.twist.linear.y + odometry_2.twist.twist.linear.y + odometry_3.twist.twist.linear.y) / 3
        fused_odometry.twist.twist.linear.z = (odometry_1.twist.twist.linear.z + odometry_2.twist.twist.linear.z + odometry_3.twist.twist.linear.z) / 3
        fused_odometry.twist.twist.angular.x = (odometry_1.twist.twist.angular.x + odometry_2.twist.twist.angular.x + odometry_3.twist.twist.angular.x) / 3
        fused_odometry.twist.twist.angular.y = (odometry_1.twist.twist.angular.y + odometry_2.twist.twist.angular.y + odometry_3.twist.twist.angular.y) / 3
        fused_odometry.twist.twist.angular.z = (odometry_1.twist.twist.angular.z + odometry_2.twist.twist.angular.z + odometry_3.twist.twist.angular.z) / 3
        self.odometry_fusion_pub.publish(fused_odometry)

    def cleanup_old_entries(self):
        now = self.get_clock().now().nanoseconds / 1e9
        with self.buffer_lock:
            self.buffer = [entry for entry in self.buffer if (now - entry["timestamp"]) < self.max_buffer_age]

def main():
    rclpy.init()
    node = PosePrediction()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()