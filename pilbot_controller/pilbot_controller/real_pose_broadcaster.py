import rclpy
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node


class RealPoseBroadcaster(Node):
    def __init__(self):
        super().__init__("real_pose_broadcaster")

        self.real_pose_sub = self.create_subscription(Odometry, "pilbot/real_pose", self.real_pose_callback, 10)

        self.br = TransformBroadcaster(self)
        self.transform_stamped = TransformStamped()
        self.transform_stamped.header.frame_id = "odom"
        self.transform_stamped.child_frame_id = "base_footprint_real"

    
    def real_pose_callback(self, msg):
        self.transform_stamped.transform.translation.x = msg.pose.pose.position.x
        self.transform_stamped.transform.translation.y = msg.pose.pose.position.y
        self.transform_stamped.transform.translation.z = msg.pose.pose.position.z
        self.transform_stamped.transform.rotation.x = msg.pose.pose.orientation.x
        self.transform_stamped.transform.rotation.y = msg.pose.pose.orientation.y
        self.transform_stamped.transform.rotation.z = msg.pose.pose.orientation.z
        self.transform_stamped.transform.rotation.w = msg.pose.pose.orientation.w
        self.transform_stamped.header.stamp = msg.header.stamp

        self.br.sendTransform(self.transform_stamped)

def main():
    rclpy.init()
    node = RealPoseBroadcaster()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()