import rclpy
from rclpy.node import Node

from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
# Sadece circular hareekt tanımlanmıştır!!
class PilbotMovements(Node):
    def __init__(self):
        super().__init__("pilbot_movements")
        self.get_logger().info("Pilbot Movements Node has been started")

        self.declare_parameter("movement_type", "circle") #rectangle, circle, line
        self.declare_parameter("linear_speed", 1.0)
        self.declare_parameter("angular_speed", 0.5)
        self.declare_parameter("distance_to_travel", 10.0) # metre => for rectangle_movement and line

        self.pub = self.create_publisher(TwistStamped, "pilbot_controller/cmd_vel", 10)

        self.movement_type = self.get_parameter("movement_type").get_parameter_value().string_value
        self.linear_speed = self.get_parameter("linear_speed").get_parameter_value().double_value
        self.angular_speed = self.get_parameter("angular_speed").get_parameter_value().double_value
        self.distance_to_travel = self.get_parameter("distance_to_travel").get_parameter_value().double_value

        self.get_logger().info(f"Paramters: movement type {self.movement_type}, linear_speed {self.linear_speed}, angular_speed {self.angular_speed}, travel distance {self.distance_to_travel}")

        self.pose_sub = self.create_subscription(Odometry, "pilbot/real_pose", self.pose_callback , 10) # Gazebo pose subscription

        self.initial_move = True
        self.initial_pose = {}

    def rectangle_movement(self, vehicle_pose):
        x, y, theta = vehicle_pose.x, vehicle_pose.y, vehicle_pose.z

        self.get_logger().info(f"x: {x} y: {y}, z: {theta}")
        
    # Düzeltilecek
    def line_movement(self, vehicle_pose):
        x, y, theta = vehicle_pose.x, vehicle_pose.y, vehicle_pose.z
        
        if self.initial_move:
            self.initial_pose["x"] = x
            self.initial_move = False

        linear_speed = 0.0
        is_stop = False
        is_moving_back = False

        if x >= self.initial_pose["x"] + self.distance_to_travel and not is_moving_back:
            linear_speed = 0.0
            is_stop = True
        elif is_stop and linear_speed == 0.0:
            is_moving_back = True
            is_stop = False
            linear_speed = -self.linear_speed
        elif not is_moving_back:
            linear_speed = self.linear_speed
        elif self.initial_pose >= x:
            linear_speed = 0.0
            is_moving_back = False

        return linear_speed
        

    def pose_callback(self, msg):
        pilbot_position = msg.pose.pose.position
        pilbot_orientation_z = msg.pose.pose.orientation.z
        pilbot_position.z = pilbot_orientation_z

        published_msg = TwistStamped()

        if self.movement_type == "rectangle":
            self.rectangle_movement(vehicle_pose=pilbot_position)
        elif self.movement_type == "circle":
            published_msg.twist.linear.x = self.linear_speed
            published_msg.twist.angular.z = self.angular_speed
        else:
            linear_vel = self.line_movement(vehicle_pose=pilbot_position)
            published_msg.twist.linear.x = linear_vel
            published_msg.twist.angular.z = 0.0

        self.pub.publish(published_msg)

        
        

def main():
    rclpy.init()
    node = PilbotMovements()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()