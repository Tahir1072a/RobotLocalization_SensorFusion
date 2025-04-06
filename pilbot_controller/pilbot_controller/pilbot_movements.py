import rclpy
from rclpy.node import Node

from geometry_msgs.msg import TwistStamped
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
        self.timer =  self.create_timer(0.25, self.timer_callback)

        self.movement_type = self.get_parameter("movement_type").get_parameter_value().string_value
        self.linear_speed = self.get_parameter("linear_speed").get_parameter_value().double_value
        self.angular_speed = self.get_parameter("angular_speed").get_parameter_value().double_value
        self.distance_to_travel = self.get_parameter("distance_to_travel").get_parameter_value().double_value

        self.get_logger().info(f"Paramters: movement type {self.movement_type}, linear_speed {self.linear_speed}, angular_speed {self.angular_speed}, travel distance {self.distance_to_travel}")
    
    def timer_callback(self):

        published_msg = TwistStamped()

        if self.movement_type == "rectangle":
            pass
        elif self.movement_type == "circle":
            published_msg.twist.linear.x = self.linear_speed
            published_msg.twist.angular.z = self.angular_speed
        else:
            pass

        self.pub.publish(published_msg)

def main():
    rclpy.init()
    node = PilbotMovements()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()