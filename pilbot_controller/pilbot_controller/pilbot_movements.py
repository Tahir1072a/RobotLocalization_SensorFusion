import rclpy
import math
from rclpy.node import Node

from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from pilbot_msgs.msg import MovementCycles

from tf_transformations import euler_from_quaternion

# Rectangle movement ve circle movement tam olarak tamamlanmadı...
class PilbotMovements(Node):
    def __init__(self):
        super().__init__("pilbot_movements")
        self.get_logger().info("Pilbot Movements Node has been started")

        self.declare_parameter("movement_type", "line") #rectangle, circle, line
        self.declare_parameter("linear_speed", 1.5)
        self.declare_parameter("angular_speed", 0.4)
        self.declare_parameter("distance_to_travel", 5.0) # metre => for rectangle_movement and line

        self.pub = self.create_publisher(TwistStamped, "pilbot_controller/cmd_vel", 10)
        self.movement_cycle_pub = self.create_publisher(MovementCycles, "movement_cycles", 10)

        self.movement_type = self.get_parameter("movement_type").get_parameter_value().string_value
        self.linear_speed = self.get_parameter("linear_speed").get_parameter_value().double_value
        self.angular_speed = self.get_parameter("angular_speed").get_parameter_value().double_value
        self.distance_to_travel = self.get_parameter("distance_to_travel").get_parameter_value().double_value

        self.get_logger().info(f"Paramters: movement type {self.movement_type}, linear_speed {self.linear_speed}, angular_speed {self.angular_speed}, travel distance {self.distance_to_travel}")

        self.pose_sub = self.create_subscription(Odometry, "pilbot/real_pose", self.pose_callback , 10) # Gazebo pose subscription

        self.initial_pose = {}
        self.movement_cycle_msg = MovementCycles()
        
        # Vehicle states
        self.initial_move = True
        self.is_starting_point = False
        self.is_finished_point = False
        self.is_on_the_path = False

        # Line movement
        self.is_moving_forward = False
        self.is_moving_back = False
        self.line_turn_num = -0.5

        # Rectangle Movement
        self.move_start_pose = 0.0
        self.turn_start_orientation = 0.0
        self.rectangle_turn_num = 0.0
        self.is_ready = False
        self.is_moving = False
        self.is_stop = False
        self.is_turning_right = False

    def rectangle_movement(self, vehicle_pose):
        x, y, theta = vehicle_pose.x, vehicle_pose.y, vehicle_pose.z
        
        angular_speed = 0.0
        lineer_speed = 0.0

        if self.initial_move:
            self.move_start_pose = x
            self.turn_start_orientation = theta
            self.initial_move = False
            self.is_starting_point = True

        if self.is_starting_point:
            self.is_moving = True
            self.is_starting_point = False
        if self.is_stop:
            self.is_turning_right = True
            self.is_stop = False
            self.turn_start_orientation = theta
            self.get_logger().info("Start to turning right")
        if self.is_turning_right:
            angular_speed = self.angular_speed
            lineer_speed = 0.0
            #self.get_logger().info(f"{self.turn_start_orientation}")
            self.get_logger().info(f"{abs(self.turn_start_orientation - theta)}")
            if abs(self.turn_start_orientation - theta) >= 1.57:
                self.is_turning_right = False
                self.is_ready = True
                self.rectangle_turn_num += 0.25
                self.get_logger().info("Ready")
        if self.is_moving:
            lineer_speed = self.linear_speed
            angular_speed = 0.0
            if self.rectangle_turn_num - int(self.rectangle_turn_num) == 0.0 or self.rectangle_turn_num - int(self.rectangle_turn_num) == 0.5:
                if x >= self.move_start_pose + self.distance_to_travel or x <= self.move_start_pose - self.distance_to_travel:
                    self.is_moving = False
                    self.is_stop = True
            else:
                if y >= self.move_start_pose + self.distance_to_travel or y <= self.move_start_pose - self.distance_to_travel:
                    self.is_moving = False
                    self.is_stop = True
        if self.is_ready:
            if self.rectangle_turn_num - int(self.rectangle_turn_num) == 0.0 or self.rectangle_turn_num - int(self.rectangle_turn_num) == 0.5:
                self.move_start_pose = x
            else:
                self.move_start_pose = y
            self.is_moving = True
            self.is_ready = False
            self.get_logger().info("Start to moving forward")

        self.movement_cycle_msg.rectangle = int(self.rectangle_turn_num)
        self.movement_cycle_pub.publish(self.movement_cycle_msg)

        return lineer_speed, angular_speed
        
    def line_movement(self, vehicle_pose):
        x, y, theta = vehicle_pose.x, vehicle_pose.y, vehicle_pose.z
        
        if self.initial_move:
            self.initial_pose["x"] = x
            self.initial_move = False
            self.is_starting_point = True

        linear_speed = 0.0

        if self.is_starting_point:
            linear_speed = self.linear_speed
            self.is_starting_point = False
            self.is_on_the_path = True
            self.is_moving_forward = True
            self.line_turn_num += 1
            self.get_logger().info("Start to moving forward")
        elif self.is_on_the_path and self.is_moving_forward:
            linear_speed = self.linear_speed

            if x >= self.initial_pose["x"] + self.distance_to_travel:
                self.is_finished_point = True
                self.is_on_the_path = False
                self.is_moving_forward = False
            self.get_logger().info("Moving forward")
        elif self.is_finished_point:
            linear_speed = -self.linear_speed
            self.is_finished_point = False
            self.is_moving_back = True
            self.is_on_the_path = True
            self.get_logger().info("Start to moving backward")
        elif self.is_on_the_path and self.is_moving_back:
            linear_speed = -self.linear_speed

            if x <= self.initial_pose["x"]:
                self.is_starting_point = True
                self.is_on_the_path = False
                self.is_moving_back = False
            self.get_logger().info("Moving backward")

        #self.get_logger().info(f"States: start: {self.is_starting_point}, finish: {self.is_finished_point}, on the path: {self.is_on_the_path}, moving forward: {self.is_moving_forward}, moving_backward: {self.is_moving_back}")

        self.movement_cycle_msg.line = int(self.line_turn_num)
        self.movement_cycle_pub.publish(self.movement_cycle_msg)

        return linear_speed
        

    def pose_callback(self, msg):
        pilbot_position = msg.pose.pose.position
        roll, pitch, yaw = euler_from_quaternion([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])
        pilbot_orientation_z = yaw
        pilbot_position.z = pilbot_orientation_z

        published_msg = TwistStamped()

        if self.movement_type == "rectangle":
            linear_vel, angular_vel = self.rectangle_movement(vehicle_pose=pilbot_position)
            published_msg.twist.linear.x = linear_vel
            published_msg.twist.angular.z = angular_vel
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