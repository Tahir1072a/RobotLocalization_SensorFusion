import rclpy
from rclpy.node import Node

class PilpotMapBroadcaster(Node):
    def __init__(self):
        super().__init__("pilbot_map_broadcaster")

        #self_sub = 