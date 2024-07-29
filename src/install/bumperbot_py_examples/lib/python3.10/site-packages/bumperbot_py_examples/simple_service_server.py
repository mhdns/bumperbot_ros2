import rclpy
from rclpy.node import Node

class SimpleServiceServer(Node):
    def __init__(self):
        super().__init__('simple_service_server')

        self.service_ = self.create_service()