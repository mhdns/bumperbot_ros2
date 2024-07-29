import rclpy
from rclpy.node import Node
from bumperbot_msgs.srv import AddTwoInts

class SimpleServiceServer(Node):
    def __init__(self):
        super().__init__('simple_service_server')

        self.service_ = self.create_service(AddTwoInts, "add_two_ints", self.serviceCallback)

        self.get_logger().info('SimpleServiceServer started')

    def serviceCallback(self, req, res):
        self.get_logger().info(f'New request received with {req.a} and {req.b}')
        res.sum = req.a + req.b
        self.get_logger().info(f'Returning response with {res.sum}')

        return res

def main(args=None):
    rclpy.init(args=args)
    simple_service_server = SimpleServiceServer()
    rclpy.spin(simple_service_server)
    simple_service_server.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()