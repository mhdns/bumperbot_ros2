import rclpy
from rclpy.node import Node
from bumperbot_msgs.srv import AddTwoInts
import sys
class SimpleServiceClient(Node):
    def __init__(self, a, b):
        super().__init__('simple_service_client')
        self.client_ = self.create_client(AddTwoInts, 'add_two_ints')

        while not self.client_.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')

        self.req = AddTwoInts.Request()
        self.req.a = a
        self.req.b = b
        self.future_ = self.client_.call_async(self.req)
        self.future_.add_done_callback(self.responseCallback)

    def responseCallback(self, future):
        self.get_logger().info(f'future result: {future.result().sum}')

def main(args=None):
    rclpy.init(args=args)
    if len(sys.argv) != 3:
        print("Wrong number of arguments, usage: python simple_service_client.py <a> <b>")
        return -1
    simple_service_client = SimpleServiceClient(int(sys.argv[1]), int(sys.argv[2]))
    rclpy.spin(simple_service_client)
    simple_service_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()