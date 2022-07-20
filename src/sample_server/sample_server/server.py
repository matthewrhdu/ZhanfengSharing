from example_interfaces.srv import AddTwoInts

import rclpy
from rclpy.node import Node


class Service(Node):
    def __init__(self):
        super().__init__('service')
        self.service = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)

    def add_two_ints_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info('Incoming request\na: %d b: %d' % (request.a, request.b))

        return response


def main(args=None):
    rclpy.init(args=args)

    service = Service()
    try:
        rclpy.spin(service)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()


if __name__ == '__main__':
    main()