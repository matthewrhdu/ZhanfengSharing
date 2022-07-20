from data_structures.srv import Boolsa

import rclpy
from rclpy.node import Node


class Service(Node):
    def __init__(self):
        super().__init__('service')
        self.service = self.create_service(Boolsa, 'boolsa', self.add_two_ints_callback)

    def add_two_ints_callback(self, request, response):
        response.response = not request.request
        self.get_logger().info(f'I got {request.request}')

        return response


def main(args=None):
    rclpy.init(args=args)

    service = Service()
    rclpy.spin(service)

    rclpy.shutdown()


if __name__ == '__main__':
    main()