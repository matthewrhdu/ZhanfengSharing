from example_interfaces.srv import AddTwoInts

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.client import Client
from rclpy.subscription import Subscription

from std_msgs.msg import Int8

from threading import Thread

class Client(Node):
    """ A client node with a subscriber and a service client

    :ivar client: The client to do things with. In this case, it is the one found in the tutorial since I am lazy :)
    :ivar first_subscription: The subscription to the first publisher
    :ivar second_subscription: The subscription to the second publisher
    """
    client: Client
    first_subscription: Subscription
    second_subscription: Subscription

    def __init__(self) -> None:
        """ Initializer """
        super().__init__('client')            
        self.client = self.create_client(AddTwoInts, 'add_two_ints', callback_group=MutuallyExclusiveCallbackGroup())

        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        
        self.first_subscription = self.create_subscription(Int8, 'first', self.first_callback, 10, callback_group=MutuallyExclusiveCallbackGroup())
        self.first = None

        self.second_subscription = self.create_subscription(Int8, 'second', self.second_callback, 10, callback_group=MutuallyExclusiveCallbackGroup())
        self.second = None

    def first_callback(self, msg) -> None:
        """ The callback function for the first subscriber """
        self.first = int(msg.data)

    def second_callback(self, msg) -> None:
        """ The callback function for the second subscriber """
        self.second = int(msg.data)

    def send_request(self) -> None:
        """ Sending a request """
        self.get_logger().info("Waiting for publishers")
        while self.first is None or self.second is None:
            pass
        
        self.get_logger().info("Both requests received")
        req = AddTwoInts.Request()
        req.a = self.first
        req.b = self.second

        
        future = self.client.call_async(req)
        self.get_logger().info("Requests Sent")
        while not future.done():
            pass
        
        self.get_logger().info(f'Response Received: {self.first} + {self.second} = {future.result().sum}')


def main(args=None):
    """ The main function """
    rclpy.init(args=args)

    client = Client()

    # This creates a parallel thread of execution that will execute the `send_request` method of the client node. This is because I want the send request to run concurrently with the callbacks of the node.
    Thread(target=client.send_request).start()

    # I am using a MultiThreadedExecutor here as I want all the callbacks to run on a different thread each 
    executor = MultiThreadedExecutor()
    executor.add_node(client)
    executor.spin()
        
    client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()