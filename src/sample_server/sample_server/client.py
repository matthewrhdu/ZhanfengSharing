from data_structures.srv import Boolsa

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.client import Client
from rclpy.subscription import Subscription

from std_msgs.msg import Bool

from threading import Thread

class Client(Node):
    """ A client node with a subscriber and a service client

    :ivar client: The client to do things with. In this case, it is the one found in the tutorial since I am lazy :)
    :ivar first_subscription: The subscription to the first publisher
    :ivar first: The value from the first subscriber
    """
    client: Client
    first_subscription: Subscription

    def __init__(self) -> None:
        """ Initializer """
        super().__init__('client')            
        self.client = self.create_client(Boolsa, 'boolsa', callback_group=MutuallyExclusiveCallbackGroup())

        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        
        self.first_subscription = self.create_subscription(Bool, 'first', self.first_callback, 10, callback_group=MutuallyExclusiveCallbackGroup())
        self.first = None

    def first_callback(self, msg) -> None:
        """ The callback function for the first subscriber """
        self.first = bool(msg.data)

    def send_request(self) -> None:
        """ Sending a request """
        self.get_logger().info("Waiting for publishers")
        
        # Since we are on another thread in this function, this won't run forever. This simply blocks execution on this thread until self.first has been set. Then, it continues.
        while self.first is None:
            pass

        self.get_logger().info("Both requests received")
        request = Boolsa.Request()
        request.request = self.first
        
        future = self.client.call_async(request)
        self.get_logger().info("Requests Sent")
        while not future.done():
            pass
        
        self.get_logger().info(f'Response Received: not {self.first} = {future.result().response}')


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