import rclpy
from rclpy.node import Node
from rclpy.publisher import Publisher

from std_msgs.msg import Int8


class First(Node):
    """ A ROS2 Publisher that publishes and Int8 with value '5' on an infinite loop on topic 'first'

    :ivar publisher_: the publisher
    """
    publisher_: Publisher

    def __init__(self) -> None:
        """ Initializer """
        super().__init__('first')
        self.publisher_ = self.create_publisher(Int8, 'first', 10)

    def run(self):
        """ Sends the message on the publisher """
        msg = Int8()
        msg.data = 5
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')


def main(args=None):
    """ Main function """
    rclpy.init(args=args)

    # Creates the node
    first = First()

    # Infinite loop
    while True:
        try:
            # This does NOT need to be spun, since there is no callback in the node.
            first.run()
        
        # This is just so when [ctrl-C] is pressed, the loop can exit and the node can be destroyed (This is not necessary since the garbage cleaner will pick it up)
        except KeyboardInterrupt:
            break

    first.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()