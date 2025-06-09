import rclpy
from rclpy.node import Node

from std_msgs.msg import String

from rosgraph_msgs.msg import Clock
import time


class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            Clock,
            '/clock',
            self.listener_callback,
            1)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        printprint("{} {}".format(msg.clock.sec,msg.clock.nanosec))("{} {}".format(msg.clock.sec,msg.clock.nanosec))
        print("time: {}".format(time.time()))
        #self.get_logger().info('I heard: "%s"' % msg.data)


def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
