import rclpy
from rclpy.node import Node
from std_msgs.msg import String

import gpiozero
import time

class Mobility(Node):
    def __init__(self):
        super().__init__('mobility_node')
        self.get_logger().info('Mobility node has been started.')

        self.subscription = self.create_subscription(
            String,
            'topic',
            self.listener_callback,
            10)

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)

def main():
    rclpy.init()
    mobility_node = Mobility()
    rclpy.spin(mobility_node)
    mobility_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()