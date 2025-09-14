import rclpy
from rclpy.node import Node
from std_msgs.msg import String

import gpiozero
import time

class Navigation(Node):
    def __init__(self):
        super().__init__('navigation_node')
        self.get_logger().info('Navigation node has been started.')
        self.pipeline = self.create_publisher(String, '/pipeline_filters', 10)

        # Pipeline options:
        # isleMarkers
        # pickingStation
        # colourMask
        # items
        # obstacles
        # shelves
        self.pipeline.publish(String(data='pickingStation,isleMarkers,colourMask'))



def main():
    rclpy.init()
    navigation_node = Navigation()
    rclpy.spin(navigation_node)
    navigation_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()