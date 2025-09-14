import rclpy
from rclpy.node import Node
from std_msgs.msg import String

import gpiozero
import time

class Navigation(Node):
    def __init__(self):
        super().__init__('navigation_node')
        self.get_logger().info('Navigation node has been started.')
        # For sending pipeline filter data
        self.pipeline = self.create_publisher(String, '/pipeline_filters', 10)
        # For sending target item data
        self.targetItem = self.create_publisher(String, '/target_item', 10)

        # Pipeline Options:
        # isleMarkers
        # pickingStation
        # colourMask
        # items
        # obstacles
        # shelves
        self.pipeline.publish(String(data='pickingStation,isleMarkers,colourMask'))
        # Item Options:
        # Bowl
        # Coffee Cup
        # Robot Oil Bottle
        # Rubiks Cube
        # Soccer Ball
        # Wheet Bots
        self.targetItem.publish(String(data='Soccer Ball,3')) # item name,isle number



def main():
    rclpy.init()
    navigation_node = Navigation()
    rclpy.spin(navigation_node)
    navigation_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()