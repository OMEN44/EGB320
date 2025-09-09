import rclpy
from rclpy.node import Node

import pyfakewebcam
import cv2
import numpy as np

cap = cv2.VideoCapture(0)

class Vision(Node):
    def __init__(self):
        super().__init__('vision_node')
        self.get_logger().info('Vision node has been started.')

def main():
    rclpy.init()
    vision_node = Vision()
    rclpy.spin(vision_node)
    vision_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
