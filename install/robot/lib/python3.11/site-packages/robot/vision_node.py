import rclpy
from rclpy.node import Node

import robot.vision.video as v

class Vision(Node):
    def __init__(self):
        super().__init__('vision_node')
        self.get_logger().info('Vision node has been started.')
        v.useVideo()

def main():
    rclpy.init()
    vision_node = Vision()
    rclpy.spin(vision_node)
    vision_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
