import rclpy
from rclpy.node import Node
from std_msgs.msg import String

import robot.vision.video as v
import robot.vision.image as i

from time import sleep

WIDTH = 640
HEIGHT = 480

class Vision(Node):
    def __init__(self):
        super().__init__('vision_node')
        self.get_logger().info('Vision node has been started.')

        self.publisher = self.create_publisher(String, 'poi', 10)

        video = True

        if (video):
            v.useVideo(self)
        else:
            # while True:
            i.useImage(self, '1mb')
            # sleep(3)
            # i.useImage(self, '1mb')
            # sleep(3)
            # i.useImage(self, '1mc')
            # sleep(3)


def main():
    rclpy.init()
    vision_node = Vision()
    rclpy.spin(vision_node)
    vision_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
