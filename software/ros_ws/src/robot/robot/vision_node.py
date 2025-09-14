import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_msgs.msg import Float32MultiArray

import robot.vision.video as v
import robot.vision.image as i
from robot.vision.utils import setupFakeCam, setupCameraWithDefaults

from time import sleep
import numpy as np

WIDTH = 640
HEIGHT = 480

class Vision(Node):
    def __init__(self):
        super().__init__('vision_node')
        self.get_logger().info('Vision node has been started.')

        # Pipeline options:
        # isleMarkers
        # pickingStation
        # colourMask
        # items
        # obstacles
        # shelves
        self.pipeline = ["pickingStation", "isleMarkers", "items", "colourMask", "shelves", "obstacles"]

        # Initialise subscribers
        self.filters = self.create_subscription(String, '/pipeline_filters', self.updatePipeline, 10)
        self.webuiTopic = self.create_subscription(String, '/web_topic', self.onWebMessage, 10)
        self.numberTopic = self.create_subscription(Float32MultiArray, '/number_topic', self.onNumberMessage, 10)

        # Initialise publishers
        self.poi = self.create_publisher(String, '/poi', 10)

        # Initialise timers
        timer_period = 0.00001
        self.videoTimer = self.create_timer(timer_period, self.consumeFrame)

        # node variables
        self.sink = setupFakeCam()
        self.cap = setupCameraWithDefaults()
        self.colourMask = (np.array([70,0,0]), np.array([100, 255, 255]))
        self.areaLimit = [100, 500]
        self.frameName = 'isle1'

    def consumeFrame(self):
        video = False

        if video:
            v.useVideo(self)
        else:
            i.useImage(self, self.frameName)

    def onWebMessage(self, msg):
        self.get_logger().info(f'Received message from web UI: {msg.data}')
        self.frameName = msg.data

    def onNumberMessage(self, msg):
        self.colourMask = (np.array([msg.data[0], msg.data[2], msg.data[4]]), np.array([msg.data[1], msg.data[3], msg.data[5]]))
        self.areaLimit = [msg.data[6], msg.data[7]]


    def updatePipeline(self, msg):
        self.pipeline = msg.data.split(',')
        self.get_logger().info(f'Pipeline updated: {self.pipeline}')

def main():
    rclpy.init()
    vision_node = Vision()
    rclpy.spin(vision_node)
    vision_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
