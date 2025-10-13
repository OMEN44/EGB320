import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_msgs.msg import Float32MultiArray
from robot_interfaces.msg import PoiGroup

import robot.vision.video as v
import robot.vision.image as i
from robot.vision.utils import setupFakeCam, setupCameraWithDefaults

import numpy as np

import cv2

import csv
import os
import json

BASE_URL = '/home/pi/EGB320/'

class Vision(Node):
    def __init__(self):
        super().__init__('vision_node')
        self.get_logger().info('Vision node has been started.')

        # Pipeline options:
        # aisleMarkers
        # pickingMarkers
        # pickingStation
        # items
        # obstacles
        # shelves
        # colourMask
        # self.pipeline = ["pickingStation", "aisleMarkers", "items", "colourMask", "shelves", "obstacles"]
        # self.pipeline = ["shelves", "aisleMarkers", "items", "colourMask", ""]
        # self.pipeline = ["items", "colourMask", "pickingStation"]
        # self.pipeline = ["test", "colourMask"]
        self.pipeline = ["pickingStation", "pickingMarkers", "aisleMarkers", "items", "shelves", "colourMask", "obstacles"]

        # Initialise subscribers
        self.filters = self.create_subscription(String, '/pipeline_filters', self.updatePipeline, 10)
        self.webuiTopic = self.create_subscription(String, '/web_topic', self.onWebMessage, 10)
        self.orderTopic = self.create_subscription(String, '/order', self.onOrderMessage, 10)
        self.numberTopic = self.create_subscription(Float32MultiArray, '/number_topic', self.onNumberMessage, 10)

        # Initialise publishers
        self.poi = self.create_publisher(PoiGroup, '/poi', 10)
        self.orderFilePub = self.create_publisher(String, '/order_file', 10)

        # Initialise timers
        timer_period = 0.00001
        self.videoTimer = self.create_timer(timer_period, self.consumeFrame)

        # node variables
        self.sink = setupFakeCam()
        self.cap = setupCameraWithDefaults()
        self.colourMask = (np.array([70,0,0]), np.array([100, 255, 255]))
        self.areaLimit = [100, 500]
        self.frameName = 'isle1'
        self.number = [1, 1.3]
        self.poiHistory = {
            'items': [],
            'ramps': [],
            'shelves': [],
            'obstacles': [],
            'aisle_markers': [],
            'picking_markers': [],
            'picking_station': [],
        }
        self.calibration = {
            'map1': None,
            'map2': None
        }
        v.initCalibration(self)

    def consumeFrame(self):
        video = True

        if video:
            v.useVideo(self)
        else:
            i.useImage(self, self.frameName)

    def onWebMessage(self, msg):
        # self.get_logger().info(f'Received message from web UI: {msg.data}')
        # self.frameName = msg.data
        ret, frame = self.cap.read()
        if not ret:
            print("Failed to grab frame")
            return

        cv2.imwrite(f'/home/pi/EGB320/software/python/calibration/{msg.data}.jpg', frame)

    def onOrderMessage(self, msg):
        if msg.data == 'init':
            try:
                files = os.listdir(BASE_URL)
                self.files = {}
                for file in files:
                    if file.endswith('.csv'):
                        with open(os.path.join(BASE_URL, file), mode='r', newline='') as f:
                            reader = csv.reader(f)
                            next(reader)
                            self.files[file] = list(reader)
            except Exception as e:
                self.get_logger().info(f'Error accessing base directory: {e}')

            if len(self.files) > 0:
                self.orderFilePub.publish(String(data=json.dumps(self.files)))
        else:
            print(f'Order received: {msg.data}')

    def onNumberMessage(self, msg):
        self.colourMask = (np.array([msg.data[0], msg.data[2], msg.data[4]]), np.array([msg.data[1], msg.data[3], msg.data[5]]))
        self.areaLimit = [msg.data[6], msg.data[7]]
        self.number = [msg.data[8], msg.data[9]]


    def updatePipeline(self, msg):
        self.pipeline = msg.data.split(',')

def main():
    rclpy.init()
    vision_node = Vision()
    rclpy.spin(vision_node)
    vision_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
