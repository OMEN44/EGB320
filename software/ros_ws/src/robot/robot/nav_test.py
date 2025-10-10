import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32 # Import the Int32 message type
from std_msgs.msg import Bool
from robot_interfaces.msg import PoiGroup
import time

# import robot.navigation.apf as apf
# import robot.navigation.imu as imu

import numpy as np
import math
import random

from PiicoDev_MPU6050 import PiicoDev_MPU6050

class Navigation(Node):
    def __init__(self):
        super().__init__('nav_test')
        self.get_logger().info('Navigation node has been started.')

        # Publishers
        self.pipeline_pub = self.create_publisher(String, '/pipeline_filters', 10)
        self.target_item_pub = self.create_publisher(String, '/target_item', 10)
        self.velocities_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        # self.collection_pub = self.create_publisher(Int32, '/collection_action', 10)

        # Subscribers
        self.point_of_interest_sub = self.create_subscription(PoiGroup, "/poi", self.poi_callback, 10)
        self.arm_status_sub = self.create_subscription(Bool, "/arm_status", self.arm_status_callback, 10)

        # State machine variables
        self.state = 'TURN_CALIBRATION'
        self.current_heading = 0.0
        self.target_heading = 0.0

        self.timer_ = self.create_timer(0.001, self.state_machine)

        self.shelves = []
        self.items = []
        self.obstacles = []
        self.aisle_markers = []
        self.picking_stations = []

        self.pois = []

        self.phi = np.linspace(-np.pi, np.pi, 360)  # Shared angular grid for fields

        self.arm_status = False  

        self.zone_dist_aisle_marker = 1.24
        # self.aisle_imu = imu.getYaw()

        # Setup IMU
        calibrationSteps = 100

        self.t = time.time()
        self.Ax_bias = 0
        self.Gz_bias = 0
        self.imu = PiicoDev_MPU6050()
        self.rotation = 0
        self.aX = 0
        self.gZ = 0

        count = 0
        while count < calibrationSteps:
            accel = self.imu.read_accel_data() # read the accelerometer [ms^-2]
            self.Ax_bias += accel["x"]

            gyro = self.imu.read_gyro_data()   # read the gyro [deg/s]
            self.Gz_bias += gyro["z"]
            count += 1

        self.Ax_bias /= calibrationSteps
        self.Gz_bias /= calibrationSteps


    # --------------------------- Publish to vision (objects needed to be detected, target item/picking bay) ---------------------------
    def send_vision_data(self, pipeline_string, target_item_data):
        pipeline_msg = String()
        pipeline_msg.data = pipeline_string
        self.pipeline_pub.publish(pipeline_msg)

        target_item_msg = String()
        target_item_msg.data = target_item_data
        self.target_item_pub.publish(target_item_msg)
        # self.get_logger().info("Sent pipeline and target item data to vision...")

    # --------------------------- Robot movement and actions publisher ---------------------------
    def publish_velocity(self, forward_vel, angular_vel):
        twist_msg = Twist()
        twist_msg.linear.x = forward_vel
        twist_msg.angular.z = angular_vel
        self.velocities_pub.publish(twist_msg)
        # self.get_logger().info(str(forward_vel))
        # self.get_logger().info(str(angular_vel))
        # self.get_logger().info("Twist message sent to mobility...")

    # def publish_collection(self, collection_command):
    #     # Placeholder: implement gripper or collection action
    #     collection_msg = Int32()
    #     collection_msg.data = collection_command
    #     self.collection_pub.publish(collection_msg)
        # self.get_logger().info("Command sent to collection...")

    # --------------------------- Point of Interest callback ---------------------------
    def poi_callback(self, msg):
        self.pois = msg.pois
        # for item in msg.pois:
        #     self.pois.append((item.name, item.type, ((item.distance)/100000), [((item.bearing[0])/1000),((item.bearing[1])/1000), ((item.bearing[2])/1000)]))

    def arm_status_callback(self, msg):
        self.arm_status = msg.data

    # def filter_poi(self, poi_type):
        
    #     listOfPois = []
    #     for poi in self.pois:
    #         if poi_type == poi[0]:
    #             listOfPois.append(poi)

    #     return listOfPois

    # ---------------- STATE MACHINE --------------
        # ---------------- STATE MACHINE --------------
    def state_machine(self):
        # Update IMU data

        accel = self.imu.read_accel_data() # read the accelerometer [ms^-2]
        self.aX = accel["x"] - self.Ax_bias

        gyro = self.imu.read_gyro_data()   # read the gyro [deg/s]
        self.gZ = gyro["z"] - self.Gz_bias

        now = time.time()
        dt = now - self.t
        self.t = now

        self.rotation += self.gZ * dt * (np.pi / 180) # THE ERRORR IS HERE IF TURNIGN BAD

        # Initialize state tracking variables once
        if not hasattr(self, 'start_time'):
            self.start_time = self.get_clock().now()
            self.get_logger().info("Beginning motion sequence.")
        
        # Compute elapsed time
        elapsed = (self.get_clock().now() - self.start_time).nanoseconds / 1e9  # seconds

        msg = Twist()

        # ----------- SEQUENCE CONTROL -----------
        if self.state == 'TURN_CALIBRATION':
            speed = 0.2 # change this to change speed
            # 0.08 in code was 0.001 m/s in real life
            # 0.3 in code was 0.6 m/s in real life
            if elapsed < 5.0:
                self.publish_velocity(speed, 0.0) 
            else:
                self.publish_velocity(0.0, 0.0)
                self.state = 'SEQUENCE_COMPLETE'
                self.get_logger().info("Timed motion sequence complete.")
                self.start_time = self.get_clock().now()  # reset timer if needed

        elif self.state == 'SEQUENCE_COMPLETE':
            # Idle or loop back if desired
            self.publish_velocity(0.0, 0.0)
            # Uncomment to loop sequence:
            # self.state = 'TURN_CALIBRATION'
            pass


def main():
    rclpy.init()
    navigation_node = Navigation()
    rclpy.spin(navigation_node)
    navigation_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
