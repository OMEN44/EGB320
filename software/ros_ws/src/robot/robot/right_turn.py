import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32 # Import the Int32 message type
from std_msgs.msg import Bool
from std_msgs.msg import Float64MultiArray
from robot_interfaces.msg import PoiGroup
from PiicoDev_MPU6050 import PiicoDev_MPU6050
from PiicoDev_VL53L1X import PiicoDev_VL53L1X

import robot.navigation.apf as apf
from robot.mobility.status import statusLEDs

import numpy as np
import math
import time
import json

SUCCESS_THRESHOLD = 8  # Number of consecutive confirmations needed
BACK_PROOF_LIMIT = 30

SUCCESS_COUNT_THRESHOLD = 10  # Number of consecutive successes required

class Navigation(Node):
    def __init__(self):
        super().__init__('navigation_node')
        self.get_logger().info('Navigation node has been started.')

        self.picking_marker_index = 0
        self.picking_bay_distances= [0.95, 0.6, 0.3]  # Distances to picking bays in meters

        self.back_proof_counter = 0

        self.tof_distance = None
        self.ref_bearing = None
        self.success_count = 0 

        self.prev_tof_distance = None


        self.ref_bearing = None
        self.success_count = 0 

        self.start_turn_imu = 0.0

        # Publishers
        self.pipeline_pub = self.create_publisher(String, '/pipeline_filters', 10)
        self.target_item_pub = self.create_publisher(String, '/target_item', 10)
        self.arm_action_pub = self.create_publisher(Int32, '/arm_action', 10)
        self.gripper_action_pub = self.create_publisher(Bool, '/gripper_action', 10)
        self.velocities_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.collection_pub = self.create_publisher(Int32, '/collection_action', 10)
        self.nav_status_pub = self.create_publisher(String, '/nav_status', 10)

        # Subscribers
        self.point_of_interest_sub = self.create_subscription(PoiGroup, "/poi", self.poi_callback, 10)
        self.arm_status_sub = self.create_subscription(Bool, "/arm_status", self.arm_status_callback, 10)
        self.orders = self.create_subscription(String, '/order', self.order_callback, 10)

        # State machine variables
        self.state = 'START'
        self.current_heading = 0.0
        self.target_heading = 0.0

        # Order queue
        # index: order number
        # station: Picking station (0), 1, 2)
        # name: Item name (Bowl, Bottle, Cube, Ball, Mug, Weetbots)
        # shelf: which blue shelf (0, 1, 2, 3, 4, 5)
        # bay: depth into the isle (0, 1, 2, 3)
        # height: (0, 1, 2)
        self.order_queue = []
        self.order_index = 0



        # Create timers
        self.imuCallback = self.create_timer(0.001, self.imu_callback)
        self.timer_ = self.create_timer(0.001, self.state_machine)

        self.shelves = []
        self.items = []
        self.obstacles = []
        self.aisle_markers = []
        self.picking_station = []
        self.picking_markers = []
        self.phi = np.linspace(-np.pi, np.pi, 360)  # Shared angular grid for fields
        self.arm_status = False  
        self.aisle_index = None
        self.leds = statusLEDs()

        self.linear_accel = None
        self.yaw = None
        self.angular_rot = None

        # IMU
        self.imu = PiicoDev_MPU6050(addr=0x68)
        self.t = time.time()

    
        # Calibrate
        count = 0
        calibrationSteps = 100

        self.Ax_bias = 0
        self.Gz_bias = 0
        self.rot_z = 0
        self.aX = 0
        self.gZ = 0

        self.get_logger().info('Calibrating IMU...')
        while count < calibrationSteps:
            accel = self.imu.read_accel_data() # read the accelerometer [ms^-2]
            self.Ax_bias += accel["x"]

            gyro = self.imu.read_gyro_data()   # read the gyro [deg/s]
            self.Gz_bias += gyro["z"]
            count += 1

        self.Ax_bias /= calibrationSteps
        self.Gz_bias /= calibrationSteps
        self.get_logger().info('Calibration complete. Ax_bias=%.2f m/s², Gz_bias=%.2f °/s' % (self.Ax_bias, self.Gz_bias))

        # Search state for shelf scanning
        self.searching_for_shelf = False
        self.search_start_rot_deg = 0.0  # degrees, taken from self.rot_z when starting search
        self.search_timeout_complete = False  # optional flag if you want to track failure

        # Delivery measurements
        self.item_name = None 
        self.picking_marker_index = None  # Assuming station numbers are 1-indexed
        self.aisle_index = None
        self.picking_bay_distance = None 
        self.shelf_distance = None
        self.shelf_height = None
        self.aisle_distance = None 
        self.aisle_turn_direction = None
        self.shelf_turn_direction = None

        self.prev_imu = None

        # Tof
        self.tof_sensor = PiicoDev_VL53L1X()

    def get_distance(self):
        return self.tof_sensor.read() / 1000
    
    def order_callback(self, msg):
        if msg.data != "init":
            try:
                order_data = json.loads(msg.data)
                order_data['name'] = order_data['name'].strip()
                order_data['station'] = int(order_data['station']) - 1
                order_data['shelf'] = int(order_data['shelf'])
                order_data['bay'] = int(order_data['bay'])
                order_data['height'] = int(order_data['height'])
                self.order_queue.append(order_data)
                status = String()
                status.data = f"delivery queued: {order_data['name']}, station {order_data['station']+1}, shelf {order_data['shelf']}, bay {order_data['bay']}, height {order_data['height']}"
                self.nav_status_pub.publish(status)
            except json.JSONDecodeError:
                self.get_logger().error("Failed to decode order message JSON.")

    def get_delivery_measurements(self, order):
        # Determine picking bay index based on station
        picking_bay_distances = [0.8, 0.6, 0]  # Distances to picking bays (1,2,3) in meters
        shelf_distances =  [0.95, 0.67, 0.35, 0]  # Distances to shelves (0-5) in meters
        aisle_distances = [[0.2, np.pi/2], [0.8, -np.pi/2], [0.2, -np.pi/2]]

        shelf_direction = [np.pi/2, -np.pi/2, np.pi/2, -np.pi/2, np.pi/2, -np.pi/2]

        self.item_name = order[self.order_index]['name'] 
        self.picking_marker_index = order[self.order_index]['station']  # Assuming station numbers are 1-indexed
        self.aisle_index = int((order[self.order_index]['shelf'])/2)
        self.picking_bay_distance = picking_bay_distances[self.picking_marker_index] 
        self.shelf_distance = shelf_distances[order[self.order_index]['bay']]
        self.shelf_height = order[self.order_index]['height']
        self.aisle_distance, self.aisle_turn_direction  = aisle_distances[self.aisle_index]
        self.shelf_turn_direction = shelf_direction[order[self.order_index]['shelf']]
        

    # --------------------------- Publish to vision (objects needed to be detected, target item/picking bay) ---------------------------
    def imu_callback (self):
        accel = self.imu.read_accel_data() # read the accelerometer [ms^-2]
        self.aX = accel["x"] - self.Ax_bias
        
        gyro = self.imu.read_gyro_data()   # read the gyro [deg/s]
        self.gZ = gyro["z"] - self.Gz_bias

        now = time.time()
        dt = now - self.t
        self.t = now

        self.rot_z += self.gZ * dt
    
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

    def publish_collection(self, level, open_grip):
        # Placeholder: implement gripper or collection action
        arm_level = Int32() # Arm level 
        gripper_state = Bool() # Gripper state (open/close)
        arm_level.data = level
        gripper_state.data = open_grip
        self.arm_action_pub.publish(arm_level)
        self.gripper_action_pub.publish(gripper_state)

    # --------------------------- Point of Interest callback ---------------------------
    def poi_callback(self, msg):
        self.shelves = msg.shelves
        self.items = msg.items
        self.obstacles = msg.obstacles
        self.aisle_markers = msg.aisle_markers
        self.picking_station = msg.picking_station
        self.picking_markers = msg.picking_markers

    def arm_status_callback(self, msg):
        self.arm_status = msg.data

    # ---------------- STATE MACHINE --------------
    def state_machine(self):
        # self.get_logger().info(str(self.state))
        # print(self.gZ)
        if self.state == 'START':
            self.prev_imu = self.rot_z  
            self.state = 'TURN_90_DEG'
            self.target_angle = self.prev_imu + 85.0

        elif self.state == 'TURN_90_DEG':
            self.get_logger().info(f'{self.rot_z}, {self.target_angle}, {self.target_angle > self.rot_z}')
            if self.target_angle > self.rot_z:
                self.publish_velocity(0.0, -0.7)
            else:
                self.publish_velocity(0.0, 0.0)
            # Target: turn +90 degrees from the starting orientation
            # target_angle = self.prev_imu + 90.0  # degrees
            # current_angle = self.rot_z            # degrees
            # error = target_angle - current_angle  # difference in degrees

            # # Normalize error to [-180, 180]
            # error = (error + 180) % 360 - 180

            # # Proportional control for smooth turning
            # k_p = 0.015
            # angular_vel = k_p * error

            # # Clamp angular velocity for safety
            # angular_vel = max(min(angular_vel, 0.65), -0.65)

            # if abs(error) < 3:  # within ±3° of target
            #     self.publish_velocity(0.0, 0.0)
            #     self.get_logger().info("90-degree turn complete.")
            #     self.state = 'DONE'
            # else:
            #     self.publish_velocity(0.0, angular_vel)

        # elif self.state == 'TURN_90_DEG':
        #     target_heading = np.radians(self.prev_imu + 85)  # radians
        #     current_heading = np.radians(self.rot_z)  # radians

        #     e_theta = apf.angle_wrap(target_heading - current_heading)

        #     rotation_velocity = 0.7

        #     if abs(np.degrees(e_theta)) < 10:  # close enough to target
        #         self.publish_velocity(0.0, 0.0)
        #         self.state = 'DONE'
        #     else:
        #         self.publish_velocity(0.0, rotation_velocity)


        elif self.state == 'DONE':
            self.publish_velocity(0.0, 0.0)
            # self.get_logger().info("Task complete. Standing by...")
    # ---------------------------------------------


def main():
    rclpy.init()
    navigation_node = Navigation()
    rclpy.spin(navigation_node)
    navigation_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
