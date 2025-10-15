import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32 # Import the Int32 message type
from std_msgs.msg import Bool
from std_msgs.msg import Float64MultiArray
from robot_interfaces.msg import PoiGroup
from PiicoDev_MPU6050 import PiicoDev_MPU6050

import robot.navigation.apf as apf
from robot.mobility.status import statusLEDs

import numpy as np
import time

class Navigation(Node):
    def __init__(self):
        super().__init__('navigation_node')
        self.get_logger().info('Navigation node has been started.')

        self.ref_bearing = None

        # Publishers
        self.pipeline_pub = self.create_publisher(String, '/pipeline_filters', 10)
        self.target_item_pub = self.create_publisher(String, '/target_item', 10)
        self.arm_action_pub = self.create_publisher(Int32, '/arm_action', 10)
        self.gripper_action_pub = self.create_publisher(Bool, '/gripper_action', 10)
        self.velocities_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.skid_velocities_pub = self.create_publisher(Float64MultiArray, '/skid_cmd_vel', 10)
        self.collection_pub = self.create_publisher(Int32, '/collection_action', 10)

        # Subscribers
        self.point_of_interest_sub = self.create_subscription(PoiGroup, "/poi", self.poi_callback, 10)
        self.arm_status_sub = self.create_subscription(Bool, "/arm_status", self.arm_status_callback, 10)

        # State machine variables
        self.state = 'START'
        self.current_heading = 0.0
        self.target_heading = 0.0

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

        # PID
        self.integral = 0.0
        self.e_prev = 0.0
        self.time = time.time()
        self.time_prev = time.time()


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

    def PID(self, Kp, Ki, Kd, setpoint, measurement):
        # if (self.time - self.time_prev) == 0:
        #     return 0

        self.time = time.time()
        offset = 0
        # PID calculations
        e = setpoint - measurement 
        P = Kp*e
        self.integral = self.integral + Ki*e*(self.time - self.time_prev)
        D = Kd*(e - self.e_prev) / (self.time - self.time_prev)  # calculate manipulated variable - MV 
        MV = offset + P + self.integral + D 
        # update stored data for next iteration
        self.e_prev = e
        self.time_prev = self.time
        return MV
    
    def bearing_to_angle(self, bearing):
        angle = bearing / 1000 * 180 / np.pi # Convert radians to millidegrees
        return angle

    # ---------------- STATE MACHINE --------------
    def state_machine(self):
        self.get_logger().info(str(self.state))
        if self.state == 'START':
            self.aisle_index = None
            self.state = 'PID'

        elif self.state == 'PID':
            self.send_vision_data("shelves,obstacles,pickingStation", "")

            Kp = 0.0005
            Ki = 0.0
            Kd = 0.05

            if len(self.shelves) > 0 and len(self.picking_station) > 0:
                left_shelf_bearing = 100000 # large number
                right_picking_bearing = -100000 # large negative number
                for shelf in self.shelves:
                    if shelf.bearing[0] < left_shelf_bearing:
                        left_shelf_bearing = shelf.bearing[0]
                for picking in self.picking_station:
                    if picking.bearing[2] > right_picking_bearing:
                        right_picking_bearing = picking.bearing[2]

                output = self.PID(Kp, Ki, Kd, 0, left_shelf_bearing + (right_picking_bearing - left_shelf_bearing) / 2)
                # output = 0
                scale = 0.5
                self.skid_velocities_pub.publish(Float64MultiArray(data=[scale * output, scale * output]))
                # turn_vel = max(0.4)
                self.get_logger().info(f'left: {scale * output} right: {scale * output}')
            else:
                if len(self.shelves) == 0: # No shelves detected
                    self.skid_velocities_pub.publish(Float64MultiArray(data=[0.4, -0.4]))
                    self.get_logger().info('No shelves detected, rotating to find shelves...')
                elif len(self.picking_station) == 0: # No picking station detected
                    self.skid_velocities_pub.publish(Float64MultiArray(data=[-0.4, 0.4]))
                    self.get_logger().info('No picking station detected, moving forward to find picking station...')
                else:
                    self.publish_velocity(0, 0.6) # Move forward


        elif self.state == 'DONE':
            self.publish_velocity(0.0, 0.0)
            self.get_logger().info("Task complete. Standing by...")
    # ---------------------------------------------


def main():
    rclpy.init()
    navigation_node = Navigation()
    rclpy.spin(navigation_node)
    navigation_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
