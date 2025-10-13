import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32 # Import the Int32 message type
from std_msgs.msg import Bool
from robot_interfaces.msg import PoiGroup
import time

import robot.navigation.apf as apf

import numpy as np
import math
import random

class Navigation(Node):
    def __init__(self):
        super().__init__('navigation_node')
        self.get_logger().info('Navigation node has been started.')

        # Publishers
        self.pipeline_pub = self.create_publisher(String, '/pipeline_filters', 10)
        self.target_item_pub = self.create_publisher(String, '/target_item', 10)
        self.arm_action_pub = self.create_publisher(Int32, '/arm_action', 10)
        self.gripper_action_pub = self.create_publisher(Bool, '/gripper_action', 10)
        self.velocities_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.collection_pub = self.create_publisher(Int32, '/collection_action', 10)

        # Subscribers
        self.point_of_interest_sub = self.create_subscription(PoiGroup, "/poi", self.poi_callback, 10)
        self.arm_status_sub = self.create_subscription(Bool, "/arm_status", self.arm_status_callback, 10)

        # State machine variables
        self.state = 'START'
        self.current_heading = 0.0
        self.target_heading = 0.0

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

        self.aisle_IMU = None
        self.distance_pickingbay1 = None
        self.distance_pickingbay2 = None
        self.picking_bay_marker_distance = 0.29  # meters (distance between the two picking bay markers)

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
        self.get_logger().info(str(self.state))
        if self.state == 'START':
            self.state = 'FIND_PICKING_MARKER_1'

        elif self.state == 'FIND_PICKING_MARKER_1':
            self.send_vision_data("picking_markers", "")
            if self.picking_markers[0].exists:
                picking_bay1_bearing = math.degrees(self.picking_markers[0].bearing[1])
                kp = 0.01
                rotation_velocity = kp * picking_bay1_bearing
                rotation_velocity = max(min(rotation_velocity, 0.3), -0.3)
                if abs(picking_bay1_bearing) < 0.4:
                    self.publish_velocity(0.0, 0.0)
                    self.distance_pickingbay1 = self.picking_markers[0].distance
                    self.state = 'FIND_PICKING_MARKER_2'
                else:
                    self.publish_velocity(0.0, rotation_velocity)
            else:
                self.publish_velocity(0.0, 0.2)
            
        if self.state == 'FIND_PICKING_MARKER_2':
            self.send_vision_data("picking_markers", "")
            if self.picking_markers[1].exists:
                picking_bay2_bearing = math.degrees(self.picking_markers[1].bearing[1])
                kp = 0.01
                rotation_velocity = kp * picking_bay2_bearing
                rotation_velocity = max(min(rotation_velocity, 0.3), -0.3)
                if abs(picking_bay2_bearing) < 0.4:
                    self.publish_velocity(0.0, 0.0)
                    self.distance_pickingbay2 = self.picking_markers[1].distance
                    self.state = 'CALCULATE_AISLE_IMU'
                else:
                    self.publish_velocity(0.0, rotation_velocity)
            else:
                self.publish_velocity(0.0, 0.2)
                
        elif self.state == 'CALCULATE_AISLE_IMU':
            cos_D = (self.icking_bay_marker_distance**2 + self.distance_pickingbay2**2 - self.distance_pickingbay1**2) / (2 * self.distance_pickingbay2 * self.picking_bay_marker_distance)
            # Numerical stability check (rounding errors may push value slightly out of [-1, 1])
            cos_D = max(-1, min(1, cos_D))
            # Return angle in radians
            angleD = math.acos(cos_D)
            currentIMU = GET_YAW()  # Placeholder function to get current IMU heading in radians
            self.aisle_IMU = currentIMU - angleD - np.pi/2
            self.state = 'TURN_TO_IMU_VALUE'

        elif self.state == 'TURN_TO_IMU_VALUE':
            offset = np.pi/2
            target_heading = self.aisle_IMU + offset  # radians
            current_heading = GET_YAW()  # radians

            e_theta = apf.angle_wrap(target_heading - current_heading)

            kp = 0.5  # Adjust gain as needed
            rotation_velocity = kp * e_theta
            rotation_velocity = max(min(rotation_velocity, 0.3), -0.3)

            if abs(np.degrees(e_theta)) < 0.4:  # close enough to target
                self.publish_velocity(0.0, 0.0)
                current_heading = GET_YAW()
                self.aisle_IMU = current_heading - offset
                self.state = 'DONE'
            else:
                self.publish_velocity(0.0, rotation_velocity)

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
