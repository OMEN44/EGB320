import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32 # Import the Int32 message type
from std_msgs.msg import Bool
from std_msgs.msg import Float64MultiArray
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

        self.ref_bearing = None

        # Publishers
        self.pipeline_pub = self.create_publisher(String, '/pipeline_filters', 10)
        self.target_item_pub = self.create_publisher(String, '/target_item', 10)
        self.arm_action_pub = self.create_publisher(Int32, '/arm_action', 10)
        self.gripper_action_pub = self.create_publisher(Bool, '/gripper_action', 10)
        self.velocities_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.collection_pub = self.create_publisher(Int32, '/collection_action', 10)

        # Subscribers
        self.imu_sum = self.create_subscription(Float64MultiArray, "/imu", self.imu_callback, 10)
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

        self.linear_accel = None
        self.yaw = None
        self.angular_rot = None 

    # --------------------------- Publish to vision (objects needed to be detected, target item/picking bay) ---------------------------
    def imu_callback (self, msg):
        self.linear_accel = msg.data[0]
        self.yaw = msg.data[1]
        self.angular_rot = msg.data[1] 
    
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
            self.aisle_index = None
            self.state = 'APF'

        elif self.state == 'APF':
            self.send_vision_data("shelves,obstacles,pickingStation", "")
            
            # --- 1. Collect obstacles ---
            all_obstacles = []
            for group in self.obstacles:
                dist_m = group.distance / 1000.0
                bearing_rad = group.bearing[1] / 1000.0  # ensure radians
                all_obstacles.append((dist_m, bearing_rad))

            for group in self.picking_station:
                bearing_rad = group.bearing[1] / 1000.0  # ensure radians
                
                all_obstacles.append((dist_m, bearing_rad))

            self.ref_bearing = None
            for shelf in self.shelves:
                dist_m = 0.5
                bearing_rad = shelf.bearing[1] / 1000.0
                # all_obstacles.append((dist_m, bearing_rad))
                if self.ref_bearing is None:
                    self.ref_bearing = shelf.bearing[0] / 1000.0


            if len(self.shelves) == 0:
                self.get_logger().info("No shelves detected — rotating to search.")
                self.publish_velocity(0.0, -0.60)
                return

            # --- 2. Compute goal ---
            offset = np.radians(6.0)
            goal_bearing = self.ref_bearing - offset
            goal_distance = 1
            goal = [goal_distance, goal_bearing]

            # --- 3. Compute fields ---
            U_rep = apf.repulsiveField(all_obstacles, self.phi)
            U_att = apf.attractiveField(goal, self.phi)
            best_bearing = apf.bestBearing(U_att, U_rep, self.phi)

            # --- 4. Motion command ---
            if best_bearing is not None:
                theta = 0.0
                e_theta = apf.angle_wrap(best_bearing - theta)

                k_omega = 0.3
                v_max = 0.25
                sigma = np.radians(40)

                omega = k_omega * e_theta
                v = v_max * np.exp(-(e_theta**2) / (2 * sigma**2))
                self.publish_velocity(v, omega)
                print(f"Driving (v={v:.2f}, ω={omega:.2f})")
            else:
                print("No valid bearing found — rotating slowly.")
                self.publish_velocity(0.0, 0.15)

            # --- 5. Distance check ---
            tof_distance = 0.7  # Placeholder
            if tof_distance < 0.6:
                self.publish_velocity(0.0, 0.0)
                self.state = 'DONE'


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
