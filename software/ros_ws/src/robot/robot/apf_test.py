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


SUCCESS_COUNT_THRESHOLD = 3  # Number of consecutive successes required

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

        self.success_count = 0
        
        
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


        self.aisle_distances = [0.95, 0.67, 0.35, 0]

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
        try:
            # self.get_logger().info(str(self.state))
            if self.state == 'START':
                self.aisle_index = None
                self.success_count = 0
                self.state = 'APF'

            elif self.state == 'APF':
                self.send_vision_data("aisleMarkers,obstacles,shelves", "")
                for i, marker in enumerate(self.aisle_markers):
                    if marker.exists:
                        self.aisle_index = i
                        break
                if self.aisle_index is None:
                    self.publish_velocity(0.0, 0.7)
                    return
                # aisleBearing = math.degrees((self.aisle_markers[self.aisle_index].bearing[1])/1000)
                aisleBearing = ((self.aisle_markers[self.aisle_index].bearing[1])/1000) - math.radians(10) 
                aisleDistance = (self.aisle_markers[self.aisle_index].distance)/100000
                if self.aisle_markers[self.aisle_index].exists:
                    if aisleDistance < ((self.aisle_distances[2]) + 0.15):
                        self.success_count += 1
                        if (self.success_count >= SUCCESS_COUNT_THRESHOLD):
                            self.publish_velocity(0.0, 0.0)
                            self.state = 'DONE'
                    goal = [aisleDistance, aisleBearing]
                    # print(goal)
                    all_obstacles = []
                    for i, group in enumerate(self.obstacles):
                        # all_obstacles.append(((group.distance)/100000, (group.bearing[0])/1000))  # merge lists
                        all_obstacles.append(((group.distance)/100000, (group.bearing[1])/1000))  # merge lists

                        # all_obstacles.append(((group.distance)/100000, (group.bearing[2])/1000))  # merge lists

                    for i, group in enumerate(self.shelves):
                            # all_obstacles.append((0.5, (group.bearing[0])/1000))  # merge lists
                            all_obstacles.append((0.5, (group.bearing[1])/1000))  # merge lists

                            # all_obstacles.append((0.5/100000, (group.bearing[2])/1000))  # merge lists


                    U_rep = apf.repulsiveField(all_obstacles, self.phi)
                    U_att = apf.attractiveField(goal, self.phi)
                    # print(all_obstacles)
                    best_bearing = apf.bestBearing(U_att, U_rep, self.phi)
                    # print(best_bearing)

                    if best_bearing is not None:
                        theta = 0.0
                        e_theta = apf.angle_wrap(best_bearing - theta)

                        k_omega = 0.3
                        v_max = 0.2
                        sigma = np.radians(40)

                        omega = -(k_omega * e_theta)
                        v = v_max * np.exp(-(e_theta**2) / (2*sigma**2))
                        self.publish_velocity(v, omega)
                        # print("(" + str(v) + ", " + str(omega) + ")")
                        # print(v)
                    else:
                        self.publish_velocity(0.0, 0.4)
                        print("No valid bearing found, rotating...")
                    print(aisleDistance)

            elif self.state == 'DONE':
                self.publish_velocity(0.0, 0.0)
                self.get_logger().info("Task complete. Standing by...")
        # ---------------------------------------------
        except KeyboardInterrupt as e:
            self.publish_velocity(0.0, 0.0)
            self.get_logger().error(f"Error in state machine: {e}")

def main():
    rclpy.init()
    navigation_node = Navigation()
    rclpy.spin(navigation_node)
    navigation_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()