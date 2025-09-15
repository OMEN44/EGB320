import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import numpy as np
import math

import gpiozero
import time

shelf_to_aisle = {
    "0": ["1", np.pi/2],
    "1": ["1", -np.pi/2],
    "2": ["2", np.pi/2],
    "3": ["2", -np.pi/2],
    "4": ["3", np.pi/2],
    "5": ["3", -np.pi/2]   # fixed duplicate key
}

# pickingbay_distance_wall = {"1":0.75, "2":0.45, "3":0.15}
shelf_distance_marker = {"1":1, "2":1, "3":0.42, "4":2}
state = 'START'
shelfID = 1.3  # Example shelf ID

class Navigation(Node):   

    def getMeasurements(self, shelfID):
        # Split shelf into two parts
        first, second = str(shelfID).split(".")

        # Shelf → Aisle mapping
        aisle_id, shelfOrientation = shelf_to_aisle[first]
        aisle_index = int(aisle_id) - 1

        # Shelf marker distance (based on the second digit)
        shelfMarkerDistance = shelf_distance_marker[second]

        return aisle_index, shelfMarkerDistance, shelfOrientation

    def __init__(self):
        super().__init__('navigation_node')
        self.get_logger().info('Navigation node has been started.')
        # For sending pipeline filter data
        self.pipeline = self.create_publisher(String, '/pipeline_filters', 10)
        # For sending target item data
        self.targetItem = self.create_publisher(String, '/target_item', 10)
        self.velocities = self.create_publisher(Twist, '/mobility_twist', 10)
        
    def send_vision_data(self, pipeline_string_list, target_item_data):
        # Pipeline Options:
        # isleMarkers
        # pickingStation
        # colourMask
        # items
        # obstacles
        # shelves
        pipeline_msg = String()
        pipeline_msg.data = pipeline_string_list
        self.pipeline.publish(pipeline_msg)
            
        # Item Options:
        # Bowl
        # Coffee Cup
        # Robot Oil Bottle
        # Rubiks Cube
        # Soccer Ball
        # Wheet Bots
        target_item_msg = String()
        target_item_msg.data = target_item_data
        self.targetItem.publish(target_item_data)) # item name,isle number
           
    def send_velocity_messages(self, forward_vel, angular_vel):
        twist_msg = Twist()
        twist_msg.linear.x = forward_vel     # Forward/backward velocity
        twist_msg.linear.y = 0.0             # Typically 0 unless strafing
        twist_msg.linear.z = 0.0
        twist_msg.angular.x = 0.0
        twist_msg.angular.y = 0.0
        twist_msg.angular.z = angular_vel    # Rotation around z-axis
            
        self.velocities_pub.publish(twist_msg)
        
    def state_machine(self):
        if self.state == 'START':
            aisle_index, shelfMarkerDistance, shelfOrientation = getMeasurements(shelfID)
            state = 'DRIVE_TO_AISLE'
            
        
        elif self.state == 'DRIVE_INTO_AISLE':
            send_vision_data("isleMarkers, obstacles, shelves", "Bowl, 1")
            error = distance_to_aisle - shelf_distance
            if (error < 0.01):
                self.publish_velocity(0.0, 0.0)
                self.state = 'TURN_TO_SHELF'
                self.get_logger("Turning to shelf...")

            else: v = 0.05:
                self.publish_velocity(v, 0.0)
                
        elif self.state == 'TURN_TO_SHELF':
            e_theta = self.target_heading - self.current_heading
            # wrap to [-pi, pi]
            e_theta = (e_theta + math.pi) % (2 * math.pi) - math.pi

            if abs(e_theta) < math.radians(1):
                self.publish_velocity(0.0, 0.0)
                self.state = 'DROP_ITEM'
                self.get_logger().info("Facing shelf, dropping item...")
            else:
                kp = 0.3
                omega = max(min(kp * e_theta, 0.3), -0.3)
                self.publish_velocity(0.0, omega)

        elif self.state == 'DROP_ITEM':
            self.publish_collection()
            # Collection feedback
            #if (collection_feedback == True):
            #    self.state = 'DONE'

        elif self.state == 'DONE':
            self.publish_velocity(0.0, 0.0)
            # Node can optionally shut down or wait



def main():
    rclpy.init()
    navigation_node = Navigation()
    rclpy.spin(navigation_node)
    navigation_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()