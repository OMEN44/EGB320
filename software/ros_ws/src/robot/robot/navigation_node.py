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

# ------------------- CONFIG --------------------
shelf_to_aisle = {
    "0": ["1", np.pi/2],
    "1": ["1", -np.pi/2],
    "2": ["2", np.pi/2],
    "3": ["2", -np.pi/2],
    "4": ["3", np.pi/2],
    "5": ["3", -np.pi/2]   # fixed duplicate key
}

obstacle_width = 0.15  # meters

shelf_distance_marker = {"1": 1, "2": 1, "3": 0.42, "4": 2}
aisle_distance_wall = {"1": [0.2, np.pi/2], "2": [0.8, -np.pi/2], "3": [0.2, -np.pi/2]}
shelfID = 1.3  # Example shelf ID
# ------------------------------------------------


class Navigation(Node):
    def __init__(self):
        super().__init__('navigation_node')
        self.get_logger().info('Navigation node has been started.')

        # Publishers
        self.pipeline_pub = self.create_publisher(String, '/pipeline_filters', 10)
        self.target_item_pub = self.create_publisher(String, '/target_item', 10)
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

        self.obstacles = []
        self.aisle_markers = []
        self.shelves = []
        self.picking_Station = []

        self.pois = []

        self.phi = np.linspace(-np.pi, np.pi, 360)  # Shared angular grid for fields

        self.arm_status = False  

        self.zone_dist_aisle_marker = 1.24

    # --------------------------- Get distance measurements ---------------------------
    def get_measurements(self, shelfID):
        # Return aisle index, shelf marker distance, and orientation.
        first, second = str(shelfID).split(".")
        aisle_id, shelfOrientation = shelf_to_aisle[first]
        aisle_index = int(aisle_id) - 1
        shelfMarkerDistance = shelf_distance_marker[second]
        return aisle_index, shelfMarkerDistance, shelfOrientation

    # --------------------------- Publish to vision (objects needed to be detected, target item/picking bay) ---------------------------
    def send_vision_data(self, pipeline_string, target_item_data):
        pipeline_msg = String()
        pipeline_msg.data = pipeline_string
        self.pipeline_pub.publish(pipeline_msg)

        target_item_msg = String()
        target_item_msg.data = target_item_data
        self.target_item_pub.publish(target_item_msg)
        self.get_logger().info("Sent pipeline and target item data to vision...")

    # --------------------------- Robot movement and actions publisher ---------------------------
    def publish_velocity(self, forward_vel, angular_vel):
        twist_msg = Twist()
        twist_msg.linear.x = forward_vel
        twist_msg.angular.z = angular_vel
        self.velocities_pub.publish(twist_msg)
        self.get_logger().info("Twist message sent to mobility...")

    def publish_collection(self, collection_command):
        # Placeholder: implement gripper or collection action
        collection_msg = Int32()
        collection_msg.data = collection_command
        self.collection_pub.publish(collection_msg)
        # self.get_logger().info("Command sent to collection...")

    # --------------------------- Point of Interest callback ---------------------------
    def poi_callback(self, msg):
        self.pois = []
        for item in msg.pois:
            self.pois.append((item.name, item.type, ((item.distance)/100000), [((item.bearing[0])/1000),((item.bearing[1])/1000), ((item.bearing[2])/1000)]))

    def arm_status_callback(self, msg):
        self.arm_status = msg.data

    def filter_poi(self, poi_type):
        
        listOfPois = []
        for poi in self.pois:
            if poi_type == poi[0]:
                listOfPois.append(poi)

        return listOfPois

    # ---------------- STATE MACHINE --------------
    def state_machine(self):
        print(self.state)
        if self.state == 'START':
            self.aisle_index, self.shelfMarkerDistance, self.shelfOrientation = self.get_measurements(shelfID)
            self.get_logger().info(f"Target shelf {shelfID}, aisle {self.aisle_index+1}")
            self.state = 'DRIVE_INTO_AISLE'
            
        elif self.state == 'TURN_CALIBRATION':
            calibration_turn_speed = 0.14
            self.send_vision_data("isleMarkers,shelves,pickingStation", "")
            self.aisle_markers = self.filter_poi("isleMarkers")
            self.shelves = self.filter_poi("shelf")
            self.picking_Station = self.filter_poi("pickingStation")
            if len(self.aisle_markers) != 0:
                self.publish_velocity(0.0, 0.0)
                for i, row in self.aisle_markers:
                    if row is not None and len(row) > 0:
                        row_index = i
                self.state = 'CALIBRATION_AISLE_MARKER'
            if len(self.picking_station) != 0:
                self.publish_velocity(0.0, 0.0)
                self.state = 'CALIBRATION_PICKING_STATION'
            elif len(self.shelves) != 0:
                self.publish_velocity(0.0, 0.0)
                self.state = 'CALIBRATION_SHELF'
            else:
                self.publish_velocity(0.0, calibration_turn_speed)

        elif self.state == 'CALIBRATION_AISLE_MARKER':
            self.send_vision_data("isleMarkers,shelves,obstacles", "")
            self.obstacles = self.filter_poi("obstacle")
            self.aisle_markers = self.filter_poi("isleMarkers")
            self.shelves = self.filter_poi("shelf")
            if len(self.aisle_markers) == 0:
                return
            distance_to_marker = self.aisle_markers[row_index][2]
            marker_bearing = self.aisle_markers[row_index][3]
            all_obstacles = []
            for group in (self.obstacles, self.shelves):
                all_obstacles.extend([(obj.distance, obj.bearing) for obj in group])
            error = distance_to_marker - self.zone_dist_aisle_marker
            U_rep = apf.repulsiveField(all_obstacles, self.phi)
            U_att = apf.attractiveField((distance_to_marker, marker_bearing), self.phi)
            best_bearing = apf.bestBearing(U_att, U_rep, self.phi)
            if best_bearing is not None:
                theta = 0.0  # if you have a robot heading API, replace this with it
                e_theta = apf.angle_wrap(best_bearing - theta)
                k_omega = 0.3
                v_max = 0.1
                sigma = np.radians(30)
                omega = k_omega * e_theta
                v = v_max * np.exp(-(e_theta**2) / (2*sigma**2))
                self.publish_velocity(v, omega)
            else:
                # No valid direction -> gentle spin to search
                self.publish_velocity(0.0, 0.15)
            if abs(error) < 0.01:
                self.publish_velocity(0.0, 0.0)
                self.state = 'FIND_PICKING_BAY1'
                self.get_logger().info("Front of aisle...")

        elif self.state == 'FIND_PICKING_BAY1':
            calibration_turn_speed = 0.14
            self.send_vision_data("isleMarkers", "")
            self.aisle_markers = self.filter_poi("isleMarkers")
            if len(self.aisle_markers) == 0:
                self.publish_velocity(0.0, calibration_turn_speed)
            else:
                if self.aisle_markers[0] is not None:
                    self.state == 'GET_PICKING_BAY1_DIST'
                else:
                    self.publish_velocity(0.0, calibration_turn_speed)
        
        elif self.state == 'GET_PICKING_BAY1_DIST':
            self.send_vision_data("pickingStation", "")
            self.picking_Station = self.filter_poi("pickingStation")
            picking_station_bearing = math.degrees(self.picking_Station[0][1])
            kp = 0.01
            rotation_velocity = kp * picking_station_bearing 
            rotation_velocity = max(min(rotation_velocity, 0.3), -0.3)
            if abs(picking_station_bearing) < 0.5:
                distance_picking_bay_1 = self.picking_Station[0][0]
                self.state = 'FIND_PICKING_BAY2'

        elif self.state == 'FIND_PICKING_BAY2':
            calibration_turn_speed = 0.14
            self.send_vision_data("isleMarkers", "")
            self.aisle_markers = self.filter_poi("isleMarkers")
            if len(self.aisle_markers) == 0:
                self.publish_velocity(0.0, calibration_turn_speed)
            else:
                if self.aisle_markers[1] is not None:
                    self.state == 'GET_PICKING_BAY1_DIST'
                else:
                    self.publish_velocity(0.0, calibration_turn_speed)

        elif self.state == 'GET_PICKING_BAY2_DIST':
            self.send_vision_data("pickingStation", "")
            self.picking_Station = self.filter_poi("pickingStation")
            picking_station_bearing = math.degrees(self.picking_Station[1][1])
            kp = 0.01
            rotation_velocity = kp * picking_station_bearing 
            rotation_velocity = max(min(rotation_velocity, 0.3), -0.3)
            if abs(picking_station_bearing) < 0.5:
                distance_picking_bay_2 = self.picking_Station[1][0]
                self.state = 'CALCULATE_AISLE_IMU'

        elif self.state == 'CALCULATE_AISLE_IMU':
            picking_bay_marker_distance = 0.29
            cos_D = (picking_bay_marker_distance**2 + distance_picking_bay_2**2 - distance_picking_bay_1**2) / (2 * distance_picking_bay_2 * distance_picking_bay_1)
    
            # Numerical stability check (rounding errors may push value slightly out of [-1, 1])
            cos_D = max(-1, min(1, cos_D))
            
            # Return angle in radians
            angleD = math.acos(cos_D)
            # currentIMU = bot.robotPose[5]
            # aisleIMU = currentIMU - angleD - np.pi/2
                      



        elif self.state == 'DRIVE_INTO_AISLE':
            self.send_vision_data("isleMarkers,shelves", "")
            # self.objects = self.filter_poi("obstacle")
            self.aisle_markers = self.filter_poi("isleMarkers")
            self.shelves = self.filter_poi("shelf")
            # print(self.aisle_markers)
            if len(self.aisle_markers) == 0:
                # print('NO MARKERS')
                return
            # print('YAY')
            distance_to_marker = self.aisle_markers[0][2]
            marker_bearing = self.aisle_markers[0][3]
            all_obstacles = []
            for group in (self.objects, self.shelves):
                all_obstacles.extend([(obj.distance, obj.bearing) for obj in group])
            error = distance_to_marker - self.shelfMarkerDistance
            U_rep = apf.repulsiveField(all_obstacles, self.phi)
            U_att = apf.attractiveField((distance_to_marker, marker_bearing), self.phi)
            best_bearing = apf.bestBearing(U_att, U_rep, self.phi)
            if best_bearing is not None:
                theta = 0.0  # if you have a robot heading API, replace this with it
                e_theta = apf.angle_wrap(best_bearing - theta)
                k_omega = 0.3
                v_max = 0.1
                sigma = np.radians(30)
                omega = k_omega * e_theta
                v = v_max * np.exp(-(e_theta**2) / (2*sigma**2))
                self.publish_velocity(v, omega)
            else:
                # No valid direction -> gentle spin to search
                self.publish_velocity(0.0, 0.15)
            if abs(error) < 0.01:
                self.publish_velocity(0.0, 0.0)
                self.state = 'LIFT_ARM'
                self.get_logger().info("Lifting Arm...")
            
        elif self.state == 'LIFT_ARM':
            self.publish_collection(3) # Command to lift arm
            if self.arm_status == True: # Assuming arm_status is updated via a subscriber
                self.state = False
                time_turn = time.time()
                self.state = 'TURN_TO_SHELF'
                self.get_logger().info("Turning to shelf...")


        elif self.state == 'TURN_TO_SHELF':
            turn_dir = (self.shelf_orientation < 0.0)
            if turn_dir:
                omega = 0.2
            else: 
                omega = -0.2
            
            turning_duration = abs(self.shelfOrientation) / omega
            if (time.time() - time_turn) > turning_duration:
                self.publish_velocity(0.0, 0.0)
                self.state = 'ALIGN_TO_SHELF'
            else:
                self.publish_velocity(0.0, omega)

        elif self.state == 'ALIGN_TO_SHELF':
            self.send_vision_data("shelves", "")
            self.shelves = self.filter_poi("shelves")
            if self.shelves:
                left_most_shelf = self.shelves[0]["bearing"][0]
                right_most_shelf = self.shelves[0]["bearing"][2]
                error = left_most_shelf - right_most_shelf

                kp = 0.4
                omega = max(min(kp*error, 0.3), -0.3)

                if abs(error) < 0.01:
                    self.publish_velocity(0.0, 0.0)
                    self.get_logger().info("Aligned with shelf...")
                    time_to_shelf = time.time()
                    self.state = 'DRIVE_TO_SHELF'
                else:
                    self.publish_velocity(0.0, omega)
            else:
                self.state = 'DRIVE_INTO_AISLE'           
                 

        elif self.state == 'DRIVE_TO_SHELF':
            if (time.time() - time_to_shelf) > 3.0:
                self.publish_velocity(0.0, 0.0)
                self.state = 'DROP_ITEM'
                return
            else: 
                self.publish_velocity(0.01, 0.0)

        elif self.state == 'DROP_ITEM':
            self.publish_collection(4) # Command to drop item
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
