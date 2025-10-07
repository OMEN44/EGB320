import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32 # Import the Int32 message type
from std_msgs.msg import Bool
from robot_interfaces.msg import PoiGroup
import time

import robot.navigation.apf as apf
import robot.navigation.imu as imu

import numpy as np
import math
import random

# ------------------- CONFIG --------------------
shelf_to_aisle = {
    "0": ["1", np.pi/2],
    "1": ["1", -np.pi/2],
    "2": ["2", np.pi/2],
    "3": ["2", -np.pi/2],
    "4": ["3", np.pi/2],
    "5": ["3", -np.pi/2]   
}

pickingbay_distance_wall = {"1":0.75, "2":0.45, "3":0.1}
shelf_distance_marker = {"1":0.93, "2":0.65, "3":0.42, "4":0.15}
aisle_distance_wall = {"1":[0.2, np.pi/2], "2":[0.8, -np.pi/2], "3":[0.2, -np.pi/2]}
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

        self.shelves = []
        self.items = []
        self.obstacles = []
        self.aisle_markers = []
        self.picking_stations = []

        self.pois = []

        self.phi = np.linspace(-np.pi, np.pi, 360)  # Shared angular grid for fields

        self.arm_status = False  

        self.zone_dist_aisle_marker = 1.24

        self.deliveries = []
        for picking_bay in [1, 2, 3]:
            shelf_first = random.randint(0, 5)   # first number 0–5
            shelf_second = random.randint(1, 4)  # second number 1–4
            shelf_id = float(f"{shelf_first}.{shelf_second}")  # combine into format like 1.4
            self.deliveries.append((picking_bay, shelf_id))

        # Unpack into individual variables
        self.delivery_one, self.delivery_two, self.delivery_three = self.deliveries

        # Example deliveries: (picking bay number, shelf id)
        # deliveries = [(3, 0.3), (2, 0.3), (1, 5.4)]

        self.deliveryNo = 0

    # --------------------------- Get distance measurements ---------------------------
    # --------------------------- Get measurements for a delivery ---------------------------
    def get_delivery_measurements(self, deliveries, delivery_no):
        """
        Returns all relevant measurements for a given delivery:
        picking bay index, aisle id, picking bay wall distance,
        aisle wall distance, shelf marker distance, shelf orientation, aisle orientation
        """
        picking_bay, shelf = deliveries[delivery_no]

        # Array index for picking bay
        picking_bay_index = int(picking_bay) - 1

        # Split shelf ID into first (aisle) and second (shelf distance key)
        first, second = str(shelf).split(".")

        # Look up picking bay wall distance
        picking_bay_wall_distance = aisle_distance_wall[str(picking_bay)][0]

        # Shelf → Aisle mapping
        aisle_id, shelf_orientation = shelf_to_aisle[first]

        # Aisle wall distance and orientation
        aisle_wall_distance, aisle_orientation = aisle_distance_wall[aisle_id]

        aisle_id = int(aisle_id) - 1
        # Shelf marker distance (based on the second digit)
        shelf_marker_distance = shelf_distance_marker[second]

        return (
            picking_bay_index,
            aisle_id,
            picking_bay_wall_distance,
            aisle_wall_distance,
            shelf_marker_distance,
            shelf_orientation,
            aisle_orientation
        )


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

    def publish_collection(self, collection_command):
        # Placeholder: implement gripper or collection action
        collection_msg = Int32()
        collection_msg.data = collection_command
        self.collection_pub.publish(collection_msg)
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

    def move_to_marker_apf(self, target_marker, target_distance, next_state):
        """
        Moves robot to a target marker using APF.
        Stops robot and switches to next_state when within target_distance.
        """
        if target_marker is None:
            # If marker is not visible, gently rotate to search
            self.publish_velocity(0.0, 0.15)
            return

        distance_to_marker = target_marker[0]
        middle_bearing = target_marker[1][1]  # middle bearing only

        # Build obstacle list (middle bearings only)
        all_obstacles = []
        for group in (self.obstacles, self.shelves):
            for obj in group:
                all_obstacles.append((obj[0], obj[1][1]))  # distance, middle bearing

        error = distance_to_marker - target_distance

        # Calculate APF
        U_rep = apf.repulsiveField(all_obstacles, self.phi)
        U_att = apf.attractiveField((distance_to_marker, middle_bearing), self.phi)
        best_bearing = apf.bestBearing(U_att, U_rep, self.phi)

        if best_bearing is not None:
            theta = 0.0  # replace with IMU heading if available
            e_theta = apf.angle_wrap(best_bearing - theta)
            k_omega = 0.3
            v_max = 0.1
            sigma = np.radians(30)
            omega = k_omega * e_theta
            v = v_max * np.exp(-(e_theta**2) / (2*sigma**2))
            self.publish_velocity(v, omega)
        else:
            # Gentle spin if no valid direction
            self.publish_velocity(0.0, 0.15)

        # Stop and switch state if within tolerance
        if abs(error) < 0.01:
            self.publish_velocity(0.0, 0.0)
            self.state = next_state
            self.get_logger().info(f"Reached target marker. Switching to {next_state}.")



    # ---------------- STATE MACHINE --------------
    def state_machine(self):
        picking_bay_index = None
        aisle_id = None
        picking_bay_wall_distance = None
        aisle_wall_distance = None
        shelf_marker_distance = None
        shelf_orientation = None
        aisle_orientation = None
        self.get_logger().info(self.state)
        self.shelves = self.pois["shelves"]
        self.items = self.pois["items"]
        self.obstacles = self.pois["obstacles"]
        self.aisle_markers = self.pois["islemarkers"]
        self.picking_stations = self.pois["pickingstations"]
        
        # print(self.state)
        # if self.state == 'START':
        #     self.aisle_index, self.shelfMarkerDistance, self.shelfOrientation = self.get_measurements(shelfID)
        #     self.get_logger().info(f"Target shelf {shelfID}, aisle {self.aisle_index+1}")
        #     self.state = 'TURN_CALIBRATION'

        if self.state == 'START':
            self.state = 'DRIVE_FORWARD_WITH_APF'

        elif self.state == 'DRIVE_FORWARD_WITH_APF':
            imu_yaw = imu.getYaw()
            self.send_vision_data("shelves,obstacles", "")
            self.shelves = self.filter_poi("shelves")
            self.obstacles= self.filter_poi("obstacles")
            all_obstacles = []

            for group in (self.obstacles, self.shelves):
                if group:
                    all_obstacles.extend(group)

            # --- 2. Determine reference bearing ---
            # Select shelf or picking bay bearings (whichever is detected)
            bearings = []
            # if self.shelves:
            #     bearings.extend([b for _, b in self.shelfRB])
            if self.shelves:
                bearings = self.shelves[1]
                ref_bearing = max(bearings)  # rightmost

                # Offset the bearing by a few degrees (adjust sign & value as needed)
                offset_deg = 10.0
                offset = np.radians(offset_deg)
                goal_bearing = ref_bearing + offset
            
            else:
                # Fallback: straight ahead
                goal_bearing = imu_yaw

            # theta = abs(ref_bearing - bearings[1])
            # --- 3. Attractive field goal (bearing only) ---
            goal_distance = self.shelves[0]  # Arbitrary far distance to form the attractive vector
            goal = [goal_distance, goal_bearing]

            self.move_to_marker_apf(goal, target_distance=aisle_wall_distance, next_state='DONE')

        elif self.state == 'DONE':
            self.get_logger().info("Done")


def main():
    rclpy.init()
    navigation_node = Navigation()
    rclpy.spin(navigation_node)
    navigation_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
