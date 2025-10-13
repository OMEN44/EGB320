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
        self.arm_action_pub = self.create_publisher(Int32, '/arm_action', 10)
        self.gripper_action_pub = self.create_publisher(Bool, '/gripper_action', 10)
        self.velocities_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.collection_pub = self.create_publisher(Int32, '/collection_action', 10)

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
        self.picking_station = []
        self.picking_markers = []


        self.pois = []

        self.phi = np.linspace(-np.pi, np.pi, 360)  # Shared angular grid for fields

        self.arm_status = False  

        self.zone_dist_aisle_marker = 1.24

        self.picking_bay_index = None
        self.aisle_id = None
        self.picking_bay_wall_distance = None
        self.aisle_wall_distance = None
        self.shelf_marker_distance = None
        self.shelf_orientation = None
        self.aisle_orientation = None

        self.picking_marker_index = None

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
        self.pois = msg.pois

    def arm_status_callback(self, msg):
        self.arm_status = msg.data

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
            theta = 0.0  # Local frame; use IMU if global coordinates
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
            
    def get_arrays(self):
        self.shelves = self.pois["shelves"]
        self.items = self.pois["items"]
        self.obstacles = self.pois["obstacles"]
        self.aisle_markers = self.pois["aislemMarkers"]
        self.picking_station = self.pois["pickingStation"]
        self.picking_markers = self.pois["pickingMarkers"]


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
        if self.state == 'TURN_CALIBRATION':
            calibration_turn_speed = 0.14

            # Initialize variables once
            if not hasattr(self, 'calibration_complete'):
                self.calibration_complete = False
                self.rotation_start_yaw = imu.get_yaw()  # store starting angle
                self.rotation_accumulated = 0.0
                self.last_yaw = self.rotation_start_yaw

            # Continuously update yaw info
            current_yaw = imu.get_yaw()
            delta_yaw = (current_yaw - self.last_yaw + np.pi) % (2 * np.pi) - np.pi  # shortest signed diff
            self.rotation_accumulated += abs(delta_yaw)
            self.last_yaw = current_yaw

            # Always gather vision data
            self.send_vision_data("aisleMarkers,pickingMarkers,shelves", "")
            self.get_arrays()

            # ---------------------------
            # Phase 1: First 360° — look only for aisle/picking markers
            # ---------------------------
            if not self.calibration_complete:
                self.publish_velocity(0.0, calibration_turn_speed)

                if len(self.aisle_markers) > 0:
                    self.publish_velocity(0.0, 0.0)
                    for i, marker in enumerate(self.aisle_markers):
                        if marker.exists:
                            self.row_index = i
                    self.state = 'CALIBRATION_AISLE_MARKER'

                elif len(self.picking_markers) > 0:
                    self.publish_velocity(0.0, 0.0)
                    for i, marker in enumerate(self.picking_markers):
                        if marker.exists:
                            self.picking_marker_index = i
                    self.state = 'CALIBRATION_PICKING_STATION'

                # Check if full rotation done
                elif self.rotation_accumulated >= 2 * np.pi:
                    self.publish_velocity(0.0, 0.0)
                    self.calibration_complete = True
                    self.rotation_start_yaw = imu.get_yaw()
                    self.rotation_accumulated = 0.0
                    self.last_yaw = self.rotation_start_yaw

            # ---------------------------
            # Phase 2: Second 360° — now include shelves
            # ---------------------------
            else:
                self.publish_velocity(0.0, calibration_turn_speed)

                if len(self.aisle_markers) != 0:
                    self.publish_velocity(0.0, 0.0)
                    for i, marker in enumerate(self.aisle_markers):
                        if marker.exists is True:
                            self.row_index = i
                    self.state = 'CALIBRATION_AISLE_MARKER'

                elif len(self.picking_markers) != 0:
                    self.publish_velocity(0.0, 0.0)
                    for i, marker in enumerate(self.picking_markers):
                        if marker.exists is True:
                            self.picking_marker_index = i
                    self.state = 'CALIBRATION_PICKING_STATION'

                elif len(self.shelves) != 0:
                    self.publish_velocity(0.0, 0.0)
                    self.state = 'CALIBRATION_SHELF'

                # Optional: Stop if full rotation done and still found nothing
                elif self.rotation_accumulated >= 2 * np.pi:
                    self.publish_velocity(0.0, 0.0)
                    self.get_logger().info("No markers or shelves found after full rotation.")
                    # self.state = 'IDLE' or 'SEARCH_FAIL' or whatever your next fallback state is

        elif self.state == 'CALIBRATION_PICKING_STATION':
            self.send_vision_data("pickingMarkers,obstacles,shelves", "")
            self.get_arrays()

            # Look for Bay 1 marker
            target_marker = None
            if self.picking_markers[self.picking_marker_index].exists:
                target_marker = self.picking_markers[self.picking_marker_index]

            # Move using APF and switch state when target distance reached
            self.move_to_marker_apf(target_marker, target_distance=0.5, next_state='TURN_TO_AISLE_2')
            
        elif self.state == 'TURN_TO_AISLE_2':
            calibration_turn_speed = 0.14

            # Ask vision for picking stations
            self.send_vision_data("aisleMarkers", "")
            self.get_arrays()   
            target_marker = None
            if self.aisle_markers[1].exists is True:  # data = 1 means Bay 1 marker
                    target_marker = self.aisle_markers[1]
                    self.publish_velocity(0.0, calibration_turn_speed)

            # Align with centre bearing
            centre_bearing = target_marker[1][1]  # middle bearing
            kp = 0.8
            rotation_velocity = kp * centre_bearing
            rotation_velocity = max(min(rotation_velocity, 0.3), -0.3)

            if abs(centre_bearing) < 0.02:  # ~1 degree tolerance
                self.publish_velocity(0.0, 0.0)
                self.distance_picking_bay_1 = target_marker[2]  # distance
                self.state = 'CALIBRATION_AISLE_MARKER'
            else:
                self.publish_velocity(0.0, rotation_velocity)

        elif self.state == 'CALIBRATION_AISLE_MARKER':
                self.send_vision_data("aisleMarkers,shelves,obstacles", "")
                self.get_arrays()

                # Take first aisle marker as target
                target_marker = None
                for i, marker in enumerate(self.aisle_markers):
                    if self.aisle_markers[i].exists:
                        aisle_index = i
                        break
                
                target_marker = self.aisle_markers[aisle_index]

                # Move using APF and switch state when target distance reached
                self.move_to_marker_apf(target_marker, target_distance=self.zone_dist_aisle_marker, next_state='FIND_PICKING_BAY1')

        elif self.state == 'FIND_PICKING_BAY1':
            calibration_turn_speed = 0.14

            # Ask vision for picking stations
            self.send_vision_data("pickingMarkers", "")
            self.get_arrays()   
            # Look for marker #1
            target_marker = None
            if self.picking_markers[0].exists is True:  # data = 1 means Bay 1 marker
                target_marker = self.picking_markers[0]

            if target_marker:
                # Align with centre bearing
                centre_bearing = target_marker[1][1]  # middle bearing
                kp = 0.8
                rotation_velocity = kp * centre_bearing
                rotation_velocity = max(min(rotation_velocity, 0.3), -0.3)

                if abs(centre_bearing) < 0.02:  # ~1 degree tolerance
                    self.publish_velocity(0.0, 0.0)
                    self.distance_picking_bay_1 = target_marker[0]  # distance

                    self.get_logger().info(f"Picking Bay 1 found at {self.distance_picking_bay_1:.2f} m")
                    self.state = 'FIND_PICKING_BAY2'
            else:
                self.publish_velocity(0.0, calibration_turn_speed)

        # --------------------------------------------------------

        elif self.state == 'FIND_PICKING_BAY2':
            calibration_turn_speed = 0.14

            self.send_vision_data("pickingMarkers", "")
            self.get_arrays()
            # Look for marker #2
            target_marker = None
            if self.picking_markers[1].exists is True:  # data = 2 means Bay 2 marker
                target_marker = self.picking_markers[1]
            if target_marker:
                centre_bearing = target_marker[1][1]
                kp = 0.8
                rotation_velocity = kp * centre_bearing
                rotation_velocity = max(min(rotation_velocity, 0.3), -0.3)

                if abs(centre_bearing) < 0.02:
                    self.publish_velocity(0.0, 0.0)
                    self.distance_picking_bay_2 = target_marker[0]

                    self.get_logger().info(f"Picking Bay 2 found at {self.distance_picking_bay_2:.2f} m")
                    self.state = 'CALCULATE_AISLE_IMU'
            else:
                self.publish_velocity(0.0, calibration_turn_speed)

        # --------------------------------------------------------

        elif self.state == 'CALCULATE_AISLE_IMU':
            if self.distance_picking_bay_1 is not None and self.distance_picking_bay_2 is not None:
                cos_D = (self.picking_bay_marker_distance**2 + self.distance_picking_bay_2**2 - self.distance_picking_bay_1**2) / (2 * self.distance_picking_bay_2 * self.distance_picking_bay_1)
                cos_D = max(-1, min(1, cos_D))  # Clamp for numerical stability
                angleD = math.acos(cos_D)
                currentIMU = self.imu.get_yaw()
                self.aisle_IMU = currentIMU - angleD - np.pi/2
                self.get_logger().info(f"Aisle IMU calibrated to {math.degrees(self.aisle_IMU):.2f}°")
                self.state = 'TURN_READY_TO_PICK_UP'
            else:
                self.get_logger().info("Missing distance data for IMU calibration, retrying 'FIND_PICKING_BAY1'")
                self.state = 'FIND_PICKING_BAY1'  # Retry if data is missing
        
        elif self.state == 'TURN_READY_TO_PICK_UP':
            target_heading = self.aisle_IMU + math.pi/2
            current_heading = imu.getYaw()  # <-- replace with your IMU API
            e_theta = apf.angle_wrap(target_heading - current_heading)
            kp = 0.5  # Adjust gain as needed
            rotation_velocity = kp * e_theta
            rotation_velocity = max(min(rotation_velocity, 0.3), -0.3)

            if abs(np.degrees(e_theta)) < 0.4:  # close enough to target
                self.publish_velocity(0.0, 0.0)
                current_IMU = imu.getYaw()
                self.aisle_imu = current_IMU - math.pi/2 # Recalibrate aisle IMU
                self.state = 'GET_DELIVERY_DATA'
            else:
                self.publish_velocity(0.0, rotation_velocity)

        elif self.state == 'GET_DELIVERY_DATA':
            picking_bay_index, aisle_id, picking_bay_wall_distance, aisle_wall_distance, shelf_marker_distance, shelf_orientation, aisle_orientation = self.get_delivery_measurements(self.deliveries, self.delivery_no)
            self.state = 'DRIVE_TO_FRONT_OF_PICKING_STATION'

        elif self.state == 'DRIVE_TO_FRONT_OF_PICKING_STATION':
            all_obstacles = []

            for group in (self.obstacles, self.shelves):
                if group:
                    all_obstacles.extend(group)

            # --- 2. Determine reference bearing ---
            # Select shelf or picking bay bearings (whichever is detected)
            bearings = []
            # if self.shelves:
            #     bearings.extend([b for _, b in self.shelfRB])
            if self.picking_stations:
                bearings.extend([b for _, b in self.picking_stations])

                ref_bearing = max(bearings)  # rightmost

                # Offset the bearing by a few degrees (adjust sign & value as needed)
                offset_deg = 10.0
                offset = np.radians(offset_deg)
                goal_bearing = ref_bearing + offset
            
            else:
                # Fallback: straight ahead
                goal_bearing = imu.getYaw()

            # --- 3. Attractive field goal (bearing only) ---
            goal_distance = 2.0  # Arbitrary far distance to form the attractive vector
            goal = [goal_distance, goal_bearing]

            self.move_to_marker_apf(goal, target_distance=1.5, next_state='TURN_TO_PICKING_BAY')

        elif self.state == 'TURN_TO_PICKING_BAY':
            turn_speed = 0.14
            self.send_vision_data("pickingMarkers", "")
            self.get_arrays()
            # Look for marker #1
            target_marker = None
            if self.picking_markers[picking_bay_index].exists:
                target_marker = self.picking_markers[picking_bay_index]
                # Keep turning until marker found
            else:
                self.publish_velocity(0.0, turn_speed)

            # Align with centre bearing
            centre_bearing = target_marker[1][1]  # middle bearing
            kp = 0.8
            rotation_velocity = kp * centre_bearing
            rotation_velocity = max(min(rotation_velocity, 0.3), -0.3)

            if abs(centre_bearing) < 0.02:  # ~1 degree tolerance
                self.publish_velocity(0.0, 0.0)
                self.state = 'DRIVE_UP_PICKING_STATION'
            else:
                self.publish_velocity(0.0, rotation_velocity)

        elif self.state == 'DRIVE_UP_PICKING_STATION':
            self.send_vision_data("pickingMarkers,obstacles", "")
            self.get_arrays()

            # Look for current picking bay
            target_marker = None
            if self.picking_markers[picking_bay_index]:
                target_marker = self.picking_markers[picking_bay_index]

            # Move using APF and switch state when target distance reached
            self.move_to_marker_apf(target_marker, target_distance=0.2, next_state='ADJUST_TO_ITEM')
        
        elif self.state == 'ADJUST_TO_ITEM':
            #Something about camera and item and certain distance drive towards it
            self.send_vision_data("", "ITEM_NAME")
            self.state = 'COLLECT_ITEM'
        
        elif self.state == 'COLLECT_ITEM':          
            self.publish_collection(1) # Command to collect for arm (IT IS 1??) - yeah
            if self.arm_status == True: # Assuming arm_status is updated via a subscriber
                self.state = 'TURN_TOWARDS_AISLE'
        
        elif self.state == 'TURN_TOWARDS_AISLE':
            turning_speed = 0.14
            self.send_vision_data("aisleMarkers", "")
            self.get_arrays()
            if len(self.aisle_markers) != 0:
                for i, marker in enumerate(self.aisle_markers):
                    if marker.exists is True:
                        marker_index = i
                        break

                centre_bearing = self.aisle_markers[marker_index][1][1]  # middle bearing
                kp = 0.8
                rotation_velocity = kp * centre_bearing
                rotation_velocity = max(min(rotation_velocity, 0.3), -0.3)
                if abs(centre_bearing) < 0.02:  # ~1 degree tolerance
                    self.publish_velocity(0.0, 0.0)
                    self.state = 'DRIVE_OFF_PICKING_STATION'
                else:
                    self.publish_velocity(0.0, rotation_velocity)                
            else:
                self.publish_velocity(0.0, turning_speed)

        elif self.state == 'DRIVE_OFF_PICKING_STATION':
            self.send_vision_data("aisleMarkers,shelves,obstacles", "")
            self.get_arrays()

            # Take first aisle marker as target
            target_marker = self.aisle_markers[marker_index] if len(self.aisle_markers) > 0 else None

            # Move using APF and switch state when target distance reached
            self.move_to_marker_apf(target_marker, target_distance=self.zone_dist_aisle_marker, next_state='TURN_TO_SHELF_DIR')

        elif self.state == 'TURN_TO_SHELF_DIR':
            target_heading = self.aisle_IMU + aisle_orientation
            current_heading = imu.getYaw()  # <-- replace with your IMU API
            e_theta = apf.angle_wrap(target_heading - current_heading)
            kp = 0.5  # Adjust gain as needed
            rotation_velocity = kp * e_theta
            rotation_velocity = max(min(rotation_velocity, 0.3), -0.3)

            if abs(np.degrees(e_theta)) < 0.4:  # close enough to target
                self.publish_velocity(0.0, 0.0)
                current_IMU = imu.getYaw()
                self.aisle_imu = current_IMU  - aisle_orientation # Recalibrate aisle IMU
                self.state = 'DRIVE_TO_SHELF_ENTRANCE'
            else:
                self.publish_velocity(0.0, rotation_velocity)

        elif self.state == 'DRIVE_TO_SHELF_ENTRANCE':
            current_distance = None # Get TIME OF FLIGHT SENSOR WORKING
            target_distance = aisle_distance_wall 
            error = current_distance - target_distance

            # proportional control
            kp = 0.5   # tune as needed
            v = kp * error

            # clamp velocity so it doesn’t crawl too slowly or rush too fast
            v = max(min(v, 0.12), 0.03)  

            self.publish_velocity(v, 0.0)
            # print(f"Distance: {distance:.3f}, Error: {error:.3f}, v: {v:.3f}")

            # stop when within ±1 cm of 0.45 m
            if abs(error) <= 0.02:
                self.publish_velocity(0.0, 0.0)
                self.state = 'TURN_TO_DESIRED_AISLE'

        elif self.state == 'TURN_TO_DESIRED_AISLE':
            turn_speed = 0.14
            self.send_vision_data("islemarkers", "")
            self.get_arrays()

            
            target_marker = None
            if self.aisle_markers[aisle_id].exists is True:
                target_marker = self.aisle_markers[aisle_id]
                # Keep turning until marker found
            else:
                self.publish_velocity(0.0, turn_speed)

            # Align with centre bearing
            centre_bearing = target_marker[3][1]  # middle bearing
            kp = 0.8
            rotation_velocity = kp * centre_bearing
            rotation_velocity = max(min(rotation_velocity, 0.3), -0.3)

            if abs(centre_bearing) < 0.02:  # ~1 degree tolerance
                self.publish_velocity(0.0, 0.0)
                self.state = 'DRIVE_INTO_AISLE'
            else:
                self.publish_velocity(0.0, rotation_velocity)



        elif self.state == 'DRIVE_INTO_AISLE':
            self.send_vision_data("isleMarkers,shelves,obstacles", "")
            self.get_arrays()

            # Look for target aisle marker
            if self.aisle_markers[aisle_id].exists is True:
                target_marker = self.aisle_markers[aisle_id]                
            else:
                self.publish_velocity(0.0, turn_speed)


            # Move using APF and switch state when target distance reached
            self.move_to_marker_apf(
                target_marker,
                target_distance = shelf_marker_distance,
                next_state='LIFT_ARM'
            )
            
        elif self.state == 'LIFT_ARM':
            self.publish_collection(3) # Command to lift arm
            if self.arm_status == True: # Assuming arm_status is updated via a subscriber
                self.arm_status = False
                self.state = 'TURN_TO_SHELF'
                self.get_logger().info("Turning to shelf...")


        elif self.state == 'TURN_TO_SHELF':
            target_heading = self.aisle_IMU + shelf_orientation
            current_heading = imu.getYaw()  # <-- replace with your IMU API
            e_theta = apf.angle_wrap(target_heading - current_heading)
            kp = 0.5  # Adjust gain as needed
            rotation_velocity = kp * e_theta
            rotation_velocity = max(min(rotation_velocity, 0.3), -0.3)

            if abs(np.degrees(e_theta)) < 0.4:  # close enough to target
                self.publish_velocity(0.0, 0.0)
                current_IMU = imu.getYaw()
                self.aisle_imu = current_IMU - shelf_orientation # Recalibrate aisle IMU
                self.state = 'DRIVE_TO_SHELF'
            else:
                self.publish_velocity(0.0, rotation_velocity)          

        elif self.state == 'DRIVE_TO_SHELF':
            current_distance = self.tof_distance  # Time-of-Flight sensor
            forward_speed = 0.01
            distance_shelf_bay = 0.015
            error = abs(current_distance - distance_shelf_bay)
            if error < 0.01:
                self.publish_velocity(0.0, 0.0)
                self.state = 'DROP_ITEM'
                return
            else: 
                self.publish_velocity(forward_speed, 0.0)

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
