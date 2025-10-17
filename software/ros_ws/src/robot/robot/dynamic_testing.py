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

SUCCESS_THRESHOLD = 5 # Number of consecutive confirmations needed
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

        # Create timers
        self.imuCallback = self.create_timer(0.001, self.imu_callback)
        self.timer_ = self.create_timer(0.001, self.state_machine)

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

        self.send_to_web('Calibrating IMU...')
        while count < calibrationSteps:
            accel = self.imu.read_accel_data() # read the accelerometer [ms^-2]
            self.Ax_bias += accel["x"]

            gyro = self.imu.read_gyro_data()   # read the gyro [deg/s]
            self.Gz_bias += gyro["z"]
            count += 1

        self.Ax_bias /= calibrationSteps
        self.Gz_bias /= calibrationSteps

        self.target_angle = None
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

        self.return_zone_distance = 0.5
        self.turn_angle = 0

        # Tof
        self.tof_sensor = PiicoDev_VL53L1X()

        # Action queue
        # Each element has an action property and a list of parameters for that action
        self.action_queue = []
        self.action_index = -1 # Start before the first action

        self.aisle_marker_num = None

    def get_distance(self):
        return self.tof_sensor.read() / 1000

    def next(self):
        if self.action_index < len(self.action_queue) - 1:
            self.action_index += 1
            self.state = self.action_queue[self.action_index]['action']
            if self.state == 'TURN_DYNAMIC':
                self.prev_imu = self.rot_z
            if self.state == 'RETURN_ZONE_DYNAMIC':
                self.success_count = 0
                self.send_to_web(f"RESET SUCCESS COUNT")  
            self.send_to_web(f"{self.state}")
            if self.state == 'DYNAMIC_AISLE_MARKER':
                self.aisle_marker_num = None 

        else:
            self.state = 'DONE'
    
    def send_to_web(self, message):
        status = String()
        status.data = message
        self.nav_status_pub.publish(status)
    
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
        picking_bay_distances = [0.8, 0.45, 0.8]  # Distances to picking bays (1,2,3) in meters
        shelf_distances =  [0.95, 0.67, 0.47, 0]  # Distances to shelves (0-5) in meters
        aisle_distances = [[0.2, 85], [-1.15, -85], [-0.35, -85]]

        shelf_direction = [-85, 85, -85, 85, -85, 85]

        self.item_name = order['name'] 
        self.picking_marker_index = order['station']  # Assuming station numbers are 1-indexed
        # if int((order['shelf'])) == 0 or if int((order['shelf'])) == 0
        self.aisle_index = int((order['shelf'])/2)
        self.picking_bay_distance = picking_bay_distances[order['station']] 
        self.shelf_distance = shelf_distances[order['bay']]
        self.shelf_height = order['height']
        self.aisle_distance, self.aisle_turn_direction  = aisle_distances[self.aisle_index]
        self.shelf_turn_direction = shelf_direction[order['shelf']]
        self.nav_status_pub.publish(String(data=f"Distance: {self.aisle_distance:.2f} m, Aisle: {self.aisle_index}"))


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
            if len(self.order_queue) >= 1:
                self.aisle_index = None
                self.success_count = 0
                self.tof_distance = 10  # Initialize with a large distance

                # Create action queue based on order
                self.action_queue = [{"action": 'DYNAMIC_AISLE_MARKER', "params": [1.4]},
                                     {"action": 'TURN_DYNAMIC', "params": [-85]}]
                # self.action_queue = []

                
                self.prev_imu = self.rot_z
                # self.action_queue = [{"action": 'TURN_DYNAMIC', "params": [-85]}]
                for order in self.order_queue:
                    self.get_delivery_measurements(order)
                    # self.action_queue.append({"action": 'RETURN_ZONE_DYNAMIC', "params": [self.aisle_distance]})
                    # self.action_queue.append({"action": 'RETURN_ZONE_DYNAMIC', "params": [-1.05]})

                    self.action_queue.append({"action": 'RETURN_ZONE_DYNAMIC', "params": [self.picking_bay_distance]})
                    self.action_queue.append({"action": 'TURN_DYNAMIC', "params": [-85]})
                    self.action_queue.append({"action": 'FIND_PICKING_MARKER', "params": [self.picking_marker_index]})
                    self.action_queue.append({"action": 'DRIVE_UP_BAY', "params": []})
                    self.action_queue.append({"action": 'LEAVE_PICKING_BAY', "params": []})
                    self.action_queue.append({"action": 'TURN_DYNAMIC', "params": [self.aisle_turn_direction]})
                    self.action_queue.append({"action": 'RETURN_ZONE_DYNAMIC', "params": [self.aisle_distance]})
                    self.action_queue.append({"action": 'TURN_DYNAMIC', "params": [self.aisle_turn_direction]})
                    self.action_queue.append({"action": 'DYNAMIC_AISLE_MARKER', "params": [self.shelf_distance]})

                print(self.action_queue)
                self.next()
            else:
                self.send_to_web('Waiting for orders...')

        elif self.state == 'DYNAMIC_AISLE_MARKER':
            self.send_vision_data("aisleMarkers,obstacles,shelves", "")
            for i, marker in enumerate(self.aisle_markers):
                if marker.exists:
                    self.aisle_marker_num = i
                    break
            if self.aisle_marker_num is None:
                self.publish_velocity(0.0, 0.7)
                return
            # aisleBearing = math.degrees((self.aisle_markers[self.aisle_index].bearing[1])/1000)
            else:
                # self.nav_status_pub.publish(String(data=f"Distance: {aisleDistance:.2f} m"))
                if self.aisle_markers[self.aisle_marker_num].exists:
                    aisleBearing = ((self.aisle_markers[self.aisle_marker_num].bearing[1])/1000) 
                    aisleDistance = (self.aisle_markers[self.aisle_marker_num].distance)/100000
                    self.nav_status_pub.publish(String(data=f"Distance: {aisleDistance:.2f} m"))
                    if aisleDistance < (self.action_queue[self.action_index]['params'][0]):
                        self.success_count += 1
                        if (self.success_count >= SUCCESS_COUNT_THRESHOLD):
                            self.publish_velocity(0.0, 0.0)
                            # self.state = 'TURN_TOWARDS_PICKING_BAY_DIR'
                            self.next()
                            return
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
                        v_max = 0.25
                        sigma = np.radians(40)

                        omega = -(k_omega * e_theta)
                        v = v_max * np.exp(-(e_theta**2) / (2*sigma**2))
                        self.publish_velocity(v, omega)
                        # print("(" + str(v) + ", " + str(omega) + ")")
                        # print(v)
                else:
                    self.publish_velocity(0.0, 0.6)
                    print("No valid bearing found, rotating...")

        
        elif self.state == 'FIND_PICKING_MARKER':
            self.send_to_web("Finding aisle marker...")
            self.send_vision_data("pickingMarkers", "")
            time.sleep(0.001)
            if len(self.picking_markers) != 0:
                if self.picking_markers[self.picking_marker_index].exists:
                    kp = 0.08
                    markerBearing = np.degrees(self.picking_markers[self.picking_marker_index].bearing[1])
                    rotation_velocity = kp * markerBearing 
                    rotation_velocity = -(max(min(rotation_velocity, 0.55), -0.55))
                    if abs(markerBearing) < 10:
                        self.publish_velocity(0.0, 0.0)
                        self.next()
                    else:
                        self.publish_velocity(0.0, rotation_velocity)
                else:
                    self.publish_velocity(0.0, 0.6)
            else:
                self.publish_velocity(0.0, 0.6)


        elif self.state == 'DRIVE_UP_BAY':
            self.send_vision_data("pickingMarkers", "")
            time.sleep(0.001)
            all_obstacles = []
            if self.get_distance() < 0.35:
                self.success_count += 1
                if self.success_count >= SUCCESS_THRESHOLD:
                    self.publish_velocity(0.0, 0.0)
                    time.sleep(1.0)
                    self.next()

            if len(self.picking_markers) and self.picking_markers[self.picking_marker_index].exists:

            # --- 3. Attractive field goal (bearing only) ---
                goal_distance = self.picking_markers[self.picking_marker_index].distance/100000  # in meters
                goal_bearing = self.picking_markers[self.picking_marker_index].bearing[1]/1000

                goal = [goal_distance, goal_bearing]

                U_rep = apf.repulsiveField(all_obstacles, self.phi)
                U_att = apf.attractiveField(goal, self.phi)
                best_bearing = apf.bestBearing(U_att, U_rep, self.phi)

                if best_bearing is not None:
                    theta = 0.0
                    e_theta = apf.angle_wrap(best_bearing - theta)

                    k_omega = 0.4
                    v_max = 0.22
                    sigma = np.radians(50)

                    omega = -(k_omega * e_theta)
                    v = v_max * np.exp(-(e_theta**2) / (2*sigma**2))
                    # if (self.back_proof_counter >= BACK_PROOF_LIMIT):
                    #     v = max(v, 0.0)  # Minimum forward speed
                    self.publish_velocity(v, omega)

                else:
                    self.publish_velocity(0.0, 0.65)
            else:
                self.publish_velocity(0.0, 0.65)    
    
            
        elif self.state == 'LEAVE_PICKING_BAY':
            dist = self.get_distance()
            self.send_to_web(f'distance: {dist}, condition: {dist > 0.55}')
            self.publish_velocity(-0.1, 0.0)
            if dist > 0.6:
                self.publish_velocity(0.0, 0.0)
                time.sleep(0.5)
                self.next()


        elif self.state == 'RETURN_ZONE_DYNAMIC':
            distance = self.get_distance()

            self.back_proof_counter += 1
            sign = self.action_queue[self.action_index]['params'][0] / abs(self.action_queue[self.action_index]['params'][0])
            
            stop_distance = abs(self.action_queue[self.action_index]['params'][0]) + 0.1
            # print(self.get_distance())
            self.send_vision_data("shelves,obstacles", "")
            time.sleep(0.001)  # Allow time for vision to process
            all_obstacles = []

            bearings = []

            for shelf in self.shelves:
                # Use radians directly — remove the /1000 scaling
                bearings.extend([shelf.bearing[0]/1000, shelf.bearing[1]/1000, shelf.bearing[2]/1000])

            if len(bearings)!=0:
                if sign > 0:
                    offset_deg = 14 * sign
                    self.ref_bearing = min(bearings)  # Choose smallest bearing (leftmost object)
                else:
                    offset_deg = 23 * sign
                    self.ref_bearing = max(bearings)  # Choose largest bearing (rightmost object)

                goal_bearing = self.ref_bearing - np.radians(offset_deg)           
            else:
                goal_bearing = np.radians(30*sign)

            self.nav_status_pub.publish(String(data=f"Goal Bearing: {np.degrees(goal_bearing):.2f}"))

            # --- 3. Attractive field goal (bearing only) ---
            goal_distance = 0.5  # Arbitrary far distance to form the attractive vector
            goal = [goal_distance, goal_bearing]
            print(goal_bearing)
            if abs(np.degrees(goal_bearing)) < 30:
                if distance < stop_distance:
                    self.success_count += 1
                    if self.success_count >= 5:
                        self.publish_velocity(0.0, 0.0)  # Ensure stop before any transition
                        self.nav_status_pub.publish(String(data=f"Distance: {distance:.2f}"))
                        # self.state = 'TURN_90_DEG'
                        # self.turn_angle = -85
                        time.sleep(1.5)
                        self.next()
                else:
                    self.success_count = 0 
                if self.prev_tof_distance is None:
                    self.prev_tof_distance = distance
            self.nav_status_pub.publish(String(data=f"Desired: {stop_distance}, Distance: {distance:.2f} m, Condition: {distance < stop_distance}, Success: {self.success_count}"))
            # self.nav_status_pub.publish(String(data=f"Stop Distance: {self.action_queue[self.action_index]['params'][0]}"))

            if distance < self.picking_bay_distances[1]:
                self.publish_velocity(0.15, 0.05 * sign)

            
            else:
                U_rep = apf.repulsiveField(all_obstacles, self.phi)
                U_att = apf.attractiveField(goal, self.phi)
                best_bearing = apf.bestBearing(U_att, U_rep, self.phi)

                if best_bearing is not None:
                    theta = 0.0
                    e_theta = apf.angle_wrap(best_bearing - theta)

                    k_omega = 0.3
                    v_max = 0.165
                    sigma = np.radians(40)
                    omega = -(k_omega * e_theta)
                    v = v_max * np.exp(-(e_theta**2) / (2*sigma**2))
                    self.publish_velocity(v, omega)
                    # print("(" + str(v) + ", " + str(omega) + ")")
                else:
                    self.publish_velocity(0.0, 0.4)

                # print(tof_distance)
                if (distance is not None) and (self.prev_tof_distance is not None):
                    tof_distance_diff = abs(self.prev_tof_distance - distance)
                    if (distance) == 0 or tof_distance_diff > 0.3:
                        self.tof_sensor = PiicoDev_VL53L1X()
                        return  # Skip this loop iteration if reading is invalid or changed too rapidly          
    
            self.prev_tof_distance = distance


        elif self.state == 'TURN_DYNAMIC': # -175 when turning 180 degrees
            self.target_angle = self.prev_imu + self.action_queue[self.action_index]['params'][0]
            # sign = self.target_angle / abs(self.target_angle)
            sign = self.action_queue[self.action_index]['params'][0] / abs(self.action_queue[self.action_index]['params'][0])
            turn_condition = False
            self.nav_status_pub.publish(String(data=f"Target Angle: {self.target_angle}, Current Val: {self.rot_z}"))

            if self.action_queue[self.action_index]['params'][0] < 0:
                turn_condition = self.target_angle < self.rot_z
            else:
                turn_condition = self.target_angle > self.rot_z
            
            if turn_condition:
                self.publish_velocity(0.0, -sign * 0.7)
            else:
                self.publish_velocity(0.0, 0.0)
                time.sleep(0.1)
                self.next()
                

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