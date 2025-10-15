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

SUCCESS_THRESHOLD = 8  # Number of consecutive confirmations needed
BACK_PROOF_LIMIT = 30

SUCCESS_COUNT_THRESHOLD = 3  # Number of consecutive successes required

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

        self.picking_marker_index = 1
        self.picking_bay_distances= [0, 0.6, 0.9]  # Distances to picking bays in meters

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

        # Tof
        self.tof_sensor = PiicoDev_VL53L1X()

    def get_distance(self):
        return self.tof_sensor.read() / 1000

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
        self.get_logger().info(str(self.state))
        # print(self.gZ)
        if self.state == 'START':
            self.aisle_index = None
            self.success_count = 0
            self.tof_distance = 10  # Initialize with a large distance
            self.state = 'FIND_AISLE_MARKER'

        elif self.state == 'FIND_AISLE_MARKER':
            self.send_vision_data("aisleMarkers", "")
            for i, marker in enumerate(self.aisle_markers):
                if marker.exists:
                    self.aisle_index = i
                    print(self.aisle_index)
                    self.state = 'DRIVE_TO_AISLE_FRONT'
                    # self.state = 'DONE'
                    break
            if self.aisle_index is None:
                self.publish_velocity(0.0, 0.7)
                return


        elif self.state == 'DRIVE_TO_AISLE_FRONT':
            self.send_vision_data("aisleMarkers,obstacles,shelves", "")
            for i, marker in enumerate(self.aisle_markers):
                if marker.exists:
                    self.aisle_index = i
                    break
            if self.aisle_index is None:
                self.publish_velocity(0.0, 0.7)
                return
            # aisleBearing = math.degrees((self.aisle_markers[self.aisle_index].bearing[1])/1000)
            aisleBearing = ((self.aisle_markers[self.aisle_index].bearing[1])/1000) 
            aisleDistance = (self.aisle_markers[self.aisle_index].distance)/100000
            if self.aisle_markers[self.aisle_index].exists:
                if aisleDistance < (1.37):
                    self.success_count += 1
                    if (self.success_count >= SUCCESS_COUNT_THRESHOLD):
                        self.publish_velocity(0.0, 0.0)
                        self.start_turn_imu = self.rot_z
                        self.success_count = 0
                        self.state = 'TURN_TOWARDS_PICKING_BAY_DIR'
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
                    v_max = 0.3
                    sigma = np.radians(40)

                    omega = -(k_omega * e_theta)
                    v = v_max * np.exp(-(e_theta**2) / (2*sigma**2))
                    self.publish_velocity(v, omega)
                    # print("(" + str(v) + ", " + str(omega) + ")")
                    # print(v)
                else:
                    self.publish_velocity(0.0, 0.4)
                    print("No valid bearing found, rotating...")

        elif self.state == 'TURN_TOWARDS_PICKING_BAY_DIR':
            # Duration and angular velocity
            turn_duration = 1  # seconds
            angular_speed = 0.6  # rad/s (positive = left, negative = right)

            # Start time tracking
            if not hasattr(self, 'turn_start_time'):
                self.turn_start_time = time.time()
                self.get_logger().info(f"Starting timed turn for {turn_duration}s")

            elapsed = time.time() - self.turn_start_time

            if elapsed < turn_duration:
                self.publish_velocity(0.0, angular_speed)
            else:
                # Stop turning after time elapsed
                self.publish_velocity(0.0, 0.0)
                self.get_logger().info("Timed turn complete.")
                del self.turn_start_time  # reset for next time
                self.state = 'DRIVE_TO_BAY_FRONT'  # replace with your desired next state

            # target_heading = self.start_turn_imu - np.radians(90)  # Turn 90 degrees from current heading
            # current_heading = self.rot_z  # Current heading from IMU

            # e_theta = apf.angle_wrap(target_heading - current_heading)

            # kp = 0.5  # Adjust gain as needed
            # rotation_velocity = kp * e_theta
            # rotation_velocity = -(max(min(rotation_velocity, 0.6), -0.6))

            # if abs(np.degrees(e_theta)) < 10:  # close enough to target
            #     self.publish_velocity(0.0, 0.0)
            #     time.sleep(1.0)
            #     # self.state = 'DRIVE_TO_BAY_FRONT'
            #     self.state = 'DONE'
            #     # shaan was here
            # else:
            #     self.publish_velocity(0.0, rotation_velocity)
            #     # print("Turning...")

        elif self.state == 'DRIVE_TO_BAY_FRONT':
            self.back_proof_counter += 1
            # print(self.get_distance())
            self.send_vision_data("shelves,obstacles", "")
            time.sleep(0.001)  # Allow time for vision to process
            all_obstacles = []

            # for i, group in enumerate(self.obstacles):
            #     # all_obstacles.append(((group.distance)/100000, (group.bearing[0])/1000))  # merge lists
            #     all_obstacles.append(((group.distance)/100000, (group.bearing[1])/1000))  # merge lists

            # --- 2. Determine reference bearing ---
            # Select shelf or picking bay bearings (whichever is detected)
            bearings = []

            for shelf in self.shelves:
                # Use radians directly — remove the /1000 scaling
                bearings.extend([shelf.bearing[0]/1000, shelf.bearing[1]/1000, shelf.bearing[2]/1000])

            if len(bearings)!=0:
                offset_deg = 20
                self.ref_bearing = min(bearings)  # Choose smallest bearing (leftmost object)
                goal_bearing = self.ref_bearing - np.radians(offset_deg)           
            else:
                goal_bearing = np.radians(40)


            # --- 3. Attractive field goal (bearing only) ---
            goal_distance = 0.5  # Arbitrary far distance to form the attractive vector
            goal = [goal_distance, goal_bearing]
            print(goal_bearing)
            if abs(np.degrees(goal_bearing)) < 15:
                if self.prev_tof_distance is None:
                    self.prev_tof_distance = self.get_distance()
                if (self.picking_marker_index == 2):
                    stop_distance = self.picking_bay_distances[1]
            stop_distance = self.picking_bay_distances[self.picking_marker_index]+0.1
            if self.get_distance() < stop_distance:
                self.success_count += 1
                if self.success_count >= SUCCESS_THRESHOLD:
                    if (self.picking_marker_index == 2):
                        self.state = 'APF_PICKING_THREE'
                    else:
                        self.publish_velocity(0.0, 0.0)
                        self.state = 'FIND_PICKING_BAY'
                

            U_rep = apf.repulsiveField(all_obstacles, self.phi)
            U_att = apf.attractiveField(goal, self.phi)
            best_bearing = apf.bestBearing(U_att, U_rep, self.phi)

            if best_bearing is not None:
                theta = 0.0
                e_theta = apf.angle_wrap(best_bearing - theta)

                k_omega = 0.4
                v_max = 0.2
                sigma = np.radians(50)
                omega = -(k_omega * e_theta)
                v = v_max * np.exp(-(e_theta**2) / (2*sigma**2))
                self.publish_velocity(v, omega)
                # print("(" + str(v) + ", " + str(omega) + ")")
            else:
                self.publish_velocity(0.0, 0.4)

            # print(tof_distance)
            if (self.get_distance() is not None) and (self.prev_tof_distance is not None):
                tof_distance_diff = abs(self.prev_tof_distance - self.get_distance())
                if (self.get_distance()) == 0 or tof_distance_diff > 0.3:
                    self.tof_sensor = PiicoDev_VL53L1X()
                    return  # Skip this loop iteration if reading is invalid or changed too rapidly
            

            self.prev_tof_distance = self.get_distance()
                
        elif self.state == 'APF_PICKING_THREE':
            self.publish_velocity(0.25, 0.0)
            if self.get_distance() < self.picking_bay_distances[self.picking_marker_index]:
                self.publish_velocity(0.0, 0.0)
                self.success_count = 0
                self.state = 'FIND_PICKING_BAY'

        
        elif self.state == 'FIND_PICKING_BAY':
            self.send_vision_data("pickingMarkers", "")
            time.sleep(0.001)
            if len(self.picking_markers) != 0:
                if self.picking_markers[self.picking_marker_index].exists:
                    kp = 0.08
                    markerBearing = np.degrees(self.picking_markers[self.picking_marker_index].bearing[1])
                    rotation_velocity = kp * markerBearing 
                    rotation_velocity = -(max(min(rotation_velocity, 0.55), -0.55))
                    if abs(markerBearing) < 6:
                        self.publish_velocity(0.0, 0.0)
                        self.state = 'DRIVE_UP_BAY'
                    else:
                        self.publish_velocity(0.0, rotation_velocity)
                else:
                    self.publish_velocity(0.0, 0.65)
            else:
                self.publish_velocity(0.0, 0.65)

        elif self.state == 'DRIVE_UP_BAY':
            self.send_vision_data("pickingMarkers", "")
            time.sleep(0.001)
            all_obstacles = []
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
                    v_max = 0.25
                    sigma = np.radians(50)

                    omega = -(k_omega * e_theta)
                    v = v_max * np.exp(-(e_theta**2) / (2*sigma**2))
                    # if (self.back_proof_counter >= BACK_PROOF_LIMIT):
                    #     v = max(v, 0.0)  # Minimum forward speed
                    self.publish_velocity(v, omega)

                else:
                    self.publish_velocity(0.0, 0.65)
                self.tof_distance = self.get_distance() # in meters
                if goal_distance < 0.2 or self.tof_distance < 0.35:
                    self.success_count += 1
                    if self.success_count >= SUCCESS_THRESHOLD:
                        self.publish_velocity(0.0, 0.0)
                        self.state = 'DONE'
            else:
                self.publish_velocity(0.0, 0.65)    



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
