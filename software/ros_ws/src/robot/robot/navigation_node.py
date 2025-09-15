import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
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
        self.velocities_pub = self.create_publisher(Twist, '/mobility_twist', 10)

        # Subscribers
        self.point_of_interest_sub = self.create_subscription(String, "/poi", self.poi_callback, 10)

        # State machine variables
        self.state = 'START'
        self.current_heading = 0.0
        self.target_heading = 0.0

        self.timer_ = self.create_timer(0.001, self.state_machine)

        self.objects = []
        self.aisle_markers = []
        self.shelves = []

        self.phi = np.linspace(-np.pi, np.pi, 360)  # Shared angular grid for fields

    # ---------------------------
    # Potential fields
    # ---------------------------
    def repulsiveField(obstacleList, phi=np.linspace(-np.pi, np.pi, 360)):
        U_rep = np.zeros_like(phi)
        if not obstacleList:
            return U_rep
        
        for obs in obstacleList:
            if obs is None or len(obs) != 2:
                continue
            obs_distance, obs_bearing = obs
            if obs_distance <= 0 or abs(obs_bearing) > np.pi/2:
                continue

            dphi = np.arcsin((obstacle_width/2) / obs_distance) if obs_distance > (obstacle_width/2) else np.pi/2
            mask = (phi >= (obs_bearing - dphi)) & (phi <= (obs_bearing + dphi))
            U_rep[mask] = np.maximum(U_rep[mask], (1.0 / obs_distance))
        return U_rep

    def attractiveField(target, phi=np.linspace(-np.pi, np.pi, 360), max_bearing_deg=90):
        if target is None:
            return np.zeros_like(phi)
        target_distance, target_bearing = target
        slope = target_distance / np.radians(max_bearing_deg)
        U_att = np.maximum(0.0, target_distance - np.abs(phi - target_bearing) * slope)
        return U_att

    def bestBearing(U_att, U_rep, phi=np.linspace(-np.pi, np.pi, 360)):
        # Sensor-view slide uses subtraction (Goal - Obstacles)
        U_total = U_att - U_rep
        if not np.isfinite(U_total).all():
            return None
        if np.allclose(U_total, 0.0):
            return None
        best_index = np.argmax(U_total)
        return phi[best_index]   # radians

    # --------------------------- Get distance measurements ---------------------------
    def get_measurements(self, shelfID):
        """Return aisle index, shelf marker distance, and orientation."""
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

    # --------------------------- Robot movement and actions publisher ---------------------------
    def publish_velocity(self, forward_vel, angular_vel):
        twist_msg = Twist()
        twist_msg.linear.x = forward_vel
        twist_msg.angular.z = angular_vel
        self.velocities_pub.publish(twist_msg)

    def publish_collection(self):
        # Placeholder: implement gripper or collection action
        self.get_logger().info("Dropping item at shelf...")

    # --------------------------- Point of Interest callback ---------------------------
    def poi_callback(self, msg):
        self.pois = {}
        for item in msg.data:
            t = item["type"]
            if t not in self.pois:
                self.pois[t] = []
            self.pois[t].append((item["data"], item["distance"], item["bearing"]))

    def filter_poi(self, poi_type):
        distance_bearing_list = []
        for d in self.poi:
            if d["type"] == poi_type:
                distance_bearing_list.append({"data": d["data"], "distance": d["distance"], "bearing": d["bearing"]})
        return distance_bearing_list
    
    # ---------------- STATE MACHINE --------------
    def state_machine(self):
        if self.state == 'START':
            self.aisle_index, self.shelfMarkerDistance, self.shelfOrientation = self.get_measurements(shelfID)
            self.get_logger().info(f"Target shelf {shelfID}, aisle {self.aisle_index+1}")
            self.state = 'DRIVE_INTO_AISLE'

        elif self.state == 'DRIVE_INTO_AISLE':
            self.send_vision_data("isleMarkers,obstacles,shelves", "")
            self.objects = self.filter_poi("obstacle")
            self.aisle_markers = self.filter_poi("isleMarker")
            self.shelves = self.filter_poi("shelf")
            distance_to_marker = self.aisle_markers[self.aisle_index+1]["distance"]
            marker_bearing = self.aisle_markers[self.aisle_index+1]["bearing"][1] 
            all_obstacles = []
            for group in (self.objects, self.shelves):
                all_obstacles.extend([(obj["distance"], obj["bearing"]) for obj in group])
            error = distance_to_marker - self.shelfMarkerDistance
            U_rep = self.repulsiveField(all_obstacles, self.phi)
            U_att = self.attractiveField((distance_to_marker, marker_bearing), self.phi)
            best_bearing = self.bestBearing(U_att, U_rep, self.phi)
            if best_bearing is not None:
                theta = 0.0  # if you have a robot heading API, replace this with it
                e_theta = self.angle_wrap(best_bearing - theta)
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
                self.state = 'TURN_TO_SHELF'
                self.get_logger().info("Turning to shelf...")

        elif self.state == 'TURN_TO_SHELF':
            left_most_shelf = self.shelves["0"]["bearing"][0]
            right_most_shelf = self.shelves["0"]["bearing"][2]
            state = 'DRIVE_TO_SHELF' 

        elif self.state == 'DRIVE_TO_SHELF':
            distance_to_shelf = self.get_ultrasonic_distance()  # Placeholder function
            target_distance = 0.02
            error = distance_to_shelf - target_distance

            if abs(error) < 0.01:
                self.publish_velocity(0.0, 0.0)
                self.state = 'DROP_ITEM'
                self.get_logger().info("At shelf, dropping item...")
            else:
                v = 0.05
                self.publish_velocity(v, 0.0)

        elif self.state == 'DROP_ITEM':
            self.publish_collection()
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
