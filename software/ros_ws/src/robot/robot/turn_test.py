import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from robot_interfaces.msg import PoiGroup
import time
import numpy as np
from PiicoDev_MPU6050 import PiicoDev_MPU6050

class Navigation(Node):
    def __init__(self):
        super().__init__('nav_turn_360')
        self.get_logger().info('Navigation node started — IMU 360-degree turn control active.')

        # Publisher
        self.velocities_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Subscribers (kept for structure, not used)
        self.point_of_interest_sub = self.create_subscription(PoiGroup, "/poi", self.poi_callback, 10)
        self.arm_status_sub = self.create_subscription(Bool, "/arm_status", self.arm_status_callback, 10)

        # IMU setup
        self.imu = PiicoDev_MPU6050()
        self.Gz_bias = 0
        calibration_steps = 100

        self.get_logger().info("Calibrating IMU...")
        for _ in range(calibration_steps):
            gyro = self.imu.read_gyro_data()
            self.Gz_bias += gyro["z"]
        self.Gz_bias /= calibration_steps
        self.get_logger().info("IMU calibration complete.")

        # Timing
        self.t = time.time()

        # Rotation tracking
        self.rotation = 0.0
        self.turn_completed = False  # Flag to stop after turn

        # Start the 360-degree turn
        self.target_rotation = 2 * np.pi  # 360 degrees in radians

        # Create control timer
        self.timer_ = self.create_timer(0.02, self.turn_360)  # 50 Hz update rate

    # --------------------------- Callbacks ---------------------------
    def poi_callback(self, msg):
        pass

    def arm_status_callback(self, msg):
        pass

    # --------------------------- 360-Degree Turn Control ---------------------------
    def turn_360(self):
        if self.turn_completed:
            return  # Exit if turn is already completed

        gyro = self.imu.read_gyro_data()

        # Subtract bias
        gZ = gyro["z"] - self.Gz_bias

        # Time delta
        now = time.time()
        dt = now - self.t
        self.t = now

        # Integrate gyro to estimate rotation (radians)
        self.rotation += gZ * dt * (np.pi / 180)

        twist = Twist()

        # Check if 360-degree turn is complete
        if abs(self.rotation) < abs(self.target_rotation):
            # Continue turning
            angular_speed = 0.6  # Fixed angular speed (rad/s), adjust as needed
            twist.linear.x = 0.0
            twist.angular.z = angular_speed
            self.get_logger().info(f"Turning... Current angle: {np.degrees(self.rotation):.2f}°")
        else:
            # Turn complete
            self.turn_completed = True
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.get_logger().info(f"360-degree turn completed. Final angle: {np.degrees(self.rotation):.2f}°")

        # Publish velocity
        self.velocities_pub.publish(twist)

def main():
    rclpy.init()
    node = Navigation()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Ensure robot stops on shutdown
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        node.velocities_pub.publish(twist)
        node.get_logger().info("Robot stopped.")
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()