import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, String
from robot_interfaces.msg import PoiGroup
import time
import numpy as np
from PiicoDev_MPU6050 import PiicoDev_MPU6050

class Navigation(Node):
    def __init__(self):
        super().__init__('nav_drive_straight')
        self.get_logger().info('Navigation node started — IMU straight drive control active.')

        # Publisher
        self.velocities_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Subscribers (not used but left for structure)
        self.point_of_interest_sub = self.create_subscription(PoiGroup, "/poi", self.poi_callback, 10)
        self.arm_status_sub = self.create_subscription(Bool, "/arm_status", self.arm_status_callback, 10)

        # IMU setup
        self.imu = PiicoDev_MPU6050()
        self.Ax_bias = 0
        self.Gz_bias = 0
        calibration_steps = 100

        self.get_logger().info("Calibrating IMU...")
        for _ in range(calibration_steps):
            accel = self.imu.read_accel_data()
            gyro = self.imu.read_gyro_data()
            self.Ax_bias += accel["x"]
            self.Gz_bias += gyro["z"]
        self.Ax_bias /= calibration_steps
        self.Gz_bias /= calibration_steps
        self.get_logger().info("IMU calibration complete.")

        # Timing
        self.t = time.time()

        # Rotation tracking
        self.rotation = 0.0
        self.target_heading = 0.0  # initial heading

        # Create main control timer
        self.timer_ = self.create_timer(0.02, self.drive_straight)  # 50 Hz update rate

    # --------------------------- Callbacks ---------------------------
    def poi_callback(self, msg):
        pass

    def arm_status_callback(self, msg):
        pass

    # --------------------------- Straight driving ---------------------------
    def drive_straight(self):
        accel = self.imu.read_accel_data()
        gyro = self.imu.read_gyro_data()

        # Subtract biases
        gZ = gyro["z"] - self.Gz_bias

        # Time delta
        now = time.time()
        dt = now - self.t
        self.t = now

        # Integrate gyro to estimate rotation (radians)
        self.rotation += gZ * dt * (np.pi / 180)

        # Simple proportional control for heading correction
        heading_error = self.target_heading - self.rotation
        kP = 2.0  # tuning parameter (try 1.0–3.0)
        angular_correction = kP * heading_error

        # Clamp angular speed
        angular_correction = max(min(angular_correction, 0.5), -0.5)

        # Publish velocity
        twist = Twist()
        twist.linear.x = 0.2       # forward speed
        twist.angular.z = angular_correction
        self.velocities_pub.publish(twist)

        # Debug info (optional)
        # self.get_logger().info(f"Heading: {np.degrees(self.rotation):.2f}°, Corr: {angular_correction:.2f}")

def main():
    rclpy.init()
    node = Navigation()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
