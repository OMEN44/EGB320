import rclpy
from rclpy.node import Node
from PiicoDev_MPU6050 import PiicoDev_MPU6050
import time
import numpy as np

from geometry_msgs.msg import Twist

class Test(Node):
    def __init__(self):
        super().__init__('test_node')
        self.get_logger().info('Test node has been started.')

        self.testTimer = self.create_timer(0.01, self.timer_callback)
        self.twistPub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.imu = PiicoDev_MPU6050()
        self.time = time.time()
        self.rotation = 0
        self.velocity = 0
        self.switch = False
        self.bias = {
            "Ax": 0,
            "Gz": 0
        }

        # Calibrate IMU
        calibrationSteps = 100
        count = 0
        while count < calibrationSteps:
            accel = self.imu.read_accel_data() # read the accelerometer [ms^-2]
            self.bias["Ax"] += accel["x"]

            gyro = self.imu.read_gyro_data()   # read the gyro [deg/s]
            self.bias["Gz"] += gyro["z"]
            count += 1

        self.bias["Ax"] /= calibrationSteps
        self.bias["Gz"] /= calibrationSteps

    def timer_callback(self):

        aX = self.imu.read_accel_data()["x"] - self.bias["Ax"]
        gZ = self.imu.read_gyro_data()["z"] - self.bias["Gz"]

        now = time.time()
        dt = now - self.time
        self.time = now

        self.rotation += gZ * dt
        # self.velocity += np.round(aX * dt, 2)

        def pad(num):
            return ' ' if num >= 0 else ''

        self.get_logger().info(f'aX: {pad(aX):}{aX:.2f}, \ngZ: {pad(gZ):}{gZ:.2f}, \nrotation: {pad(self.rotation):}{self.rotation:.2f}')

        twist = Twist()

        if not self.switch:
            if (self.rotation < 90):
                twist.angular.z = -1.0
            else:
                self.switch = True
                twist.angular.z = 0.0
        else:
            if (self.rotation > 0):
                twist.angular.z = 1.0
            else:
                self.switch = False
                twist.angular.z = 0.0

        self.get_logger().info(f'Publishing twist: linear.x={twist.linear.x}, angular.z={twist.angular.z}')

        self.twistPub.publish(twist)



def main():
    rclpy.init()
    test_node = Test()
    try:
        rclpy.spin(test_node)
    except KeyboardInterrupt:
        pass
    finally:
        test_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()