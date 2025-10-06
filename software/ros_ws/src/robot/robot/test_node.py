# import rclpy
# from rclpy.node import Node
# from PiicoDev_MPU6050 import PiicoDev_MPU6050
# import time
# import numpy as np

# from geometry_msgs.msg import Twist

# class Test(Node):
#     def __init__(self):
#         super().__init__('test_node')
#         self.get_logger().info('Test node has been started.')

#         self.testTimer = self.create_timer(0.01, self.timer_callback)
#         self.twistPub = self.create_publisher(Twist, '/cmd_vel', 10)

#         self.imu = PiicoDev_MPU6050()
#         self.time = time.time()
#         self.rotation = 0
#         self.velocity = 0
#         self.switch = False
#         self.bias = {
#             "Ax": 0,
#             "Gz": 0
#         }

#         # Calibrate IMU
#         calibrationSteps = 100
#         count = 0
#         while count < calibrationSteps:
#             accel = self.imu.read_accel_data() # read the accelerometer [ms^-2]
#             self.bias["Ax"] += accel["x"]

#             gyro = self.imu.read_gyro_data()   # read the gyro [deg/s]
#             self.bias["Gz"] += gyro["z"]
#             count += 1

#         self.bias["Ax"] /= calibrationSteps
#         self.bias["Gz"] /= calibrationSteps

#     def timer_callback(self):

#         aX = self.imu.read_accel_data()["x"] - self.bias["Ax"]
#         gZ = self.imu.read_gyro_data()["z"] - self.bias["Gz"]

#         now = time.time()
#         dt = now - self.time
#         self.time = now

#         self.rotation += gZ * dt
#         # self.velocity += np.round(aX * dt, 2)

#         def pad(num):
#             return ' ' if num >= 0 else ''

#         self.get_logger().info(f'aX: {pad(aX):}{aX:.2f}, \ngZ: {pad(gZ):}{gZ:.2f}, \nrotation: {pad(self.rotation):}{self.rotation:.2f}')

#         twist = Twist()

#         if not self.switch:
#             if (self.rotation < 90):
#                 twist.angular.z = -1.0
#             else:
#                 self.switch = True
#                 twist.angular.z = 0.0
#         else:
#             if (self.rotation > 0):
#                 twist.angular.z = 1.0
#             else:
#                 self.switch = False
#                 twist.angular.z = 0.0

#         self.get_logger().info(f'Publishing twist: linear.x={twist.linear.x}, angular.z={twist.angular.z}')

#         self.twistPub.publish(twist)



# def main():
#     rclpy.init()
#     test_node = Test()
#     try:
#         rclpy.spin(test_node)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         test_node.destroy_node()
#         rclpy.shutdown()

# if __name__ == '__main__':
#     main()

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int8, String
from time import time

class ServoTestNode(Node):
    def __init__(self):
        super().__init__('servo_test_node')
        self.get_logger().info('Servo test node started')

        # Initialize publisher for servo actions
        self.servo_pub = self.create_publisher(Int8, '/servo_actions', 10)
        # Subscribe to servo status to confirm action completion
        self.status_sub = self.create_subscription(
            String, '/servo_status', self.status_callback, 10)

        # Command sequence and timer
        self.commands = [5, 0, 1, 5, 0, 4, 2, 5, 0, 4, 3, 5]  # Sequence to match desired actions
        self.current_command_index = 0
        self.last_time = time()
        self.action_completed = True
        self.test_timer = self.create_timer(0.1, self.timer_callback)  # Check every 0.1s

    def status_callback(self, msg):
        """Handle status messages from CollectionNode."""
        self.get_logger().info(f'Status: {msg.data}')
        if 'Completed' in msg.data or 'Gripper Closed' in msg.data or 'Gripper Opened' in msg.data:
            self.action_completed = True
            self.get_logger().info('Action completed, ready for next command')
        elif 'Invalid Command' in msg.data:
            self.get_logger().info(f'CollectionNode reported: {msg.data}')
            self.action_completed = True

    def timer_callback(self):
        """Publish the next servo command when the previous action is complete."""
        if self.current_command_index >= len(self.commands):
            return  # Stop sending commands after sequence completes

        current_time = time()
        if (current_time - self.last_time) >= 15.0 and self.action_completed:
            self.last_time = current_time
            self.action_completed = False

            # Publish the current command
            command = self.commands[self.current_command_index]
            msg = Int8()
            msg.data = command
            self.servo_pub.publish(msg)
            action = {
                0: "Floor Level",
                1: "Bottom Shelf",
                2: "Middle Shelf",
                3: "Top Shelf",
                4: "Close Gripper",
                5: "Open Gripper"
            }.get(command, "Unknown")
            self.get_logger().info(f'Publishing servo command: {command} ({action})')

            # Move to the next command
            self.current_command_index += 1

def main():
    rclpy.init()
    test_node = ServoTestNode()
    rclpy.spin(test_node)
    test_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()