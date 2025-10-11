# import rclpy
# from rclpy.node import Node
# from std_msgs.msg import Int8, String
# from gpiozero import AngularServo
# from time import sleep, time

# class CollectionNode(Node):
#     def __init__(self):
#         super().__init__('collection_node')
#         self.get_logger().info('Collection node started')

#         # Initialize servos
#         self.servo1 = AngularServo(
#             12,  # GPIO12 for base rotation
#             min_angle=-90,
#             max_angle=90,
#             min_pulse_width=0.0005,  # 500µs
#             max_pulse_width=0.0024   # 2400µs
#         )

#         self.servo3 = AngularServo(
#             13,  # GPIO13 for gripper
#             min_angle=-90,  # Closed position
#             max_angle=90,   # Open position
#             min_pulse_width=0.0005,  # 500µs
#             max_pulse_width=0.0025   # 2500µs
#         )

#         # Initialise subscribers and publishers
#         self.subscription = self.create_subscription(
#             Int8, '/collection_action', self.on_servo_command, 10)
#         self.status_publisher = self.create_publisher(String, '/arm_status', 10)

#         # Smooth movement variables for servo1
#         self.target_angle_servo1 = 0
#         self.current_angle_servo1 = 0
#         self.last_time = time()
#         self.dt = 0.005  # 5ms update interval
#         self.timer = self.create_timer(self.dt, self.timer_callback)
#         self.positions = {
#             'top':50,
#             'middle':-20,
#             'bottom':-65,
#             'floor':-85,
#         }

#     def timer_callback(self):
#         current_time = time()
#         if (current_time - self.last_time) >= self.dt:
#             self.last_time = current_time

#             # Smoothly update servo1 angle
#             if abs(self.current_angle_servo1 - self.target_angle_servo1) > 0.1:
#                 if self.current_angle_servo1 < self.target_angle_servo1:
#                     self.current_angle_servo1 += 1
#                 elif self.current_angle_servo1 > self.target_angle_servo1:
#                     self.current_angle_servo1 -= 1
#                 self.servo1.angle = max(min(self.current_angle_servo1, 90), -90)

#             self.get_logger().info(
#                 f'\nservo1_target={self.target_angle_servo1:.1f}, '
#                 f'\nservo3={self.servo3.angle if self.servo3.angle is not None else -90:.1f}, '
#                 f'\nservo1_current={self.current_angle_servo1:.1f}'
#             )
        

#     def smooth_move(self, servo1_target):
#         self.target_angle_servo1 = max(min(servo1_target, 90), -90)
#         self.get_logger().info(f'Moving: servo1={self.target_angle_servo1}')
           

#     def publish_status(self, status):
#         msg = String()
#         msg.data = status
#         self.status_publisher.publish(msg)
#         self.get_logger().info(f'Status: {status}')

#     def on_servo_command(self, msg):
#         command = msg.data
#         self.get_logger().info(f'Received command: {command}')
        
#         if command == 0:
#             self.get_logger().info('Moving to Idle Position')
#             self.servo3.angle = -90
#             self.publish_status('GC')
#             sleep(1)
#             self.smooth_move(-65)
#             self.publish_status('In Idle Position Completed')

#         elif command == 1:
#             self.get_logger().info('Moving to Floor Level')
#             self.servo3.angle = 90
#             self.smooth_move(-85)
#             self.publish_status('Floor Level Completed')

#         elif command == 2:
#             self.get_logger().info('Moving to Bottom Shelf')
#             self.servo3.angle = -90
#             self.smooth_move(-65)
#             # sleep(1)
#             self.servo3.angle = 90
#             self.publish_status('Bottom Shelf Completed')

#         elif command == 3:
#             self.get_logger().info('Moving to Middle Shelf')
#             self.servo3.angle = -90
#             self.smooth_move(-20)
#             # time.sleep(1)
#             self.servo3.angle = 90
#             self.publish_status('Middle Shelf Completed')
    
#         elif command == 4:
#             self.get_logger().info('Moving to Top Shelf')
#             self.servo3.angle = -90
#             self.smooth_move(50)
#             # sleep(1)
#             self.servo3.angle = 90
#             self.publish_status('Top Shelf Completed')

#         # elif command == 5:
#         #     self.get_logger().info('Closing gripper')
#         #     self.servo3.angle = -90
#         #     self.publish_status('Gripper Closed')

#         # elif command == 6:
#         #     self.get_logger().info('Opening gripper')
#         #     self.servo3.angle = 90
#         #     self.publish_status('Gripper Opened')

#         else:
#             self.get_logger().info(f'Invalid command: {command}')
#             self.publish_status(f'Invalid Command: {command}')

# def main():
#     rclpy.init()
#     collection_node = CollectionNode()
#     rclpy.spin(collection_node)
#     collection_node.servo1.close()
#     collection_node.servo3.close()
#     collection_node.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import time
import numpy as np

from robot.collection.servoDriver import Servo180

class CollectionNode(Node):
    def __init__(self):
        super().__init__('collection_node')
        self.get_logger().info('Collection node started')

        # setup subscribers
        self.numberTopic = self.create_subscription(Float32MultiArray, '/number_topic', self.onNumberMessage, 10)

        # timers
        self.updateTimer = self.create_timer(0.01, self.updateServos)

        # Setup servos
        self.arm = Servo180(channel=0, min_pulse=0.3)
        self.gripper = Servo180(channel=1, min_pulse=0.75, max_pulse=2.4)
        self.arm.set_angle(0)
        self.gripper.set_angle(0)
        time.sleep(1)

        self.numbers = {
            'arm': 0,
            'gripper': 0
        }

        self.currentAngles = [0, 0] # arm, gripper
        self.targets = [0, 0] # arm, gripper

    def onNumberMessage(self, msg):
        self.targets[0] = max(msg.data[8], msg.data[9])
        self.targets[1] = max(msg.data[10], msg.data[11])

    def updateServos(self):

        for i in range(2):
            if abs(self.targets[i] - self.currentAngles[i]) > 0.1:
                self.currentAngles[i] += (self.targets[i] - self.currentAngles[i]) * 0.1
            else:
                self.currentAngles[i] = self.targets[i]

        a = self.arm.set_angle(self.currentAngles[0])
        b = self.gripper.set_angle(self.currentAngles[1])

        self.get_logger().info(f'\nArm\nTarget: {self.targets[0]:.1f}, Current: {self.currentAngles[0]:.1f}, Set: {a}\nGripper\nTarget: {self.targets[1]:.1f}, Current: {self.currentAngles[1]:.1f}, Set: {b}')


def main():
    rclpy.init()
    collection_node = CollectionNode()
    rclpy.spin(collection_node)
    collection_node.servo1.close()
    collection_node.servo3.close()
    collection_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

