import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Bool, Int32
import time
import numpy as np

from robot.collection.servoDriver import Servo180

class CollectionNode(Node):
    def __init__(self):
        super().__init__('collection_node')
        self.get_logger().info('Collection node started')

        # setup subscribers
        self.numberTopic = self.create_subscription(Float32MultiArray, '/number_topic', self.onNumberMessage, 10)
        self.armAction = self.create_subscription(Int32, '/arm_action', self.onArmAction, 10)
        self.gripperAction = self.create_subscription(Bool, '/gripper_action', self.onGripperAction, 10)

        # setup publishers
        self.armStatus = self.create_publisher(Bool, '/arm_status', 10)

        # timers
        self.updateTimer = self.create_timer(0.01, self.updateServos)

        # Setup servos
        self.arm = Servo180(channel=0, min_pulse=0.3)
        self.gripper = Servo180(channel=1)
        # self.gripper = Servo180(channel=1)
        self.gripper = Servo180(channel=1, min_pulse=0.75, max_pulse=2.4)
        self.arm.set_angle(0)
        self.gripper.set_angle(0)
        time.sleep(0.5) # give servos time to move

        self.numbers = {
            'arm': 0,
            'gripper': 0
        }

        self.currentAngles = [0, 0] # arm, gripper
        self.targets = [0, 0] # arm, gripper
        self.presets = [
            -90,   # floor
            -70,   # level 1
            0,     # level 2
            20,    # level 3
            -60,   # idle
        ]

    def onArmAction(self, msg):
        if msg.data < 0 or msg.data >= len(self.presets):
            self.get_logger().error(f'Invalid action {msg.data}')
            return
        
        self.targets[0] = self.presets[msg.data] # arm
        self.get_logger().info(f'Action {msg.data} received, moving to {self.targets}')

    def onGripperAction(self, msg):
        if msg.data:
            self.targets[1] = 90 # gripper open
        else:
            self.targets[1] = -90 # gripper closed

    def onNumberMessage(self, msg):
        self.targets[0] = max(msg.data[8], msg.data[9])
        self.targets[1] = max(msg.data[10], msg.data[11])

    def updateServos(self):
        for i in range(2):
            if abs(self.targets[i] - self.currentAngles[i]) > 0.1:
                self.currentAngles[i] += (self.targets[i] - self.currentAngles[i]) * 0.1
                self.armStatus.publish(Bool(data=False))
            else:
                self.currentAngles[i] = self.targets[i]
                self.armStatus.publish(Bool(data=True))

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

