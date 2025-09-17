import rclpy
from rclpy.node import Node
from std_msgs.msg import String

import gpiozero
import time

class Collection(Node):
    def __init__(self):
        super().__init__('collection_node')
        self.get_logger().info('Collection node has been started.')

        # Node variables
        self.stop_value = 0.1  # FS90R servo stop point - DOESNT WORK :( - 0.09995
        self.servo1 = AngularServo(
            24,  # GPIO24 (Base)
            min_angle=-90,
            max_angle=90,
            min_pulse_width=0.0005,
            max_pulse_width=0.0025
        )
        self.servo2 = AngularServo(
            12,  # GPIO12 (Gripper Attachment)
            min_angle=-90,
            max_angle=90,
            min_pulse_width=0.0005,
            max_pulse_width=0.0025
        )
        self.servo3 = AngularServo(
            23,  # GPIO23 (Gripper Actuation, FS90R)
            min_angle=-90,
            max_angle=90,
            min_pulse_width=0.0005,
            max_pulse_width=0.0025
        )

        # Initialise subscribers
        self.subscription = self.create_subscription(String, '/servo_actions', self.listener_callback, 10) # Does this need to be collection_action?

        # Initalise publishers
        self.status_publisher = self.create_publisher(String, '/servo_status', 10) # Does this need to be arm_status??
    
    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)

    def on_servo_command(self, msg):
        command = msg.data
        self.get_logger().info(f'Received command: "{command}"')

        if command == "0":
            self.get_logger().info("Doing nothing...")
            sleep(1)
            self.publish_status('Do Nothing Completed')
        elif command == "1":
            self.idle_position()
            # Do i need to add self.publish_status('Arm moved level xyz??')
        elif command == "2":
            self.low_shelf()
        elif command == "3":
            self.middle_shelf()
        elif command == "4":
            self.high_shelf()
        elif command == "5":
            self.gripper_clockwise()
        elif command == "6":
            self.gripper_anticlockwise()
        elif command == "7":
            self.picking_bay_collect()
        # elif command.lower() == "q":
        #     self.get_logger().info("Shutdown command received.")
        #     self.publish_status('_shutting_down')
        #     self.destroy_node()
        #     rclpy.shutdown()
        else:
            self.get_logger().warn(f'Invalid command: "{command}"')
            self.publish_status(f'Invalid Command: {command}')

    def publish_status(self, status):
        """Publish status message to '/servo_status' topic."""
        msg = String()
        msg.data = status
        self.status_publisher.publish(msg)
        self.get_logger().info(f'Status published: {status}')

    def smooth_move(self, servo1_target, servo2_target, step=0.2, delay=0.005):
        """Move servos gradually to target angles with specified step size and delay."""
        current_servo1 = self.servo1.angle if self.servo1.angle is not None else 0
        current_servo2 = self.servo2.angle if self.servo2.angle is not None else 0
        
        # Calculate the number of steps needed
        max_steps = max(
            abs(servo1_target - current_servo1),
            abs(servo2_target - current_servo2)
        ) / step
        max_steps = int(max_steps) + 1 if max_steps > 0 else 1

        # Calculate step increments for each servo
        servo1_step = (servo1_target - current_servo1) / max_steps
        servo2_step = (servo2_target - current_servo2) / max_steps

        # Move servos incrementally
        for _ in range(max_steps):
            current_servo1 += servo1_step
            current_servo2 += servo2_step
            self.servo1.angle = current_servo1
            self.servo2.angle = current_servo2
            sleep(delay)

        self.servo1.angle = servo1_target
        self.servo2.angle = servo2_target
        sleep(1)

    def idle_position(self):
        self.get_logger().info('Moving to Idle Position')
        self.smooth_move(servo1_target=-50, servo2_target=90)
        self.publish_status('Idle Position Completed') # Or is having here fine?

    def picking_bay_collect(self):
        self.get_logger().info('Moving to Picking Bay Collection')
        self.smooth_move(servo1_target=-45, servo2_target=20)
        self.publish_status('Picking Bay Collection Completed')

    def low_shelf(self):
        self.get_logger().info('Moving to Low Shelf')
        self.smooth_move(servo1_target=-20, servo2_target=20)
        self.publish_status('Low Shelf Completed')

    def middle_shelf(self):
        self.get_logger().info('Moving to Middle Shelf')
        self.smooth_move(servo1_target=10, servo2_target=-15)
        self.publish_status('Middle Shelf Completed')

    def high_shelf(self):
        self.get_logger().info('Moving to High Shelf')
        self.smooth_move(servo1_target=45, servo2_target=-49)
        self.publish_status('High Shelf Completed')

    def gripper_clockwise(self, duration=0.10):
        self.get_logger().info('Rotating Gripper Clockwise')
        self.servo3.value = -0.5 # Need to adjust/calibrate
        sleep(duration)
        self.servo3.value = self.stop_value  # DOESNT WORK YET!!!
        self.get_logger().info('Gripper Stopped')
        self.publish_status('Gripper Clockwise Completed')

    def gripper_anticlockwise(self, duration=0.10):
        """Rotate gripper anticlockwise."""
        self.get_logger().info('Rotating Gripper Anticlockwise')
        self.servo3.value = 0.5
        sleep(duration)
        self.servo3.value = self.stop_value 
        self.get_logger().info('Gripper Stopped')
        self.publish_status('Gripper Anticlockwise Completed')

    def actuate_gripper(self):
        """Actuate gripper (close then open)."""
        self.get_logger().info('Actuating Gripper')
        self.gripper_clockwise(duration=0.10)  # Close gripper
        sleep(1)
        self.gripper_anticlockwise(duration=0.10)  # Open gripper
        self.publish_status('Gripper Actuation Completed')



def main():
    rclpy.init()
    collection_node = Collection()
    rclpy.spin(collection_node)
    collection_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()