import rclpy
from rclpy.node import Node
from std_msgs.msg import Int8, String
from gpiozero import AngularServo
from time import sleep, time

class CollectionNode(Node):
    def __init__(self):
        super().__init__('collection_node')
        self.get_logger().info('Collection node started')

        # Initialize servos
        self.servo1 = AngularServo(
            12,  # GPIO12 for base rotation
            min_angle=-90,
            max_angle=90,
            min_pulse_width=0.0005,  # 500µs
            max_pulse_width=0.0024   # 2400µs
        )

        self.servo3 = AngularServo(
            13,  # GPIO13 for gripper
            min_angle=-90,  # Closed position
            max_angle=90,   # Open position
            min_pulse_width=0.0005,  # 500µs
            max_pulse_width=0.0025   # 2500µs
        )

        # Initialize subscribers and publishers
        self.subscription = self.create_subscription(
            Int8, '/servo_actions', self.on_servo_command, 10)
        self.status_publisher = self.create_publisher(String, '/servo_status', 10)

        # Smooth movement variables for servo1
        self.target_angle_servo1 = 0
        self.current_angle_servo1 = 0
        self.last_time = time()
        self.dt = 0.005  # 5ms update interval
        self.timer = self.create_timer(self.dt, self.timer_callback)

    def timer_callback(self):
        """Update servo1 angle incrementally towards target angle."""
        current_time = time()
        if (current_time - self.last_time) >= self.dt:
            self.last_time = current_time

            # Smoothly update servo1 angle
            if abs(self.current_angle_servo1 - self.target_angle_servo1) > 0.1:
                if self.current_angle_servo1 < self.target_angle_servo1:
                    self.current_angle_servo1 += 1
                elif self.current_angle_servo1 > self.target_angle_servo1:
                    self.current_angle_servo1 -= 1
                self.servo1.angle = max(min(self.current_angle_servo1, 90), -90)

            self.get_logger().info(
                f'servo1_target={self.target_angle_servo1:.1f}, '
                f'servo3={self.servo3.angle if self.servo3.angle is not None else -90:.1f}, '
                f'servo1_current={self.current_angle_servo1:.1f}'
            )

    def smooth_move(self, servo1_target, servo3_target):
        """Set target angle for servo1 (smooth) and move servo3 instantly."""
        self.get_logger().info(f'Moving: servo1={servo1_target}, servo3={servo3_target}')
        self.target_angle_servo1 = max(min(servo1_target, 90), -90)
        self.servo3.angle = max(min(servo3_target, 90), -90)
        # Wait until servo1 reaches target angle (within 0.1 degrees)
        while abs(self.current_angle_servo1 - self.target_angle_servo1) > 0.1:
            sleep(self.dt)
        sleep(1)

    def open_gripper(self):
        """Set gripper to open position (90 degrees)."""
        self.get_logger().info('Opening gripper')
        self.servo3.angle = 90
        sleep(1)

    def close_gripper(self):
        """Set gripper to closed position (-90 degrees)."""
        self.get_logger().info('Closing gripper')
        self.servo3.angle = -90
        sleep(1)

    def floor_level(self):
        """Move to floor level and handle gripper."""
        self.get_logger().info('Moving to Floor Level')
        self.open_gripper()
        self.smooth_move(-85, 90)
        self.close_gripper()
        self.publish_status('Floor Level Completed')

    def bottom_shelf(self):
        """Move to bottom shelf and open gripper."""
        self.get_logger().info('Moving to Bottom Shelf')
        self.smooth_move(-65, 90)
        self.open_gripper()
        self.publish_status('Bottom Shelf Completed')

    def middle_shelf(self):
        """Move to middle shelf and open gripper."""
        self.get_logger().info('Moving to Middle Shelf')
        self.smooth_move(-20, 90)
        self.open_gripper()
        self.publish_status('Middle Shelf Completed')

    def top_shelf(self):
        """Move to top shelf and open gripper."""
        self.get_logger().info('Moving to Top Shelf')
        self.smooth_move(50, 90)
        self.open_gripper()
        self.publish_status('Top Shelf Completed')

    def publish_status(self, status):
        """Publish status message to '/servo_status' topic."""
        msg = String()
        msg.data = status
        self.status_publisher.publish(msg)
        self.get_logger().info(f'Status: {status}')

    def on_servo_command(self, msg):
        """Handle incoming servo commands."""
        command = msg.data
        self.get_logger().info(f'Received command: {command}')
        if command == 0:
            self.floor_level()
        elif command == 1:
            self.bottom_shelf()
        elif command == 2:
            self.middle_shelf()
        elif command == 3:
            self.top_shelf()
        elif command == 4:
            self.close_gripper()
            self.publish_status('Gripper Closed')
        elif command == 5:
            self.open_gripper()
            self.publish_status('Gripper Opened')
        else:
            self.get_logger().info(f'Invalid command: {command}')
            self.publish_status(f'Invalid Command: {command}')

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