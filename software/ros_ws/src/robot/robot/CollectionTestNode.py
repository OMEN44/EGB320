# import rclpy
# from rclpy.node import Node
# from std_msgs.msg import Int8
# from time import time

# class ServoTestNode(Node):
#     def __init__(self):
#         super().__init__('servo_test_node')
#         self.get_logger().info('Servo test node has been started.')

#         # Initialize publisher for servo actions
#         self.servo_pub = self.create_publisher(Int8, '/servo_actions', 10)

#         # Timer for sending test commands
#         self.test_timer = self.create_timer(10.0, self.timer_callback)  # Send command every 10 seconds
#         self.commands = [0, 1, 2, 3, 4]  # Commands: 0=floor, 1=bottom, 2=middle, 3=top, 4=close gripper
#         self.current_command_index = 0
#         self.last_time = time()

#     def timer_callback(self):
#         current_time = time()
#         if (current_time - self.last_time) >= 10.0:  # Ensure 10 second delay
#             self.last_time = current_time

#             # Get the current command
#             command = self.commands[self.current_command_index]
#             msg = Int8()
#             msg.data = command
#             self.servo_pub.publish(msg)
#             action = {
#                 0: "Floor Level",
#                 1: "Bottom Shelf",
#                 2: "Middle Shelf",
#                 3: "Top Shelf",
#                 4: "Close Gripper"
#             }.get(command, "Unknown")
#             self.get_logger().info(f'Servo command to be published: {command} ({action})')

#             # Move to the next command, looping back to start
#             self.current_command_index = (self.current_command_index + 1) % len(self.commands)

# def main():
#     rclpy.init()
#     test_node = ServoTestNode()
#     try:
#         rclpy.spin(test_node)
#     except KeyboardInterrupt:
#         test_node.get_logger().info('Shutting down servo test node')
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

        # Initialise publisher for servo actions
        self.servo_pub = self.create_publisher(Int8, '/servo_actions', 10)
        # Subscribe to servo status to confirm action completion
        self.status_sub = self.create_subscription(
            String, '/servo_status', self.status_callback, 10)

        # Timer for sending test commands
        self.test_timer = self.create_timer(0.1, self.timer_callback)  # 0.1s period
        self.commands = [0, 1, 2, 3, 4]  # Commands: 0=floor, 1=bottom, 2=middle, 3=top, 4=close gripper
        self.current_command_index = 0
        self.last_time = time()
        self.action_completed = True  # Flag to wait for action completion

    def status_callback(self, msg):
        """Handle status messages from CollectionNode."""
        self.get_logger().info(f'Status: {msg.data}')
        if 'Completed' in msg.data or 'Gripper Closed' in msg.data:
            self.action_completed = True
            self.get_logger().info('Action completed, ready for next command')
        elif 'Error' in msg.data or 'Invalid Command' in msg.data:
            self.get_logger().info(f'CollectionNode reported: {msg.data}')
            self.action_completed = True

    def timer_callback(self):
        "Publish the next servo command when the previous action is complete."
        current_time = time()
        if (current_time - self.last_time) >= 5.0 and self.action_completed:
            self.last_time = current_time
            self.action_completed = False  # Reset flag until action completes

            # Get the current command
            command = self.commands[self.current_command_index]
            msg = Int8()
            msg.data = command
            self.servo_pub.publish(msg)
            action = {
                0: "Floor Level",
                1: "Bottom Shelf",
                2: "Middle Shelf",
                3: "Top Shelf",
                4: "Close Gripper"
            }.get(command, "Unknown")
            self.get_logger().info(f'Publishing servo command: {command} ({action})')

            # Move to the next command, looping back to start
            self.current_command_index = (self.current_command_index + 1) % len(self.commands)

def main():
    rclpy.init()
    test_node = ServoTestNode()
    rclpy.spin(test_node)
    test_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()