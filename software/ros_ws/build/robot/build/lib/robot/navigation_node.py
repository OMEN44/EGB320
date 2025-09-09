import rclpy
from rclpy.node import Node

import gpiozero
import time

class Navigation(Node):
    def __init__(self):
        super().__init__('navigation_node')
        self.get_logger().info('Navigation node has be.')
        # Create 100ms timer
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        led = gpiozero.LED(14)
        # while True: 
        led.on()
        self.get_logger().info("LED ON")
        time.sleep(1)
        led.off()
        self.get_logger().info("LED OFF")
        time.sleep(1)


def main():
    rclpy.init()
    navigation_node = Navigation()
    rclpy.spin(navigation_node)
    navigation_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()