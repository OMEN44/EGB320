import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from dfrobot_driver import THIS_BOARD_TYPE, DFRobot_DC_Motor_IIC
import time

class MotorDriver:
    def __init__(self, addr=0x10, bus_id=1):
        self.motor = DFRobot_DC_Motor_IIC(bus_id=bus_id, addr=addr)
        try:
            while self.motor.begin() != self.motor.STA_OK:
                print("Motor driver not detected")
                time.sleep(2)
            print("Motor driver initialized")
        except Exception as e:
            print(f"Error initializing motor driver: {e}")
            raise

        # Motor IDs
        self.left_motor_id = self.motor.M1
        self.right_motor_id = self.motor.M2

        # PWM Limits
        self.max_pwm = 80  # Current max, change to 100 for testing
        self.min_pwm = 0

        # Current PWM values
        self.left_pwm = 0
        self.right_pwm = 0

        # Motor directions
        self.left_dir = self.motor.CW  # Left motor forward is CW
        self.right_dir = self.motor.CCW  # Right motor forward is CCW

        # Initialize status
        self.status = "Stopped"

    def setMotorPWM(self, motor_id, pwm, direction):
        try:
            pwm = max(self.min_pwm, min(self.max_pwm, int(abs(pwm))))
            self.motor.motor_movement([motor_id], direction, pwm)
        except Exception as e:
            print(f"Error setting motor PWM: {e}")

    def set_left_pwm(self, pwm):
        self.left_pwm = pwm
        direction = self.motor.CW if pwm >= 0 else self.motor.CCW
        self.setMotorPWM(self.left_motor_id, pwm, direction)

    def set_right_pwm(self, pwm):
        self.right_pwm = pwm
        direction = self.motor.CCW if pwm >= 0 else self.motor.CW
        self.setMotorPWM(self.right_motor_id, pwm, direction)

    def stop(self):
        try:
            self.motor.motor_stop([self.left_motor_id, self.right_motor_id])
            self.left_pwm = 0
            self.right_pwm = 0
            self.status = "Stopped"
        except Exception as e:
            print(f"Error stopping motors: {e}")

    def update_pwm_step(self, motor_id, target_pwm, direction, step_size=5):

        current_pwm = abs(self.left_pwm if motor_id == self.motor.M1 else self.right_pwm)
        target_pwm = max(self.min_pwm, min(self.max_pwm, int(abs(target_pwm))))
        
        if current_pwm == target_pwm:
            return True
        
        # Checking whether PWM needs to be increased or decreased
        if current_pwm < target_pwm:
            next_pwm = min(current_pwm + step_size, target_pwm)
        else:
            next_pwm = max(current_pwm - step_size, target_pwm)
        
        self.setMotorPWM(motor_id, next_pwm, direction)
        if motor_id == self.motor.M1:
            self.left_pwm = next_pwm if direction == self.motor.CW else -next_pwm
        else:
            self.right_pwm = next_pwm if direction == self.motor.CCW else -next_pwm
        return False

    def convertVelocitytoPWM(self, linear_velocity, angular_velocity, wheel_radius=0.215, wheelbase=0.08):

        try:

            v_left = linear_velocity - (angular_velocity * wheelbase / 2.0) 
            v_right = linear_velocity + (angular_velocity * wheelbase / 2.0) 

            # Convert velocities to PWM: v = 0.00727 * PWM - 0.0921 (EQUATION MAY CHANGE AFTER MORE TESTING)
            pwm_left = (v_left + 0.0921) / 0.00727
            pwm_right = (v_right + 0.0921) / 0.00727

            # Clamp PWM values
            pwm_left = max(self.min_pwm, min(self.max_pwm, int(pwm_left)))
            pwm_right = max(self.min_pwm, min(self.max_pwm, int(pwm_right)))

            left_direction = self.motor.CW if v_left >= 0 else self.motor.CCW
            right_direction = self.motor.CCW if v_right >= 0 else self.motor.CW

            return pwm_left, pwm_right, left_direction, right_direction
        except Exception as e:
            print(f"Error converting velocity to PWM: {e}")
            self.status = "Error"
            self.stop()
            return 0, 0, self.motor.CW, self.motor.CCW

class Mobility(Node):
    def __init__(self):
        super().__init__('mobility_node')
        self.get_logger().info('Mobility node has been started.')
        self.driver = MotorDriver(addr=0x10, bus_id=1)
        self.wheel_radius = 0.0215 
        self.wheelbase = 0.08  
        self.targetSpeed = (0, 0)  # Target PWM (left, right)
        self.targetDirections = (self.driver.left_dir, self.driver.right_dir)  # Target directions
        self.last_twist = None  

        # Create 100ms timer
        timer_period = 0.00001
        self.mobilityTimer = self.create_time(timer_period, self.timer_callback)

        # Initialise subscribers
        #self.function is the function that i'm using on the subscriber's data
        #slower, cuz update_targets_callback gets called everytime the publisher sends something to the topic (2 seconds)
        self.subscription = self.create_subscription(Twist,'/cmd_vel', self.update_targets_callback, 10)
        self.subscription

    def update_targets_callback(self, msg):
        linear_vel = msg.linear.x
        angular_vel = msg.angular.z
        self.get_logger().info(f'Received Twist: linear.x={linear_vel:.2f}, angular.z={angular_vel:.2f}')

        # Store latest Twist message
        self.last_twist = msg

        # Convert velocities to PWM and directions
        pwm_left, pwm_right, left_direction, right_direction = self.driver.convertVelocitytoPWM(
            linear_vel, angular_vel, self.wheel_radius, self.wheelbase
        )

        self.targetSpeed = (pwm_left, pwm_right)
        self.targetDirections = (left_direction, right_direction)
        self.get_logger().info(f'Target PWM: left={pwm_left}, right={pwm_right}')

        self.publisher_.publish(msg)


    # Faster timer, actually updating motor pwm
    def timer_callback(self):

        target_pwm_left, target_pwm_right = self.targetSpeed
        left_direction, right_direction = self.targetDirections

        updated_left_PWM = self.driver.update_pwm_step(
            self.driver.left_motor_id, abs(target_pwm_left), left_direction
        )

        updated_right_PWM = self.driver.update_pwm_step(
            self.driver.right_motor_id, abs(target_pwm_right), right_direction
        )

        if left_done and right_done:
            self.driver.status = f"Reached target PWM: ({target_pwm_left}, {target_pwm_right})"
        else:
            self.driver.status = f"Adjusting PWM: left={self.driver.left_pwm}, right={self.driver.right_pwm}"
        
        self.get_logger().info(self.driver.status)

        # Stop motors if target is zero
        if target_pwm_left == 0 and target_pwm_right == 0:
            self.driver.stop()

    def destroy_node(self):
        self.driver.stop()
        super().destroy_node()

def main():
    rclpy.init()
    mobility_node = Mobility()
    try:
        rclpy.spin(mobility_node)
    except KeyboardInterrupt:
        pass
    finally:
        mobility_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()