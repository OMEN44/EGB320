import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from robot.mobility.driver import motor
import time

MAX_FORWARD_VEL = 0.5  # m/s
WHEEL_BASE = 0.23      # meters

def twist_to_pwm(twist):
    v = twist.linear.x        # m/s
    w = twist.angular.z       # rad/s

    # wheel linear velocities (m/s)
    v_r = v + (w * WHEEL_BASE) / 2.0
    v_l = v - (w * WHEEL_BASE) / 2.0

    # scale to [-1, 1] by dividing by MAX_FORWARD_VEL
    pwm_r = v_r / MAX_FORWARD_VEL
    pwm_l = v_l / MAX_FORWARD_VEL

    # clip to [-1, 1]
    pwm_r = max(-1.0, min(1.0, pwm_r))
    pwm_l = max(-1.0, min(1.0, pwm_l))

    return pwm_l, pwm_r

class Mobility(Node):
    def __init__(self):
        super().__init__('mobility_node')
        self.get_logger().info('Mobility node has been started.')

        # Node variables
        self.leftMotor = motor(18, 23)  # PWM pin 18, direction pin 23
        self.rightMotor = motor(14, 15) # PWM pin 14, direction pin 15
        self.lastTime = time.time()
        self.targetPwm = [0, 0]  # [left, right]
        self.currentPwm = [0, 0] # [left, right]
        self.dt = 0.01
        self.count = 0
        self.sign = 1

        # Create 100ms timer
        self.mobilityTimer = self.create_timer(self.dt, self.timer_callback)
        self.testingTimer = self.create_timer(.4, self.test)

        # Initialise subscribers
        self.subscription = self.create_subscription(Twist,'/cmd_vel', self.cmdCallBack, 10)

    def test(self):
        self.targetPwm[0] = self.count
        self.targetPwm[1] = self.count
        self.count += self.sign * 20
        if self.count == 100 or self.count == -100:
            self.sign *= -1

    def timer_callback(self):
        currentTime = time.time()


        if (currentTime - self.lastTime) >= self.dt:
            self.get_logger().info('Target PWM: left=%d, right=%d, current left=%d, right=%d' % (self.targetPwm[0], self.targetPwm[1], self.currentPwm[0], self.currentPwm[1]))
            self.lastTime = currentTime

            # Smoothly update currentPwm towards targetPwm
            for i in range(2):
                if self.currentPwm[i] < self.targetPwm[i]:
                    self.currentPwm[i] += 1
                elif self.currentPwm[i] > self.targetPwm[i]:
                    self.currentPwm[i] -= 1

            # Update motor speeds
            self.leftMotor.setSpeed(self.currentPwm[0])
            self.rightMotor.setSpeed(self.currentPwm[1])
            self.leftMotor.motorWrite()
            self.rightMotor.motorWrite()

    def cmdCallBack(self, msg):
        self.get_logger().info('Received cmd_vel message: linear_x=%f, angular_z=%f' % (msg.linear.x, msg.angular.z))
        

    def destroy_node(self):
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