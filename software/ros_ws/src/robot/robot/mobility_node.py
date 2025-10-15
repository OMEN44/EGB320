import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped, Twist
from std_msgs.msg import Float64MultiArray
from robot.mobility.driver import motor
import time

from PiicoDev_MPU6050 import PiicoDev_MPU6050

MAX_FORWARD_VEL = 0.5  # m/s
WHEEL_BASE = 0.34      # meters

class Mobility(Node):
    def __init__(self):
        super().__init__('mobility_node')
        self.get_logger().info('Mobility node has been started.')

        self.gain = [1.0, 1.1]  # [left, right] motor gain adjustments

        # Node variables
        self.leftMotor = motor(15, 14)  # PWM pin 15, direction pin 14    Channel B
        self.rightMotor = motor(23, 18) # PWM pin 23, direction pin 18    Channel A
        self.flipLeft = -1
        self.lastTime = time.time()
        self.targetPwm = [0, 0]  # [left, right]
        self.currentPwm = [0, 0] # [left, right]
        self.dt = 0.00008
        self.count = 0
        self.sign = 1

        # Create 100ms timer
        self.mobilityTimer = self.create_timer(self.dt, self.timer_callback)

        # Initialise subscribers
        self.subscription = self.create_subscription(Twist, '/cmd_vel', self.cmdCallBack, 10)
        self.subscription2 = self.create_subscription(TwistStamped, '/web_vel', self.cmdCallBack, 10)
        self.subscriptionSkid = self.create_subscription(Float64MultiArray, '/skid_cmd_vel', self.skidCallback, 10)
    
    def twist_to_pwm(self, twist):
        v = twist.linear.x        # m/s
        w = twist.angular.z * 2   # rad/s

        # wheel linear velocities (m/s)
        v_r = v + (w * WHEEL_BASE) / 2.0
        v_l = v * 2 - v_r
        # self.get_logger().info('Wheel velocities: v_l=%.2f m/s, v_r=%.2f m/s' % (v_l, v_r))

        # scale to [-1, 1] by dividing by MAX_FORWARD_VEL
        pwm_r = v_r / MAX_FORWARD_VEL
        pwm_l = v_l / MAX_FORWARD_VEL

        # clip to [-1, 1]
        pwm_r = max(-1.0, min(1.0, pwm_r))
        pwm_l = max(-1.0, min(1.0, pwm_l))

        return pwm_l, pwm_r

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
            self.leftMotor.setSpeed(self.flipLeft * self.currentPwm[0])
            self.rightMotor.setSpeed(self.currentPwm[1])
            self.leftMotor.motorWrite()
            self.rightMotor.motorWrite()

    def skidCallback(self, msg):
        left_pwm = msg.data[0]
        right_pwm = msg.data[1]
        self.targetPwm[0] = int(left_pwm * self.gain[0] * 100)   # Scale to [-100, 100]
        self.targetPwm[1] = int(right_pwm * self.gain[1] * 100)  # Scale to [-100, 100]
        # self.get_logger().info('Received skid cmd_vel: left=%.2f, right=%.2f' % (left_pwm, right_pwm))
        # self.get_logger().info('Converted to PWM: left=%d, right=%d' % (self.targetPwm[0], self.targetPwm[1]))

    def cmdCallBack(self, msg):
        left_pwm = 0
        right_pwm = 0
        if msg.__class__ == TwistStamped:
            left_pwm, right_pwm = self.twist_to_pwm(msg.twist)
        else:
            left_pwm, right_pwm = self.twist_to_pwm(msg)
        self.targetPwm[0] = int(left_pwm * self.gain[0] * 100)   # Scale to [-100, 100]
        self.targetPwm[1] = int(right_pwm * self.gain[1] * 100)  # Scale to [-100, 100]
        # self.get_logger().info('Received cmd_vel: linear.x=%.2f, angular.z=%.2f' % (msg.twist.linear.x, msg.twist.angular.z))
        # self.get_logger().info('Converted to PWM: left=%d, right=%d' % (self.targetPwm[0], self.targetPwm[1]))
        

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