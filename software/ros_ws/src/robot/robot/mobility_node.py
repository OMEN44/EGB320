import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from robot.mobility.driver import motor
import time

MAX_FORWARD_VEL = 0.5  # m/s
WHEEL_BASE = 0.14      # meters

class Mobility(Node):
    def __init__(self):
        super().__init__('mobility_node')
        self.get_logger().info('Mobility node has been started.')

        # Node variables
        self.leftMotor = motor(18, 23)  # PWM pin 18, direction pin 23
        self.rightMotor = motor(14, 15) # PWM pin 14, direction pin 15
        self.flipLeft = -1
        self.lastTime = time.time()
        self.targetPwm = [0, 0]  # [left, right]
        self.currentPwm = [0, 0] # [left, right]
        self.dt = 0.005
        self.count = 0
        self.sign = 1

        # Create 100ms timer
        self.mobilityTimer = self.create_timer(self.dt, self.timer_callback)
        # self.testingTimer = self.create_timer(.4, self.test)

        # Initialise subscribers
        self.subscription = self.create_subscription(TwistStamped,'/cmd_vel', self.cmdCallBack, 10)
        self.subscription2 = self.create_subscription(TwistStamped,'/web_vel', self.cmdCallBack, 10)

    def test(self):
        self.targetPwm[0] = self.count
        self.targetPwm[1] = self.count
        self.count += self.sign * 20
        if self.count == 100 or self.count == -100:
            self.sign *= -1

    
    def twist_to_pwm(self, twist):
        v = twist.linear.x        # m/s
        w = twist.angular.z *2      # rad/s

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
            # self.get_logger().info('Target PWM: left=%d, right=%d, current left=%d, right=%d' % (self.targetPwm[0], self.targetPwm[1], self.currentPwm[0], self.currentPwm[1]))
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

    def cmdCallBack(self, msg):
        left_pwm, right_pwm = self.twist_to_pwm(msg.twist)
        self.targetPwm[0] = int(left_pwm * 100)   # Scale to [-100, 100]
        self.targetPwm[1] = int(right_pwm * 100)  # Scale to [-100, 100]
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