from __future__ import print_function
import curses
import sys
import time

from dfrobot_driver import THIS_BOARD_TYPE, DFRobot_DC_Motor_IIC

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
        self.max_pwm = 100
        self.min_pwm = 0

        # Current PWM values
        self.left_pwm = 0
        self.right_pwm = 0

        # Motor directions
        self.left_dir = self.motor.CW  # left motor forward is CW
        self.right_dir = self.motor.CCW  # right motor forward is CCW

        # Initialize status
        self.status = "Stopped"

    def setMotorPWM(self, motor_id, pwm, direction):
        try:
            pwm = max(self.min_pwm, min(self.max_pwm, int(abs(pwm))))
            self.motor.motor_movement([motor_id], direction, pwm)  # Pass motor_id as list
        except Exception as e:
            print(f"Error setting motor PWM: {e}")

    def set_left_pwm(self, pwm):
        self.left_pwm = pwm
        direction = self.motor.CW if pwm >= 0 else self.motor.CCW  # Forward: CW, Backward: CCW
        self.setMotorPWM(self.left_motor_id, pwm, direction)

    def set_right_pwm(self, pwm):
        self.right_pwm = pwm
        direction = self.motor.CCW if pwm >= 0 else self.motor.CW  # Forward: CCW, Backward: CW
        self.setMotorPWM(self.right_motor_id, pwm, direction)

    def stop(self):
        try:
            self.motor.motor_stop([self.left_motor_id, self.right_motor_id])
            self.left_pwm = 0
            self.right_pwm = 0
            self.status = "Stopped"
        except Exception as e:
            print(f"Error stopping motors: {e}")

    def robotForward(self, speed):
        try:
            speed = max(self.min_pwm, min(self.max_pwm, int(abs(speed))))
            if speed == 0:
                self.stop()
                return
            self.set_left_pwm(speed)   # Positive: CW (forward for left)
            self.set_right_pwm(speed)  # Positive: CCW (forward for right)
            self.status = f"Moving forward at {speed}%"
            print(f"Robot moving forward at {speed}% PWM")
        except Exception as e:
            print(f"Error in robotForward: {e}")
            self.status = "Error"

    def robotBackward(self, speed):
        try:
            speed = max(self.min_pwm, min(self.max_pwm, int(abs(speed))))
            if speed == 0:
                self.stop()
                return
            self.set_left_pwm(-speed)   # Negative: CCW (backward for left)
            self.set_right_pwm(-speed)  # Negative: CW (backward for right)
            self.status = f"Moving backward at {speed}%"
            print(f"Robot moving backward at {speed}% PWM")
        except Exception as e:
            print(f"Error in robotBackward: {e}")
            self.status = "Error"

    def robotTurnLeft(self, speed):
        try:
            speed = max(self.min_pwm, min(self.max_pwm, int(abs(speed))))
            if speed == 0:
                self.stop()
                return
            self.set_left_pwm(-speed)   # Backward
            self.set_right_pwm(speed)   # Forward
            self.status = f"Turning left at {speed}%"
            print(f"Robot turning left at {speed}% PWM")
        except Exception as e:
            print(f"Error in robotTurnLeft: {e}")
            self.status = "Error"
   
    def robotTurnRight(self, speed):
        try:
            speed = max(self.min_pwm, min(self.max_pwm, int(abs(speed))))
            if speed == 0:
                self.stop()
                return
            self.set_left_pwm(speed)    # Forward
            self.set_right_pwm(-speed)  # Backward
            self.status = f"Turning right at {speed}%"
            print(f"Robot turning right at {speed}% PWM")
        except Exception as e:
            print(f"Error in robotTurnRight: {e}")
            self.status = "Error"

class RobotController:
    def __init__(self):
        self.driver = MotorDriver(addr=0x10, bus_id=1)  # Initialize motor driver
        self.currentSpeed = 0.5  # Default speed (0.0 to 1.0, scaled to 0–100 PWM)

    def run(self, stdscr):
        curses.noecho()  # Don't echo keypresses
        curses.cbreak()  # Immediate input
        stdscr.keypad(True)  # Enable arrow keys
        stdscr.timeout(100)  # Non-blocking input with 100ms timeout

        status = self.driver.status  # Initial status from driver
        last_key = -1

        actions = {
            curses.KEY_UP: lambda: self.driver.robotForward(self.currentSpeed * self.driver.max_pwm),
            curses.KEY_DOWN: lambda: self.driver.robotBackward(self.currentSpeed * self.driver.max_pwm),
            curses.KEY_LEFT: lambda: self.driver.robotTurnLeft(self.currentSpeed * self.driver.max_pwm),
            curses.KEY_RIGHT: lambda: self.driver.robotTurnRight(self.currentSpeed * self.driver.max_pwm),
            ord('w'): lambda: self.driver.robotForward(self.currentSpeed * self.driver.max_pwm),
            ord('a'): lambda: self.driver.robotTurnLeft(self.currentSpeed * self.driver.max_pwm),
            ord('s'): lambda: self.driver.robotBackward(self.currentSpeed * self.driver.max_pwm),
            ord('d'): lambda: self.driver.robotTurnRight(self.currentSpeed * self.driver.max_pwm)
        }

        status_messages = {
            curses.KEY_UP: "Moving forward",
            curses.KEY_DOWN: "Moving backward",
            curses.KEY_LEFT: "Turning left",
            curses.KEY_RIGHT: "Turning right",
            ord('w'): "Moving forward",
            ord('a'): "Turning left",
            ord('s'): "Moving backward",
            ord('d'): "Turning right"
        }

        key_names = {
            curses.KEY_UP: "UP",
            curses.KEY_DOWN: "DOWN",
            curses.KEY_LEFT: "LEFT",
            curses.KEY_RIGHT: "RIGHT",
            ord('w'): "w",
            ord('a'): "a",
            ord('s'): "s",
            ord('d'): "d",
            ord('v'): "v",
            ord('q'): "q"
        }

        while True:
            stdscr.clear()
            try:
                stdscr.addstr(1, 0, "Controls: UP/w = forward, DOWN/s = backward, LEFT/a = left, RIGHT/d = right")
                stdscr.addstr(2, 0, "V = change velocity, Q = quit")
                stdscr.addstr(3, 0, f"Current speed: {self.currentSpeed:.1f} (PWM: {int(self.currentSpeed * self.driver.max_pwm)})")
                stdscr.addstr(4, 0, f"Status: {self.driver.status}")
                stdscr.addstr(5, 0, f"Last key: {key_names.get(last_key, 'Unknown')}")
            except curses.error:
                pass  # Ignore window bounds errors
            stdscr.refresh()

            key = stdscr.getch()
            last_key = key
            print(f"Key pressed: {key_names.get(key, key)}")

            if key == -1:  # No key pressed
                self.driver.stop()
                status = "Stopped"
                continue

            if key == ord('q'):
                break
            elif key == ord('v'):
                stdscr.clear()
                stdscr.addstr(0, 0, "Enter speed (0.0 to 1.0): ")
                stdscr.refresh()
                curses.echo()  # Enable echo for input
                stdscr.timeout(-1)  # Blocking input for getstr
                try:
                    desired_speed = stdscr.getstr().decode("utf-8")
                    new_speed = float(desired_speed)
                    if 0.0 <= new_speed <= 1.0:
                        self.currentSpeed = new_speed
                        status = f"Speed changed to {self.currentSpeed:.1f}"
                    else:
                        status = "Invalid: Speed must be 0 to 1"
                except ValueError:
                    status = "Invalid: Enter a number"
                curses.noecho()  # Disable echo
                stdscr.timeout(100)  # Restore non-blocking
            else:
                action = actions.get(key)
                if action is not None:
                    try:
                        action()
                        status = status_messages.get(key, "Stopped")
                    except Exception as e:
                        self.driver.status = f"Error: {e}"
                        stdscr.addstr(6, 0, f"Action error: {e}")
                        print(f"Motor action error: {e}")

        self.driver.stop()

if __name__ == "__main__":
    curses.wrapper(RobotController().run)