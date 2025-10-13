import numpy as np
from gpiozero import DigitalInputDevice
import time
import driver

# GPIO pin configuration for the encoder
RIGHT_PHASE_A = 17  
RIGHT_PHASE_B = 4   
LEFT_PHASE_A = 7
LEFT_PHASE_B = 27

# Encoder parameters
# CPR = 341.2  # Counts Per Revolution
CPR = 341  # Counts Per Revolution
WHEEL_RADIUS = 0.03  # Wheel radius in meters 

# Determines direction of wheel
class GpiozeroDecoder:
    def __init__(self, gpioA, gpioB):
        # Initialise GPIO pins with pull-up resistors and debouncing
        self.phase_a = DigitalInputDevice(gpioA, pull_up=True, bounce_time = 0.01)
        self.phase_b = DigitalInputDevice(gpioB, pull_up=True, bounce_time = 0.01)
        self.gpioA = gpioA
        self.gpioB = gpioB

        # Saving starting state of phase (0 or 1)
        self.levA = self.phase_a.value
        self.levB = self.phase_b.value
        self.lastGpio = None
        self.count = 0

        # Set up interrupt callbacks for both edges
        self.phase_a.when_activated = self._pulse # when phase A is HIGH, call pulse function
        self.phase_a.when_deactivated = self._pulse # when phase A is LOW
        self.phase_b.when_activated = self._pulse
        self.phase_b.when_deactivated = self._pulse

        print(f"levA: {self.levA},  levB: {self.levB}, count: {self.count}")

    def _pulse(self, _):
        
        # Determines direction of wheel and wraps the count to counts per revolution

        levA = self.phase_a.value
        levB = self.phase_b.value
        currentGpio = self.gpioA if levA != self.levA else self.gpioB # If phase A changed position

        print('pulse', currentGpio, levA, levB)
        if currentGpio != self.lastGpio:  # debounce
            self.lastGpio = currentGpio

            if currentGpio == self.gpioA:
                if levA == 1:
                    if levB == 1:
                        self.count += 1
                    else:
                        self.count -= 1
                else:  # levA == 0
                    if levB == 0:
                        self.count += 1
                    else:
                        self.count -= 1
            else:  # currentGpio == self.gpioB
                if levB == 1:
                    if levA == 1:
                        self.count -= 1
                    else:
                        self.count += 1
                else:  # levB == 0
                    if levA == 0:
                        self.count -= 1
                    else:
                        self.count += 1
            print("count1", self.count)
            self.count = self.count_wrap(self.count)
            print("count2", self.count)
        
        self.levA = levA
        self.levB = levB

        print(f"Count: {self.get_count()}, Angle: {self.get_angle():.4f} rad")

    def get_angle(self):
        # 0 to 2pi
        return self.count * 2 * np.pi / CPR

    def get_count(self):
        return self.count

    def cancel(self):
        # Frees pins
        self.phase_a.close()
        self.phase_b.close()

    @staticmethod
    def count_wrap(count):
        return (count % CPR + CPR) % CPR

# Determines speed of wheel
class EncoderReader:
    def __init__(self):
        self.last_wheel_pos = 0.0
        self.last_wheel_time = time.perf_counter()

    @staticmethod
    def angle_wrap_wheel_diff(angle):
        # Wrapping angles from -2pi, 2pi to -pi, pi
        diff = (angle + np.pi) % (2 * np.pi) - np.pi
        return -np.pi if diff == np.pi else diff

    def get_angle_diff(self, current_wheel_angle):
        # Computing how far the wheel has moved via angles
        wheel_diff = current_wheel_angle - self.last_wheel_pos
        wheel_diff = self.angle_wrap_wheel_diff(wheel_diff)
        self.last_wheel_pos = current_wheel_angle
        return wheel_diff

    def calculate_velocity(self, current_wheel_angle):
        # velocity = angle_diff/time_diff
        angle_diff = self.get_angle_diff(current_wheel_angle)
        curr_time = time.perf_counter() # checking how many seconds has passed
        angle_velocity = WHEEL_RADIUS * angle_diff / (curr_time - self.last_wheel_time)
        self.last_wheel_time = curr_time
        return angle_velocity

def main():
    # Initialise encoder and reader
    right_decoder = GpiozeroDecoder(RIGHT_PHASE_A, RIGHT_PHASE_B)
    right_reader = EncoderReader()

    left_decoder = GpiozeroDecoder(LEFT_PHASE_A, LEFT_PHASE_B)
    left_reader = EncoderReader()

    motor = driver.motor(14, 15)
    motor.setSpeed(-50)
    motor.motorWrite()

    try:
        while True:
            print("-----")
            right_count = right_decoder.get_count()
            right_angle = right_decoder.get_angle()
            right_velocity = right_reader.calculate_velocity(right_angle)

            left_count = left_decoder.get_count()
            left_angle = left_decoder.get_angle()
            left_velocity = left_reader.calculate_velocity(left_angle)
            # print(f"Count: {right_count}, Angle: {right_angle:.4f} rad, Velocity: {right_velocity:.4f} m/s")
            print(';;;')
            if right_count >= 341 / 2:
                
                motor.setSpeed(0)
                motor.motorWrite()
                break
            time.sleep(0.02)  # 50 Hz = 1/50 seconds, dictates how fast terminal gets updated
    except KeyboardInterrupt:
        right_decoder.cancel()
        left_decoder.cancel()

if __name__ == "__main__":
    main()