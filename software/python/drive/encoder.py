#import numpy as np
import pigpio
import time
import logging

# Configure logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

# GPIO pin configuration for the encoder
RIGHT_PHASE_A = 17  # GPIO 17 for Phase A
RIGHT_PHASE_B = 4   # GPIO 4 for Phase B

# Encoder parameters
#PPR = 374  # Pulses Per Revolution
CPR = 341.2  # Counts Per Revolution
WHEEL_RADIUS = 0.0  # Wheel radius in meters (adjust as needed)

class PigpioDecoder:
    def __init__(self, gpioA, gpioB):
        self.pi = pigpio.pi('localhost', 8888)  # Explicitly connect to localhost on port 8888
        if not self.pi.connected:
            raise Exception("Failed to connect to pigpio daemon")
        self.gpioA = gpioA
        self.gpioB = gpioB

        self.levA = 0
        self.levB = 0
        self.lastGpio = None
        self.count = 0

        # Set GPIO modes and pull-up resistors
        self.pi.set_mode(self.gpioA, pigpio.INPUT)
        self.pi.set_mode(self.gpioB, pigpio.INPUT)
        self.pi.set_pull_up_down(self.gpioA, pigpio.PUD_UP)
        self.pi.set_pull_up_down(self.gpioB, pigpio.PUD_UP)

        # Set up callbacks for both phases
        self.cbA = self.pi.callback(self.gpioA, pigpio.EITHER_EDGE, self._pulse)
        self.cbB = self.pi.callback(self.gpioB, pigpio.EITHER_EDGE, self._pulse)

    def _pulse(self, gpio, level, tick):
        """
        Decode the rotary encoder pulse on interrupts, then angle wrap it.
        """
        if gpio == self.gpioA:
            self.levA = level
        else:
            self.levB = level

        if gpio != self.lastGpio:  # debounce
            self.lastGpio = gpio

            if gpio == self.gpioA:
                if level == 1:
                    if self.levB == 1:
                        self.count += 1
                    else:
                        self.count -= 1
                else:  # level == 0
                    if self.levB == 0:
                        self.count += 1
                    else:
                        self.count -= 1
            else:  # gpio == self.gpioB
                if level == 1:
                    if self.levA == 1:
                        self.count -= 1
                    else:
                        self.count += 1
                else:  # level == 0
                    if self.levA == 0:
                        self.count -= 1
                    else:
                        self.count += 1
            self.count = self.count_wrap(self.count)

    def get_angle(self):
        """Return angle in [0, 2pi) after count wrap [0, CPR)."""
        return self.count * 2 * 3.14 / CPR #replace 3.14 with np.pi

    def cancel(self):
        """Cancel the callbacks."""
        self.cbA.cancel()
        self.cbB.cancel()

    @staticmethod
    def count_wrap(count):
        """Wrap count to [0, CPR)."""
        return count % CPR

class EncoderReader:
    def __init__(self):
        self.last_wheel_pos = 0.0
        self.last_wheel_time = time.perf_counter()
        logger.info("EncoderReader has been initialized")

    @staticmethod
    def angle_wrap_wheel_diff(angle):
        """Input angle: [-2pi, 2pi), wrapping angles to [-pi, pi)."""
        diff = (angle + 3.14) % (2 * 3.14) - 3.14
        return -3.14 if diff == 3.14 else diff #replace 3.14 with np.pi

    def get_angle_diff(self, current_wheel_angle):
        """Compute angle-wrapped wheel difference."""
        wheel_diff = current_wheel_angle - self.last_wheel_pos
        wheel_diff = self.angle_wrap_wheel_diff(wheel_diff)
        self.last_wheel_pos = current_wheel_angle
        return wheel_diff

    def calculate_velocity(self, current_wheel_angle):
        """Calculate and return wheel velocity based on the current wheel angle."""
        angle_diff = self.get_angle_diff(current_wheel_angle)
        curr_time = time.perf_counter()
        angle_velocity = WHEEL_RADIUS * angle_diff / (curr_time - self.last_wheel_time)
        self.last_wheel_time = curr_time
        logger.debug(f"Encoder velocity: {angle_velocity:.4f} m/s")
        return angle_velocity

def main():
    # Initialize encoder and reader
    decoder = PigpioDecoder(RIGHT_PHASE_A, RIGHT_PHASE_B)
    reader = EncoderReader()

    try:
        # Run at approximately 50 Hz
        while True:
            angle = decoder.get_angle()
            velocity = reader.calculate_velocity(angle)
            print(f"Angle: {angle:.4f} rad, Velocity: {velocity:.4f} m/s")
            time.sleep(0.02)  # 50 Hz = 1/50 seconds
    except KeyboardInterrupt:
        logger.info("Shutting down")
        decoder.cancel()
        decoder.pi.stop()

if __name__ == "__main__":
    main()