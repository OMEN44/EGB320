from gpiozero import DigitalInputDevice, RotaryEncoder

ENCODER_A = 17
ENCODER_B = 4

class Encoder:
    def __init__(self, gpioA, gpioB):
        self.count = 0
        self.gpioA = gpioA
        self.gpioB = gpioB
        self.phase_a = DigitalInputDevice(gpioA, pull_up=True)
        self.phase_b = DigitalInputDevice(gpioB, pull_up=True)
        self.levA = self.phase_a.value
        self.levB = self.phase_b.value
        self.lastGpio = None

        self.phase_a.when_activated = self._callback
        self.phase_a.when_deactivated = self._callback
        self.phase_b.when_activated = self._callback
        self.phase_b.when_deactivated = self._callback

        self.order = [0b01, 0b11, 0b01, 0b00, 0b01, 0b11]

    def _callback(self, _):
        levA = self.phase_a.value
        levB = self.phase_b.value
        currentGpio = self.gpioA if levA != self.levA else self.gpioB # If phase A changed position


        if currentGpio == ENCODER_A:
            self.count ^= 0b10
        else:  # currentGpio == ENCODER_B
            self.count ^= 0b01

        print(bin(self.count))

import time
import driver

# encoder = Encoder(ENCODER_A, ENCODER_B)
motorA = driver.motor(14, 15)
motorA.setSpeed(50)
motorA.motorWrite()

while True:
    try:
        # print("Count A:", encoder.countA, "Count B:", encoder.countB)
        pass
        # time.sleep(1)
    except KeyboardInterrupt:
        # lgpio.gpiochip_close(chip)
        break
