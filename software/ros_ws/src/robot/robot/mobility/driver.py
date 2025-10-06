from gpiozero import PWMOutputDevice, DigitalOutputDevice
import time
import numpy as np

FORWARD = True
REVERSE = False

class motor:
    def __init__(self, pwmPin, dirPin):
        self.pwmPin = pwmPin
        self.dirPin = dirPin
        self.dirDevice = DigitalOutputDevice(self.dirPin)
        self.pwm = PWMOutputDevice(self.pwmPin, frequency=200)

        self.dutyCycle = 0
        self.direction = FORWARD

    # Speed is a value between -100 and 100
    # Negative values are reverse
    def setSpeed(self, speed):
        if speed > 100:
            speed = 100
        elif speed < -100:
            speed = -100

        if speed < 0:
            self.direction = REVERSE
        else:
            self.direction = FORWARD

        speed = abs(speed)
        
        self.dutyCycle = speed / 100

    def motorWrite(self):
        if self.direction == FORWARD:
            self.dirDevice.on()
        else:
            self.dirDevice.off()

        self.pwm.value = self.dutyCycle

