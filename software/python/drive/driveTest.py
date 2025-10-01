from gpiozero import PWMOutputDevice, DigitalOutputDevice
import time

dirPinA = 14
pwmPinA = 15

dirPinB = 18
pwmPinB = 23

directionA = DigitalOutputDevice(dirPinA)
directionB = DigitalOutputDevice(dirPinB)
directionA.on()
directionB.on()

pwmA = PWMOutputDevice(pwmPinA, frequency=200)
pwmB = PWMOutputDevice(pwmPinB, frequency=200)

value = 0

while True:
    # Use 0.3 to 1 for duty cycle
    pwmA.value = value / 100
    pwmB.value = value / 100
    print(value, '%')
    value += 10
    if value > 100:
        value = 0

    # directionA.toggle()
    time.sleep(0.5)