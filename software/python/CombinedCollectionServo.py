from gpiozero import AngularServo
from time import sleep

# Software PWM version (no pigpio)
# GPIO24 -> Physical pin 18
# GPIO12 -> Physical pin 32

servo1 = AngularServo(
    24,  # GPIO24
    min_angle=-90,
    max_angle=90,
    min_pulse_width=0.0005,
    max_pulse_width=0.0025
)

servo2 = AngularServo(
    12,  # GPIO12
    min_angle=-90,
    max_angle=90,
    min_pulse_width=0.0005,
    max_pulse_width=0.0025
)


while True:
    print('Idle Position')
    servo1.angle = -60
    servo2.angle = 80
    sleep(2)

    print('Low Shelf')
    servo1.angle = -30
    servo2.angle = 40
    sleep(2)

    print('Middle Shelf')
    servo1.angle = 0
    servo2.angle = -30
    sleep(2)

    print('High Shelf')
    servo2.angle = -89
    servo1.angle = 60
    
    sleep(2)