from gpiozero import Servo
from time import sleep

servo = Servo(24)
servo1 = Servo(12)

while True:
    servo.min()
    sleep(2)
    servo.mid()
    sleep(2)
    servo.max()
    sleep(2)
    servo1.min()
    sleep(2)
    servo1.mid()
    sleep(2)
    servo1.max()
    sleep(2)

    