# from gpiozero import Servo
# from time import sleep

# servo = Servo(24)
# servo1 = Servo(12)

# while True:
#    servo.min()
#    sleep(2)
#    servo.mid()
#    sleep(2)
#    servo.max()
#    sleep(2)
#    servo1.min()
#    sleep(2)
#    servo1.mid()
#    sleep(2)
#    servo1.max()
#    sleep(2)

# -----------------------------------------
# from gpiozero import AngularServo
# from time import sleep

# servo = AngularServo(12, min_angle=-90, max_angle=90)

# while True:
#    servo.angle = -90
#    sleep(2)
#    servo.angle = -45
#    sleep(2)
#    servo.angle = 0
#    sleep(2)
#    servo.angle = 45
#    sleep(2)
#    servo.angle = 90
#    sleep(2)


# -----------------------------------------
# # Makes it super slow
# from gpiozero import AngularServo
# from time import sleep

# servo1 = AngularServo(
#     24,
#     min_angle=-90,
#     max_angle=90,
#     min_pulse_width=0.0005,
#     max_pulse_width=0.0025
# )

# servo2 = AngularServo(
#     12,
#     min_angle=-90,
#     max_angle=90,
#     min_pulse_width=0.0005,
#     max_pulse_width=0.0025
# )

# def move_slowly(servo, start_angle, end_angle, step=1, delay=0.02):
#     """
#     Move the servo from start_angle to end_angle in small steps.
#     step: size of each step in degrees
#     delay: pause between steps (controls speed)
#     """
#     if start_angle < end_angle:
#         angles = range(int(start_angle), int(end_angle) + 1, step)
#     else:
#         angles = range(int(start_angle), int(end_angle) - 1, -step)

#     for angle in angles:
#         servo.angle = angle
#         sleep(delay)

# while True:
#     # Move both servos slowly together
#     move_slowly(servo1, servo1.angle or 0, -60)
#     move_slowly(servo2, servo2.angle or 0, 80)
#     sleep(1)

#     move_slowly(servo1, servo1.angle or -60, 0)
#     move_slowly(servo2, servo2.angle or 80, -10)
#     sleep(1)

#     move_slowly(servo1, servo1.angle or 0, 60)
#     move_slowly(servo2, servo2.angle or -10, -90)
#     sleep(1)

## Moving all the parts correctly but too fast
# from gpiozero import AngularServo
# from time import sleep

# # Software PWM version (no pigpio)
# # GPIO24 -> Physical pin 18
# # GPIO12 -> Physical pin 32

# servo1 = AngularServo(
#     24,  # GPIO24
#     min_angle=-90,
#     max_angle=90,
#     min_pulse_width=0.0005,
#     max_pulse_width=0.0025
# )

# servo2 = AngularServo(
#     12,  # GPIO12
#     min_angle=-90,
#     max_angle=90,
#     min_pulse_width=0.0005,
#     max_pulse_width=0.0025
# )


# while True:
#     servo1.angle = -60
#     servo2.angle = 90
#     sleep(2)
    
#     servo1.angle = -30
#     servo2.angle = 15
#     sleep(2)

#     servo1.angle = 0
#     servo2.angle = -30
#     sleep(2)

#     servo1.angle = 60
#     servo2.angle = -70
#     sleep(2)

# -----------------------------
# Making code slower
from gpiozero import AngularServo
from time import sleep

# --- Servo Setup ---
from gpiozero import AngularServo
from time import sleep

# --- Servo Setup ---
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

def smooth_move(target1, target2, step=0.1, delay=0.05):
    """
    Smoothly move both servos to the target angles.
    step: angle change per loop (smaller = smoother, slower)
    delay: time between steps (seconds)
    """
    current1 = servo1.angle if servo1.angle is not None else target1
    current2 = servo2.angle if servo2.angle is not None else target2

    moving = True
    while moving:
        moving = False

        # Move servo1 gradually
        if abs(target1 - current1) > step:
            moving = True
            current1 += step if target1 > current1 else -step
        else:
            current1 = target1

        # Move servo2 gradually
        if abs(target2 - current2) > step:
            moving = True
            current2 += step if target2 > current2 else -step
        else:
            current2 = target2

        servo1.angle = current1
        servo2.angle = current2
        sleep(delay)

# --- Main Loop ---
while True:
    smooth_move(-60, 90)
    sleep(1)

    smooth_move(-30, 15)
    sleep(1)

    smooth_move(0, -30)
    sleep(1)

    smooth_move(60, -70)
    sleep(1)

