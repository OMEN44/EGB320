# --------------------------------------------
# Coninuous servo
# from gpiozero import Servo
# from time import sleep

# # Continuous rotation servo on GPIO24 (change to your pin if needed)
# servo = Servo(
#     12,
#     min_pulse_width=0.0005,
#     max_pulse_width=0.0025
# )

# # Spin clockwise for 5 seconds
# servo.max()  # full speed clockwise
# sleep(0.2)

# servo.mid()


# servo.min()  # full speed clockwise
# sleep(0.2)


# # Stop the servo
# servo.mid()
# -------------------------------

from gpiozero import Servo
from time import sleep

# FS90R Continuous rotation servo on GPIO12
servo = Servo(
    12,
    min_pulse_width=0.0005,
    max_pulse_width=0.0025
)

# def smooth_speed(start, end, step, delay):
#     stepSize = (end - start) / step
#     stepCount = (end - start) / stepSize
#     stepTime = delay / stepCount

#     value = start
#     servo.value = value
#     while (value < end):
#         value += stepSize
#         servo.value = max(min(value, 0.2), -0.2)
#         sleep(stepTime)
#     servo.value = end

# Example motion sequence

# print("Ramping forward...")
# smooth_speed(0.0, 0.2, step=0.1, delay=0.1)  # accelerate forward
# sleep(1)

print("Rotating CW...")
servo.value = -0.1
sleep(0.9) # Rotate for 3 seconds

