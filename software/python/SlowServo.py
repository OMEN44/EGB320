# from gpiozero import AngularServo
# from time import sleep

# # --- Servo Setup ---
# servo1 = AngularServo(
#     24,  # GPIO24
#     min_angle=-90,
#     max_angle=90,
#     min_pulse_width=0.0005,
#     max_pulse_width=0.0025
# )

# # servo2 = AngularServo(
# #     12,  # GPIO12
# #     min_angle=-90,
# #     max_angle=90,
# #     min_pulse_width=0.0005,
# #     max_pulse_width=0.0025
# # )

# def slow_move_servo1(target_angle, step=0.5, delay=0.1):
#     current_angle = servo1.angle if servo1.angle is not None else target_angle

#     if target_angle > current_angle:
#         while current_angle < target_angle:
#             current_angle += step
#             if current_angle > target_angle:
#                 current_angle = target_angle
#             servo1.angle = current_angle
#             sleep(delay)
#     else:
#         while current_angle > target_angle:
#             current_angle -= step
#             if current_angle < target_angle:
#                 current_angle = target_angle
#             servo1.angle = current_angle
#             sleep(delay)

# # def slow_move_servo2(target_angle2, step2=0.5, delay=0.02):
# #     current_angle2 = servo2.angle2 if servo2.angle2 is not None else target_angle2

# #     if target_angle2 > current_angle2:
# #         while current_angle2 < target_angle2:
# #             current_angle2 += step2
# #             if current_angle2 > target_angle2:
# #                 current_angle2 = target_angle2
# #             servo1.angle2 = current_angle2
# #             sleep(delay)
# #     else:
# #         while current_angle2 > target_angle2:
# #             current_angle2 -= step2
# #             if current_angle2 < target_angle2:
# #                 current_angle2 = target_angle2
# #             servo1.angle2 = current_angle2
# #             sleep(delay)
            

# slow_move_servo1(45)   # Slowly move to +45 degrees
# # slow_move_servo2(45)
# sleep(1)
# slow_move_servo1(-45)
# # slow_move_servo2(-45)

# # Code works for the one base servo off the battery
# --------------------------------------------------
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

def slow_move_servos(target1, target2, step=0.05, delay=0.02):
    
    current1 = servo1.angle if servo1.angle is not None else target1
    current2 = servo2.angle if servo2.angle is not None else target2

    moving = True
    while moving:
        moving = False

        # --- Servo 1 ---
        if abs(target1 - current1) > step:
            moving = True
            current1 += step if target1 > current1 else -step
        else:
            current1 = target1

        # --- Servo 2 ---
        if abs(target2 - current2) > step:
            moving = True
            current2 += step if target2 > current2 else -step
        else:
            current2 = target2

        # Apply updates
        servo1.angle = current1
        servo2.angle = current2

        sleep(delay)


# --- Example Usage ---
slow_move_servos(45, -40)   # Move servo1 to +45° and servo2 to -30° slowly
sleep(2)
slow_move_servos(-35, 30)   # Move servo1 to -45° and servo2 to +30° slowly
