# from gpiozero import AngularServo
# from time import sleep

# # At Base
# servo1 = AngularServo(
#     24,  # GPIO24
#     min_angle=-90,
#     max_angle=90,
#     min_pulse_width=0.0005,
#     max_pulse_width=0.0025
# )
# # At Gripper Attachment
# servo2 = AngularServo(
#     12,  # GPIO12
#     min_angle=-90,
#     max_angle=90,
#     min_pulse_width=0.0005,
#     max_pulse_width=0.0025
# )

# # # Gripper Actuation
# # servo3 = AngularServo(
# #     18, #GPIO18
# #     print("Rotating CW")
# #     servo.value = -0.1
# #     sleep(0.9)
# # )


# def idle_position():
#     print('Idle Position')
#     servo1.angle = -50
#     servo2.angle = 90
#     sleep(2)

# def low_shelf():
#     print('Low Shelf')
#     servo1.angle = -20
#     servo2.angle = 35
#     sleep(2)

# def middle_shelf():
#     print('Middle Shelf')
#     servo1.angle = 7
#     servo2.angle = -5
#     sleep(2)

# def high_shelf():
#     print('High Shelf')
#     servo2.angle = -30
#     servo1.angle = 30
#     sleep(2)

# def actuate_gripper(): 
#     print('Actuate Gripper')
    

    

# # while True:
# #     idle_position()
# #     low_shelf()
# #     middle_shelf()
# #     high_shelf()
# #     actuate_gripper()

# while True:
    
#     user_input = input("Enter 1 (Idle), 2 (Low Shelf), 3 (Middle Shelf), 4 (High Shelf), or q to quit: ")

#     # if user_input == "1":
#     #     idle_position()
#     # elif user_input == "2":
#     #     low_shelf()
#     # elif user_input == "3":
#     #     middle_shelf()
#     # elif user_input == "4":
#     #     high_shelf()
#     # elif user_input.lower() == "q":
#     #     print("Exiting program.")
#     #     break

#     # 0 is do nothing
#     # 1 is ground level
#     # 2 is bottom shelf
#     # 3 is middle shelf
#     # 4 is top shelf
#     # 5 is actuate gripper?
#     # feedback response Ros2

#     if user_input == "0":
#         print("Doing nothing...")
#         sleep(1)

#     elif user_input == "1":
#         idle_position()

#     elif user_input == "2":
#         low_shelf()

#     elif user_input == "3":
#         middle_shelf()

#     elif user_input == "4":
#         high_shelf()

#     elif user_input == "5":
#         # actuate_gripper()
#         print('Actuate Gripper')
#     elif user_input.lower() == "q":
#         print("Exiting program.")
#         break


# ---------------------------------------
# Slow movement code
from gpiozero import AngularServo
from time import sleep

# At Base
servo1 = AngularServo(
    24,  # GPIO24
    min_angle=-90,
    max_angle=90,
    min_pulse_width=0.0005,
    max_pulse_width=0.0025
)
# At Gripper Attachment
servo2 = AngularServo(
    12,  # GPIO12
    min_angle=-90,
    max_angle=90,
    min_pulse_width=0.0005,
    max_pulse_width=0.0025
)

# Gripper Actuation (Continuous Rotation Servo FS90R)
servo3 = AngularServo(
    23,  # GPIO18
    min_angle=-90,
    max_angle=90,
    min_pulse_width=0.0005,
    max_pulse_width=0.0025
)

def smooth_move(servo1_target, servo2_target, step=0.2, delay=0.005):
    current_servo1 = servo1.angle if servo1.angle is not None else 0
    current_servo2 = servo2.angle if servo2.angle is not None else 0
    
    # Calculate the number of steps needed
    max_steps = max(
        abs(servo1_target - current_servo1),
        abs(servo2_target - current_servo2)
    ) / step
    max_steps = int(max_steps) + 1 if max_steps > 0 else 1

    # Calculate step increments for each servo
    servo1_step = (servo1_target - current_servo1) / max_steps
    servo2_step = (servo2_target - current_servo2) / max_steps

    # Move servos incrementally
    for _ in range(max_steps):
        current_servo1 += servo1_step
        current_servo2 += servo2_step
        servo1.angle = current_servo1
        servo2.angle = current_servo2
        sleep(delay)

    # Ensure final position is exact
    servo1.angle = servo1_target
    servo2.angle = servo2_target
    sleep(1)  # Brief pause at final position

def idle_position():
    print('Idle Position')
    smooth_move(servo1_target=-50, servo2_target=90)

def picking_bay_collect():
    print('Picking Bay Collection')
    smooth_move(servo1_target=-45, servo2_target=20)

def low_shelf():
    print('Low Shelf')
    smooth_move(servo1_target=-20, servo2_target=20)

def middle_shelf():
    print('Middle Shelf')
    smooth_move(servo1_target=10, servo2_target=-15)

def high_shelf():
    print('High Shelf')
    smooth_move(servo1_target=45, servo2_target=-49)

def gripper_clockwise(duration=0.10):
    print('Rotating Gripper Clockwise')
    servo3.value = -0.5  # Adjust speed as needed (-1 for full speed clockwise)
    sleep(duration)
    servo3.value = 0.1  # Stop
    print('Gripper Stopped')

def gripper_anticlockwise(duration=0.1):
    print('Rotating Gripper Anticlockwise')
    servo3.value = 0.5  # Adjust speed as needed (1 for full speed anticlockwise)
    sleep(duration)
    servo3.value = 0.1  # Stop
    print('Gripper Stopped')

while True:
    user_input = input("Enter 0 (Do nothing), 1 (Idle), 2 (Low Shelf), 3 (Middle Shelf), 4 (High Shelf), 5 (Actuate Gripper), or q to quit: ")

    if user_input == "0":
        print("Doing nothing...")
        sleep(1)
    elif user_input == "1":
        idle_position()
    elif user_input == "2":
        low_shelf()
    elif user_input == "3":
        middle_shelf()
    elif user_input == "4":
        high_shelf()
    elif user_input == "5":
        gripper_clockwise()
    elif user_input == "6":
        gripper_anticlockwise()
    elif user_input == "7":
        picking_bay_collect()
    elif user_input.lower() == "q":
        print("Exiting program.")
        break