from gpiozero import AngularServo
from time import sleep

# Gripper Actuation (SG90 positional servo, not FS90R)
servo3 = AngularServo(
    13,  # GPIO23
    min_angle=-90,  # Open position
    max_angle=90,   # Closed position
    min_pulse_width=0.0005,  # 0.5ms for full open
    max_pulse_width=0.0025   # 2.5ms for full closed
)

# def smooth_move(servo1_target, servo2_target, step=0.01, delay=0.001):
#     current_servo1 = servo1.angle if servo1.angle is not None else 0
#     current_servo2 = servo2.angle if servo2.angle is not None else 0
    
#     # Calculate the number of steps needed
#     max_steps = max(
#         abs(servo1_target - current_servo1),
#         abs(servo2_target - current_servo2)
#     ) / step
#     max_steps = int(max_steps) + 1 if max_steps > 0 else 1

#     # Calculate step increments for each servo
#     servo1_step = (servo1_target - current_servo1) / max_steps
#     servo2_step = (servo2_target - current_servo2) / max_steps

#     # Move servos incrementally
#     for _ in range(max_steps):
#         current_servo1 += servo1_step
#         current_servo2 += servo2_step
#         servo1.angle = current_servo1
#         servo2.angle = current_servo2
#         sleep(delay)

#     # Ensure final position is exact
#     servo1.angle = servo1_target
#     servo2.angle = servo2_target
#     sleep(1)  # Brief pause at final position

# def idle_position():
#     print('Idle Position')
#     smooth_move(servo1_target=-48, servo2_target=90)
#     # servo3.angle = -90  # Open gripper
#     sleep(1)

# def picking_bay_collect():
#     print('Picking Bay Collection')
    
#     # Open gripper
#     print('Opening Gripper')
#     # servo3.angle = -90  # Full open (~0.5ms pulse)
#     # sleep(1)
    
#     # Move to picking bay position
#     smooth_move(servo1_target=-45, servo2_target=0)
    
#     # Close gripper to pick up item
#     # print('Closing Gripper')
#     # servo3.angle = 90   # Full closed (~2.5ms pulse)
#     sleep(1)

# def low_shelf():
#     # servo3.angle = 90  # Close gripper
#     print('Low Shelf')
#     smooth_move(servo1_target=-28.95, servo2_target=3)
#     # servo3.angle = -90  # Open gripper
#     sleep(1)

# def middle_shelf():
#     # servo3.angle = 90  # Close gripper
#     print('Middle Shelf')
#     smooth_move(servo1_target=3, servo2_target=-20)
#     # servo3.angle = -90  # Open gripper
#     sleep(1)

# def high_shelf():
#     # servo3.angle = 90  # Open gripper
#     print('High Shelf')
#     smooth_move(servo1_target=40, servo2_target=-60)
#     # servo3.angle = -90  # Open gripper
#     sleep(1)

while True:
    user_input = input("Enter 0 (Do nothing), 1 (Open), 2 (Close), or q to quit: ")

    if user_input == "0":
        print("Doing nothing...")
        sleep(1)
    elif user_input == "1":
        servo3.angle = 90 # Open
        sleep(1)
    elif user_input == '2':
        servo3.angle = -90 # Close
        sleep(1)

    
    # user_input = input("Enter 0 (Do nothing), 1 (Idle), 2 (Low Shelf), 3 (Middle Shelf), 4 (High Shelf), 5 (Picking Bay Collect), or q to quit: ")

    # if user_input == "0":
    #     print("Doing nothing...")
    #     sleep(1)

    # elif user_input == "1":
    #     servo3.angle = 90 # Open
    #     sleep(1)
    #     idle_position()
    #     sleep(1)

    # elif user_input == "2":
    #     low_shelf()
    #     sleep(1)
    #     servo3.angle = 90 # Open
    #     sleep(1)

    # elif user_input == "3":
    #     # servo3.angle = -90 # Close
    #     # sleep(1)
    #     middle_shelf()
    #     sleep(1)
    #     servo3.angle = 90 # Open
    #     sleep(1)


    # elif user_input == "4":
    #     # servo3.angle = -90 # Close
    #     # sleep(1)
    #     high_shelf()
    #     sleep(1)
    #     servo3.angle = 90 # Open
    #     sleep(1)

    # elif user_input == "5":
    #     servo3.angle = 90 # Open
    #     sleep(1)
    #     picking_bay_collect()
    #     sleep(1)
    #     servo3.angle = -90 # Close
    #     sleep(1)


    # elif user_input.lower() == "q":
    #     print("Exiting program.")
    #     servo1.close()
    #     servo2.close()
    #     servo3.close()
    #     break
