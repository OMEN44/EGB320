from gpiozero import AngularServo
from time import sleep

# At Base (servo1, controls base rotation)
servo1 = AngularServo(
    24,  # GPIO24
    min_angle=-90,
    max_angle=90,
    min_pulse_width=0.0005,  # 500µs
    max_pulse_width=0.0024   # 2500µs (adjusted to standard value)
)

servo3 = AngularServo(
    23,  # GPIO23
    min_angle=-90,  # Open position
    max_angle=90,   # Closed position
    min_pulse_width=0.0005,  # 0.5ms for full open
    max_pulse_width=0.0025   # 2.5ms for full closed
)

def smooth_move(servo1_target, step=1, delay=0.1):
    # Ensure target is within servo's angle range
    servo1_target = max(min(servo1_target, 90), -90)
    
    current_servo1 = servo1.angle if servo1.angle is not None else 0
    
    # Calculate the number of steps needed
    max_steps = abs(servo1_target - current_servo1) / step
    max_steps = int(max_steps) + 1 if max_steps > 0 else 1

    # Calculate step increments for servo
    servo1_step = (servo1_target - current_servo1) / max_steps

    # Move servo incrementally
    for _ in range(max_steps):
        current_servo1 += servo1_step
        servo1.angle = current_servo1
        sleep(delay)

    # Ensure final position is exact
    servo1.angle = servo1_target
    sleep(1)  # Brief pause at final position

def open():
    servo3.angle = -90 # Open

def open():
    servo3.angle = 90 # Closed


def top_shelf():
    # Top Shelf
    smooth_move(50, step=0.1, delay=0.02)
    sleep(1)

def middle_shelf():
    # Middle Shelf
    smooth_move(-20, step=0.1, delay=0.02)
    sleep(1)

def bottom_shelf():
    # Bottom Shelf
    smooth_move(-65, step=0.1, delay=0.02)
    sleep(1)

def floor_level():
    # Floor level
    smooth_move(-85, step=0.1, delay=0.02)
    servo3.angle = 90 # Close
    sleep(1)

def close_gripper():
        servo3.angle = 90 # Close

while True:
    user_input = input("0 (Ground level), 1 (Low Shelf), 2 (Middle Shelf), 3 (High Shelf), q to quit, s to close gripper: ")
    
    if user_input == "s":
        close_gripper
        print("Servo closed")

    
    if user_input == "0":
        servo3.angle = 90
        print("Opening gripper to collect")
        sleep(1)
        floor_level()
        print("Moving to floor level")
        sleep(1)
        servo3.angle = -90
        print("Closing gripper to collect")
        sleep(1)

    elif user_input == "1":
        bottom_shelf()
        print("Moving to bottom shelf")
        sleep(1)
        servo3.angle = 90 # Open
        print("Open gripper to drop, shelf 1")
        sleep(1)

    elif user_input == "2":
        middle_shelf()
        print("Moving to middle shelf")
        sleep(1)
        servo3.angle = 90 # Open
        print("Open gripper to drop, shelf 2")
        sleep(1)

    elif user_input == "3":
        top_shelf()
        print("Moving to top shelf")        
        sleep(1)
        servo3.angle = 90 # Open
        print("Open gripper to drop, shelf 3")
        sleep(1)

    elif user_input.lower() == "q":
        print("Exiting program.")
        servo1.close()
        servo2.close()
        servo3.close()
        break
