from gpiozero import AngularServo
from time import sleep

# Note: To reduce servo jitter, consider using the pigpio pin factory.
# Set environment variable: export GPIOZERO_PIN_FACTORY=pigpio
# See https://gpiozero.readthedocs.io/en/stable/api_output.html#servo for details.

# Base servo (servo1, controls base rotation)
servo1 = AngularServo(
    12,  # GPIO12
    min_angle=-90,
    max_angle=90,
    min_pulse_width=0.0005,  # 500µs
    max_pulse_width=0.0024   # 2400µs
)

# Gripper servo (servo3)
servo3 = AngularServo(
    13,  # GPIO13
    min_angle=-90,  # Closed position
    max_angle=90,   # Open position
    min_pulse_width=0.0005,  # 500µs
    max_pulse_width=0.0025   # 2500µs
)

def smooth_move(servo1_target, step=0.1, delay=0.02):
    """Move servo1 to target angle smoothly."""
    # Ensure target is within servo's angle range
    servo1_target = max(min(servo1_target, 90), -90)
    current_servo1 = servo1.angle if servo1.angle is not None else 0
    
    # Calculate the number of steps needed
    max_steps = abs(servo1_target - current_servo1) / step
    max_steps = int(max_steps) + 1 if max_steps > 0 else 1

    # Calculate step increments for servo
    servo1_step = (servo1_target - current_servo1) / max_steps

    # Move servo incrementally, clamping at each step
    for _ in range(max_steps):
        current_servo1 += servo1_step
        # Clamp to valid range to prevent OutputDeviceBadValue
        current_servo1 = max(min(current_servo1, 90), -90)
        servo1.angle = current_servo1
        sleep(delay)

    # Ensure final position is exact
    servo1.angle = servo1_target
    sleep(1)  # Brief pause at final position

def open_gripper():
    """Set gripper to open position (90 degrees)."""
    servo3.angle = 90  # Open
    sleep(1)
    print("Gripper opened")

def close_gripper():
    """Set gripper to closed position (-90 degrees)."""
    servo3.angle = -90  # Closed
    sleep(1)
    print("Gripper closed")

def top_shelf():
    """Move to top shelf."""
    smooth_move(50, step=0.1, delay=0.02)
    print("Moving to top shelf")
    sleep(1)

def middle_shelf():
    """Move to middle shelf."""
    smooth_move(-20, step=0.1, delay=0.02)
    print("Moving to middle shelf")
    sleep(1)

def bottom_shelf():
    """Move to bottom shelf."""
    smooth_move(-65, step=0.1, delay=0.02)
    print("Moving to bottom shelf")
    sleep(1)

def floor_level():
    """Move to floor level and handle gripper."""
    open_gripper()
    smooth_move(-85, step=0.1, delay=0.02)
    print("Moving to floor level")
    close_gripper()

while True:
    user_input = input("0 (Ground level), 1 (Low Shelf), 2 (Middle Shelf), 3 (High Shelf), q to quit, s to close gripper: ")
    
    if user_input == "s":
        close_gripper()

    elif user_input == "0":
        floor_level()

    elif user_input == "1":
        bottom_shelf()
        open_gripper()

    elif user_input == "2":
        middle_shelf()
        open_gripper()

    elif user_input == "3":
        top_shelf()
        open_gripper()

    elif user_input.lower() == "q":
        print("Exiting program.")
        servo1.close()
        servo3.close()
        break

    else:
        print("Invalid input, please try again.")