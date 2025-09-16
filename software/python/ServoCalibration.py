from gpiozero import AngularServo
from time import sleep

servo3 = AngularServo(
    23,  # GPIO18
    min_angle=-90,
    max_angle=90,
    min_pulse_width=0.0005,
    max_pulse_width=0.0025
)

# Adjust this value after calibration if servo doesn't stop at value=0
STOP_VALUE = 0.1  # May need to be slightly adjusted, e.g., 0.05 or -0.03

def calibrate_servo():
    """Interactively test servo3 values to find the true stop point."""
    print("Calibrating servo3. Enter a value between -1 and 1 to test (e.g., 0, 0.1, -0.1). Type 'q' to quit.")
    while True:
        user_input = input("Enter test value for servo3 or 'q' to quit: ")
        if user_input.lower() == 'q':
            print("Calibration stopped.")
            break
        try:
            test_value = float(user_input)
            if -1 <= test_value <= 1:
                servo3.value = test_value
                print(f"Set servo3.value = {test_value}")
                sleep(1)  # Allow time to observe movement
            else:
                print("Value must be between -1 and 1.")
        except ValueError:
            print("Invalid input. Enter a number or 'q'.")


while True:
    calibrate_servo()

