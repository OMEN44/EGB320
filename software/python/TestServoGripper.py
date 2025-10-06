# # # --------------------------------------------
# # # Coninuous servo
# # # from gpiozero import Servo
# # # from time import sleep

# # # # Continuous rotation servo on GPIO24 (change to your pin if needed)
# # # servo = Servo(
# # #     12,
# # #     min_pulse_width=0.0005,
# # #     max_pulse_width=0.0025
# # # )

# # # # Spin clockwise for 5 seconds
# # # servo.max()  # full speed clockwise
# # # sleep(0.2)

# # # servo.mid()


# # # servo.min()  # full speed clockwise
# # # sleep(0.2)


# # # # Stop the servo
# # # servo.mid()
# # # -------------------------------

# # from gpiozero import Servo
# # from time import sleep

# # # FS90R Continuous rotation servo on GPIO12
# # servo = Servo(
# #     18,
# #     min_pulse_width=0.0005,
# #     max_pulse_width=0.0025
# # )

# # # def smooth_speed(start, end, step, delay):
# # #     stepSize = (end - start) / step
# # #     stepCount = (end - start) / stepSize
# # #     stepTime = delay / stepCount

# # #     value = start
# # #     servo.value = value
# # #     while (value < end):
# # #         value += stepSize
# # #         servo.value = max(min(value, 0.2), -0.2)
# # #         sleep(stepTime)
# # #     servo.value = end

# # # Example motion sequence

# # # print("Ramping forward...")
# # # smooth_speed(0.0, 0.2, step=0.1, delay=0.1)  # accelerate forward
# # # sleep(1)

# # print("Closing")
# # servo.value = -0.15 # Closing is -
# # sleep(1) # time




# from gpiozero import Servo
# from time import sleep

# # Define servo pin (BOARD numbering, pin 12)
# servo_pin = 23

# # Initialize servo with default pulse widths (1ms min, 2ms max, 20ms period = 50 Hz)
# servo = Servo(servo_pin, min_pulse_width=0.001, max_pulse_width=0.002, frame_width=0.02)

# print('Initialized servo on pin 12 at 50 Hz')

# # Move to center (0 in gpiozero maps to 1.5ms pulse)
# servo.value = 0
# print('Moved to center position')
# sleep(2)

# # Move to left (12% duty cycle ≈ 2.4ms, map to max = 1)
# servo.value = 1
# print('Moved to left position')
# sleep(2)

# # Move to right (1% duty cycle ≈ 0.2ms, map to min = -1)
# servo.value = -1
# print('Moved to right position')
# sleep(2)

# # Cleanup is automatic with gpiozero, but explicitly close if needed
# servo.close()
# print('Cleaned up servo')

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
