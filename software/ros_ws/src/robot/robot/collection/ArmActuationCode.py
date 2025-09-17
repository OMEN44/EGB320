# from gpiozero import AngularServo
# from time import sleep
# from evdev import InputDevice, list_devices, ecodes

# # Servo Setup - Need to change values according to servos
# base_servo = AngularServo(17, min_angle=-90, max_angle=90,
#                           min_pulse_width=0.0005, max_pulse_width=0.0025)
# shoulder_servo = AngularServo(27, min_angle=-90, max_angle=90,
#                               min_pulse_width=0.0005, max_pulse_width=0.0025)
# elbow_servo = AngularServo(22, min_angle=-90, max_angle=90,
#                            min_pulse_width=0.0005, max_pulse_width=0.0025)


# def move_arm(base, shoulder, elbow, delay=0.5):
#     """Move arm to given joint angles"""
#     base_servo.angle = base
#     shoulder_servo.angle = shoulder
#     elbow_servo.angle = elbow
#     sleep(delay)

# # Poses/Stages - Need to change values according to servos


# def rest_pose():
#     print("REST pose")
#     move_arm(0, -60, 60)


# def straight_pose():
#     print("STRAIGHT FORWARD pose")
#     move_arm(0, 0, 0)


# def vertical_pose():
#     print("VERTICAL pose")
#     move_arm(0, 60, -60)


# def low_pose():
#     print("LOW LEVEL pose (below base)")
#     move_arm(0, -75, 30)


# # Keyboard Device Setup
# devices = [InputDevice(path) for path in list_devices()]

# # Filter for a real keyboard
# must_have = {i for i in range(1, 32)}
# must_not_have = {0}
# devices = [
#     dev
#     for dev in devices
#     for keys in (set(dev.capabilities().get(ecodes.EV_KEY, [])),)
#     if must_have.issubset(keys) and must_not_have.isdisjoint(keys)
# ]

# if not devices:
#     raise RuntimeError("No keyboard device found!")

# keyboard = devices[0]
# print(f"Using keyboard: {keyboard.fn}")

# # Key Bindings
# keypress_actions = {
#     ecodes.KEY_1: rest_pose,
#     ecodes.KEY_2: straight_pose,
#     ecodes.KEY_3: vertical_pose,
#     ecodes.KEY_4: low_pose,
# }

# print("Press 1=REST, 2=STRAIGHT, 3=VERTICAL, 4=LOW (Ctrl+C to quit)")

# # Event Loop
# for event in keyboard.read_loop():
#     if event.type == ecodes.EV_KEY and event.code in keypress_actions:
#         if event.value == 1:  # key pressed
#             keypress_actions[event.code]()  # call the matching pose



## Moving all the parts correctly but too fast
from gpiozero import AngularServo
from time import sleep

# Software PWM version (no pigpio)
# GPIO24 -> Physical pin 18
# GPIO12 -> Physical pin 32

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


while True:
    servo1.angle = -60
    servo2.angle = 80
    sleep(2)
    
    servo1.angle = -30
    servo2.angle = 30
    sleep(2)

    servo1.angle = 0
    servo2.angle = -30
    sleep(2)

    servo2.angle = -90
    servo1.angle = 60
    
    sleep(2)