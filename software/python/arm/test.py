from gpiozero import AngularServo

# Initialize servos
servo1 = AngularServo(
    12,  # GPIO12 for base rotation
    min_angle=-90,
    max_angle=90,
    min_pulse_width=0.0005,  # 500µs
    max_pulse_width=0.0024   # 2400µs
)

while True:
    servo1.angle = -45
    