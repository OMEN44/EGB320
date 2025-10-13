from gpiozero import DigitalInputDevice
from time import sleep

# Define GPIO pins
PHASE_A_PIN = 17
PHASE_B_PIN = 4

# Initialize GPIO pins with pull-up resistors and debouncing
phase_a = DigitalInputDevice(PHASE_A_PIN, pull_up=True, bounce_time=0.001)
phase_b = DigitalInputDevice(PHASE_B_PIN, pull_up=True, bounce_time=0.001)

# Initialize counter and last states
counter = 0
last_a = phase_a.value
last_b = phase_b.value

# Interrupt callback for encoder state changes
def update_encoder(_):
    global counter, last_a, last_b
    a_state = phase_a.value
    b_state = phase_b.value
    # Quadrature state transitions
    if (last_a, last_b, a_state, b_state) in [(0,0,0,1), (0,1,1,1), (1,1,1,0), (1,0,0,0)]:
        counter += 1  # Clockwise
    elif (last_a, last_b, a_state, b_state) in [(0,0,1,0), (1,0,1,1), (1,1,0,1), (0,1,0,0)]:
        counter -= 1  # Counterclockwise
    last_a, last_b = a_state, b_state
    # Print counter and pin states for debugging
    print(f"Counter: {counter}, Phase A: {a_state}, Phase B: {b_state}")

# Attach interrupt handlers to both edges of both pins
phase_a.when_activated = update_encoder
phase_a.when_deactivated = update_encoder
phase_b.when_activated = update_encoder
phase_b.when_deactivated = update_encoder

print("Monitoring quadrature encoder with interrupts. Press Ctrl+C to stop.")

try:
    while True:
        sleep(1)
except KeyboardInterrupt:
    print("\nProgram stopped.")
finally:
    phase_a.close()
    phase_b.close()