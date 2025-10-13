
import time
import lgpio

PHASE_A = 17  # GPIO 17 (pin 11)
PHASE_B = 4   # GPIO 4 (pin 7)
counter = 0

def encoder_callback(channel):
    global counter
    try:
        state_a = GPIO.input(PHASE_A)
        state_b = GPIO.input(PHASE_B)
        if channel == PHASE_A:
            if state_a == state_b:
                counter += 1
            else:
                counter -= 1
        else:
            if state_a != state_b:
                counter += 1
            else:
                counter -= 1
        print(f"Position: {counter}")
    except Exception as e:
        print(f"Error in callback: {e}")

try:
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(PHASE_A, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    GPIO.setup(PHASE_B, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    GPIO.add_event_detect(PHASE_A, GPIO.BOTH, callback=encoder_callback, bouncetime=1)
    GPIO.add_event_detect(PHASE_B, GPIO.BOTH, callback=encoder_callback, bouncetime=1)
    while True:
        time.sleep(1)
except Exception as e:
    exit(1)
except KeyboardInterrupt:
    print(f"Final position: {counter}")
finally:
    GPIO.cleanup()