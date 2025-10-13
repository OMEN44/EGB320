import lgpio
import time

class QuadratureEncoder:
    def __init__(self, pin_a, pin_b):
        self.pin_a = pin_a
        self.pin_b = pin_b
        self.position = 0
        self.last_state = 0  # Bit 1 for A, Bit 0 for B

        # Open GPIO chip (0 for default)
        self.h = lgpio.gpiochip_open(0)

        # Claim pins as inputs with pull-ups
        lgpio.gpio_claim_input(self.h, self.pin_a)
        lgpio.gpio_claim_input(self.h, self.pin_b)
        lgpio.gpio_set_pullup(self.h, self.pin_a, lgpio.PULL_UP)
        lgpio.gpio_set_pullup(self.h, self.pin_b, lgpio.PULL_UP)

        # Set up interrupts for edge detection on both pins
        lgpio.callback(self.h, self.pin_a, lgpio.BOTH, self._update_position)
        lgpio.callback(self.h, self.pin_b, lgpio.BOTH, self._update_position)

    def _update_position(self, channel, level, tick):
        """Callback function triggered on pin state changes."""
        # Read current states
        current_a = lgpio.gpio_read(self.h, self.pin_a)
        current_b = lgpio.gpio_read(self.h, self.pin_b)
        current_state = (current_a << 1) | current_b  # A: bit 1, B: bit 0

        # Quadrature decoding state table
        if self.last_state == 0 and current_state == 1:
            self.position += 1
        elif self.last_state == 1 and current_state == 3:
            self.position += 1
        elif self.last_state == 3 and current_state == 2:
            self.position += 1
        elif self.last_state == 2 and current_state == 0:
            self.position += 1
        elif self.last_state == 0 and current_state == 2:
            self.position -= 1
        elif self.last_state == 2 and current_state == 3:
            self.position -= 1
        elif self.last_state == 3 and current_state == 1:
            self.position -= 1
        elif self.last_state == 1 and current_state == 0:
            self.position -= 1

        self.last_state = current_state
        print(f"Position: {self.position}")

    def read(self):
        """Return current position."""
        return self.position

    def cleanup(self):
        """Clean up GPIO resources."""
        lgpio.gpiochip_close(self.h)

# Example usage
if __name__ == "__main__":
    # Initialize encoder on GPIO 24 (A) and 10 (B)
    encoder = QuadratureEncoder(17, 4)
    print("Rotating encoder will update position. Press Ctrl+C to exit.")
    try:
        while True:
            time.sleep(0.1)  # Keep script running to handle interrupts
    except KeyboardInterrupt:
        encoder.cleanup()