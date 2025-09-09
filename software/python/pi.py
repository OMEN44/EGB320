import gpiozero
import time

print(gpiozero.Device)

led = gpiozero.LED(14)

while True:
    led.on()
    time.sleep(1)
    led.off()
    time.sleep(1)