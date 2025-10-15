from PiicoDev_VL53L1X import PiicoDev_VL53L1X
from time import sleep

sensor = PiicoDev_VL53L1X()

while True:
    dist = sensor.read() / 100
    print("Distance: {} mm".format(dist))
    sleep(0.1)