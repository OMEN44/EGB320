import mpu6050
import time
import math

# Create a new Mpu6050 object
mpu = mpu6050.mpu6050(0x68)

# Start angle (in degrees)
z_angle = 0.0
prev_time = time.time()

while True:
    # Read gyro Z data (degrees per second)
    gyro_data = mpu.get_gyro_data()
    gz = gyro_data['z']

    # Calculate time difference
    current_time = time.time()
    dt = current_time - prev_time
    prev_time = current_time

    # Integrate gyroscope Z to estimate angle
    z_angle += gz * dt

    print("Z angle (yaw approx):", z_angle)

    time.sleep(0.01)  # 10 ms loop