# Example code for PiicoDev Motion Sensor MPU6050
from PiicoDev_MPU6050 import PiicoDev_MPU6050
from PiicoDev_Unified import sleep_ms # Cross-platform compatible sleep function
import time

motion = PiicoDev_MPU6050()

# calcuate biases with averaging
Ax_bias = 0
Vx_bias = 0
Gz_bias = 0

count = 0
calibrationSteps = 800

t=time.time()

# Pass in imu object and number of steps to average over
# This will return the bias for the accel and gyro
# THE ROBOT MUST BE STILL DURING THIS CALIBRATION
def calibrate(imu, calibrationSteps=100):
    t = time.time()

    Ax_bias = 0
    Gz_bias = 0

    count = 0
    while count < calibrationSteps:
        accel = imu.read_accel_data() # read the accelerometer [ms^-2]
        Ax_bias += accel["x"]

        gyro = imu.read_gyro_data()   # read the gyro [deg/s]
        Gz_bias += gyro["z"]
        count += 1

    Ax_bias /= calibrationSteps
    Gz_bias /= calibrationSteps

    return Ax_bias, Gz_bias

def updatePosition(imu, Ax_bias, Gz_bias):
    vel_x = 0
    pos_x = 0
    rot_z = 0

    t = time.time()

    accel = imu.read_accel_data() # read the accelerometer [ms^-2]
    aX = accel["x"] - Ax_bias
    
    gyro = imu.read_gyro_data()   # read the gyro [deg/s]
    gZ = gyro["z"] - Gz_bias

    now = time.time()
    dt = now - t
    t = now

    # Dan was here :)
    
    rot_z += gZ * dt

    return aX, vel_x, pos_x, gZ, rot_z

while count < calibrationSteps:
    accel = motion.read_accel_data() # read the accelerometer [ms^-2]
    Ax_bias += accel["x"]

    now = time.time()
    dt = now - t
    t = now

    if (count > 0):
        Vx_bias += (accel["x"] - Ax_bias / count) * dt

    gyro = motion.read_gyro_data()   # read the gyro [deg/s]
    Gz_bias += gyro["z"]
    count += 1

Ax_bias /= calibrationSteps
Vx_bias /= (calibrationSteps - 1)
Gz_bias /= calibrationSteps

vel_x = 0
pos_x = 0
rot_z = 0

t = time.time()

while True:
    
    # Accelerometer data
    accel = motion.read_accel_data() # read the accelerometer [ms^-2]
    aX = accel["x"] - Ax_bias
    
    # Gyroscope Data
    gyro = motion.read_gyro_data()   # read the gyro [deg/s]
    gZ = gyro["z"] - Gz_bias

    now = time.time()
    dt = now - t
    t = now
    vel_x += aX * dt - Vx_bias
    pos_x += vel_x * dt
    rot_z += gZ * dt

    # Rough temperature
    temp = motion.read_temperature()   # read the device temperature [degC]

    # G-Force
    gforce = motion.read_accel_abs(g=True) # read the absolute acceleration magnitude
    print("Accel:\nx:" + str(aX) + "\nvel: " + str(vel_x) + "\npos: " + str(pos_x) + "\nGyro:\nz:" + str(gZ) + "\nrot: " + str(rot_z) + "\nTemperature: " + str(temp) + "°C" + "\nG-Force: " + str(gforce))
    print('\n')
    
    # sleep_ms(100)