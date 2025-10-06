__author__ = 'Majid Alekasir' # majid.alekasir@gmail.com

### Some changes are applied to the original library by 'Geir Istad' ###
### The changes aim to enhance the results of *roll-pitch-yaw* and *linear acceleration* ###
### Some changes are made for the ease of use ###
### You need to install 'Scipy library' which is used in 'DMP_get_roll_pitch_yaw' function. The former function was not accurate and was giving improper results ###

"""
MPU6050 Python I2C Class - MPU6050 example usage
Copyright (c) 2015 Geir Istad

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
"""

from mpu6050 import MPU6050
import time
i2c_bus = 1
device_address = 0x68
freq_divider = 0x01
### DMP output frequency is calculated easily using this equation: (200Hz / (1 + value))
### For example, 0x04 gives (200Hz / (1 + 4)) = 40HZ
### I propose 0x04 for a less noisy quaternion and 0x01 for accelerometer & gyrometer.

# Make an MPU6050
mpu = MPU6050.MPU6050(i2c_bus, device_address, freq_divider)

# Initiate your DMP
mpu.dmp_initialize()
mpu.set_DMP_enabled(True)

packet_size = mpu.DMP_get_FIFO_packet_size()
FIFO_buffer = [0]*64

g = 9.8 # gravity acceleration (m/s^2)
Ax_bias = 0 # You can add a bias to the acceleration in X direction
Gz_bias = 0 # You can add a bias to the gyrometer in Z direction

count = 0
calibrationSteps = 1500

while count < calibrationSteps:
    # Calcuate the biases
    if mpu.isreadyFIFO(packet_size): # Check if FIFO data are ready to use
        accel = mpu.get_acceleration()
        # Ax_bias += accel.x * 2*g / 2**15
        Ax_bias += accel.x# * 2*g / 2**15
        gyro = mpu.get_rotation()
        # Gz_bias += gyro.z * 250 / 2**15
        Gz_bias += gyro.z# * 250 / 2**15
        count += 1

Ax_bias /= calibrationSteps
Gz_bias /= calibrationSteps

velocity_x = 0
position_x = 0
rotation_z = 0

#  print values with bias correction
t = time.time()
while True: # infinite loop
    if mpu.isreadyFIFO(packet_size): 
        accel = mpu.get_acceleration()
        # Ax = accel.x * 2*g / 2**15
        Ax = (accel.x - Ax_bias) / 2**15 # * 2*g / 2**15
        gyro = mpu.get_rotation()
        # Gz = gyro.z * 250 / 2**15
        Gz = (gyro.z - Gz_bias) / 2**15 # * 250 / 2**15

        now = time.time()
        dt = now - t
        t = now

        velocity_x += Ax * dt
        position_x += velocity_x * dt
        rotation_z += Gz * dt

        print('Ax: ' + str(Ax))
        print('Vx: ' + str(velocity_x))
        print('Px: ' + str(position_x))
        print('Gz: ' + str(Gz))
        print('Rz: ' + str(rotation_z))
        print('\n')    

# while True: # infinite loop
#     if mpu.isreadyFIFO(packet_size): # Check if FIFO data are ready to use...
        
#         FIFO_buffer = mpu.get_FIFO_bytes(packet_size) # get all the DMP data here

#         ### The full range of accelerometer is set to [-2g, +2g]. ###
#         ### The transformations in this code are based on this range ###

#         # raw acceleration
#         accel = mpu.get_acceleration()
#         Ax = accel.x * 2*g / 2**15

#         # DMP acceleration (less noisy acceleration - based on fusion)
#         accel_dmp = mpu.DMP_get_acceleration_int16(FIFO_buffer)
#         Ax_dmp = accel_dmp.x * 2*g / 2**15 * 2

#         # raw gyro (full range: [-250, +250]) (unit: degree / second)
#         gyro = mpu.get_rotation()
#         Gz = gyro.z * 250 / 2**15
        
#         print('Ax: ' + str(Ax))
#         print('Ax_dmp: ' + str(Ax_dmp))
#         print('Gz: ' + str(Gz))
#         print('\n')

#         ### About transformations ###
#         # We have a 16 bit signed number for accelerometer. Thus the range of this number is [-2^15, +2^15]
#         # The range of the accelerometer is [-2g, +2g].
#         # The ranges are adjustable for accelerometer and gyrometer (in MPU6050 library).

#         # You can use RAW or DMP acceleration.