# fusiontest.py Simple test program for sensor fusion on Pyboard
# Author Peter Hinch
# Released under the MIT License (MIT)
# Copyright (c) 2017 Peter Hinch
# V0.8 14th May 2017 Option for external switch for cal test. Make platform independent.
# V0.7 25th June 2015 Adapted for new MPU9x50 interface

import PID
import time
#import matplotlib.pyplot as plt
import numpy as np
#from scipy.interpolate import spline
import FaBo9Axis_MPU9250
import time
from fusion import Fusion
import pigpio

SERVO = 16
SERVO_1 = 20

pi = pigpio.pi()


mpu9250 = FaBo9Axis_MPU9250.MPU9250()
fuse = Fusion()

# Code for external switch

# Code for Pyboard switch
#sw = pyb.Switch()

# Choose test to run
Calibrate = False
Timing = False

#pid = PID.PID(1, 0.2, 0.02)





def clamp(n, minn, maxn):
    return max(min(maxn, n), minn)

def map(x):
    return (x - (90)) * (2000 - 1000) / ((-90) - (90)) + 1000



if Calibrate:
    print("Calibrating. Press switch when done.")
    fuse.calibrate(getmag, sw, lambda : pyb.delay(100))
    print(fuse.magbias)
'''
if Timing:            
    mag = imu.mag.xyz # Don't include blocking read in time
    accel = imu.accel.xyz # or i2c
     gyro = imu.gyro.xyz
    start = time.ticks_us()  # Measure computation time only
    fuse.update(accel, gyro, mag) # 1.97mS on Pyboard
    t = time.ticks_diff(time.ticks_us(), start)
    print("Update time (uS):", t)
'''

pid_angle_roll = PID.PID(1.0, 0, 0)
pid_gyro_roll = PID.PID(0, 0.8, 0)
pid_angle_pitch = PID.PID(1.0, 0, 0)
pid_gyro_pitch = PID.PID(0, 0.8, 0)
pid_angle_roll.setSampleTime(0.0002)
pid_gyro_roll.setSampleTime(0.0002)
pid_angle_roll.SetPoint=0
pid_angle_pitch.setSampleTime(0.0002)
pid_gyro_pitch.setSampleTime(0.0002)
pid_angle_pitch.SetPoint=0

while True:
    accel = mpu9250.readAccel()
    gyro = mpu9250.readGyro()
    mag = mpu9250.readMagnet()
    accel_ali = (accel['x'], accel['y'], accel['z'])
    gyro_ali = (gyro['x'], gyro['y'], gyro['z'])
    mag_ali = (mag['x'], mag['y'], mag['z'])    
    fuse.update(accel_ali, gyro_ali, mag_ali) # Note blocking mag read
    feedback_angle_roll = fuse.roll
    pid_angle_roll.update(feedback_angle_roll)
    pid_gyro_roll.SetPoint = pid_angle_roll.output
    feedback_gyro_roll = gyro['x']
    pid_gyro_roll.update(feedback_gyro_roll)
    output_roll = clamp(int(map(pid_gyro_roll.output)),1000,2000)
    pi.set_servo_pulsewidth(SERVO, int(output_roll))
    feedback_angle_pitch = fuse.pitch
    pid_angle_pitch.update(feedback_angle_pitch)
    pid_gyro_pitch.SetPoint = pid_angle_pitch.output
    feedback_gyro_pitch = gyro['y']
    pid_gyro_pitch.update(feedback_gyro_pitch)
    output_pitch = clamp(int(map(pid_gyro_pitch.output)),1000,2000)
    pi.set_servo_pulsewidth(SERVO_1, int(output_pitch))
    
    #pi.set_servo_pulsewidth(SERVO_1, 1700)
    print("Pitch, pwm_output: {:7.3f} {:7.3f} {:7.3f} {:7.3f}".format(fuse.pitch, output_roll, pid_gyro_roll.SetPoint, pid_gyro_roll.output))
