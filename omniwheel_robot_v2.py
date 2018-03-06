# fusiontest.py Simple test program for sensor fusion on Pyboard
# Author Peter Hinch
# Released under the MIT License (MIT)
# Copyright (c) 2017 Peter Hinch
# V0.8 14th May 2017 Option for external switch for cal test. Make platform independent.
# V0.7 25th June 2015 Adapted for new MPU9x50 interface

import PID
import time
import signal
import sys
#import matplotlib.pyplot as plt
import numpy as np
#from scipy.interpolate import spline
import FaBo9Axis_MPU9250
import time
from fusion import Fusion
import pigpio
import math

SERVO_1 = 16
SERVO_2 = 20
SERVO_3 = 21

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

def map_angle_pitch(x):
    return (x - (-90)) * (500 - (-500)) / ((-90) - (90)) + (500)

def map_angle_roll(x):
    return (x - (90)) * (500 - (-500)) / ((90) - (-90)) + (500)

def map(x):
    return (x - (-500)) * (2000 - (1000)) / (500 - (-600)) + (1000)

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

pid_angle_magnitude = PID.PID(1, 0, 0)
pid_gyro_magnitude = PID.PID(1, 0, 0.0001)
pid_angle_magnitude.setSampleTime(0.0002)
pid_angle_magnitude.SetPoint=0
pid_gyro_magnitude.setSampleTime(0.0002)
pid_gyro_magnitude.SetPoint=0

def signal_handler(signal, frame):
        print('You pressed Ctrl+C!')
        pi.set_servo_pulsewidth(SERVO_1, 1500)
        pi.set_servo_pulsewidth(SERVO_2, 1500)
        pi.set_servo_pulsewidth(SERVO_3, 1500)
        sys.exit(0)
signal.signal(signal.SIGINT, signal_handler)

while True:
    start = time.time()
    accel = mpu9250.readAccel()
    gyro = mpu9250.readGyro()
    mag = mpu9250.readMagnet()
    accel_ali = (accel['x'], accel['y'], accel['z'])
    gyro_ali = (gyro['x'], gyro['y'], gyro['z'])
    mag_ali = (mag['x'], mag['y'], mag['z'])    
    fuse.update(accel_ali, gyro_ali, mag_ali) # Note blocking mag read
    magnitude = math.sqrt((map_angle_pitch(fuse.pitch)*map_angle_pitch(fuse.pitch))+(map_angle_roll(fuse.roll)*(map_angle_roll(fuse.roll))))
    theta = math.atan2(map_angle_pitch(fuse.pitch),map_angle_roll(fuse.roll))
    feedback_angle_magnitude = magnitude
    pid_angle_magnitude.update(feedback_angle_magnitude)
    pid_gyro_magnitude.SetPoint = pid_angle_magnitude.output
    feedback_gyro_magnitude = math.sqrt(gyro['x']*gyro['x'] + gyro['y']*gyro['y'])
    pid_gyro_magnitude.update(feedback_gyro_magnitude)
    output_magnitude = (pid_gyro_magnitude.output)
    Vx = -(math.cos(theta)*output_magnitude)
    Vy = -(math.sin(theta)*output_magnitude)
    speed_wheel_1 =  map((-Vx))
    speed_wheel_2 =  map((0.5*Vx)-((math.sqrt(3)/2)*Vy))
    speed_wheel_3 =  map((0.5*Vx)+((math.sqrt(3)/2)*Vy))
    pi.set_servo_pulsewidth(SERVO_1, clamp(int(speed_wheel_1),1000,2000))
    pi.set_servo_pulsewidth(SERVO_2, clamp(int(speed_wheel_2),1000,2000))
    pi.set_servo_pulsewidth(SERVO_3, clamp(int(speed_wheel_3),1000,2000))
    end = time.time()
    #pi.set_servo_pulsewidth(SERVO_1, 1700)
    print("Pitch, pwm_output: {:7.3f} {:7.3f} {:7.3f}".format(fuse.pitch, speed_wheel_2, speed_wheel_3))
