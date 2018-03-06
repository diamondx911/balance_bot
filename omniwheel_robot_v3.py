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
    return (x - (-500)) * (2000 - (1000)) / (500 - (-500)) + (1000)

def map_magnitude(x):
    return (x - (1414.21)) * (2000 - (1000)) / (2828.42 - (1414.21)) + (1000)

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

pid_pitch_magnitude = PID.PID(6, 0, 0.4)
pid_roll_magnitude = PID.PID(6, 0, 0.4)
pid_pitch_magnitude.setSampleTime(0.0002)
pid_pitch_magnitude.SetPoint=0
pid_roll_magnitude.setSampleTime(0.0002)
pid_roll_magnitude.SetPoint=0

def signal_handler(signal, frame):
        print('You pressed Ctrl+C!')
        pi.set_servo_pulsewidth(SERVO_1, 1500)
        pi.set_servo_pulsewidth(SERVO_2, 1500)
        pi.set_servo_pulsewidth(SERVO_3, 1500)
        sys.exit(0)
signal.signal(signal.SIGINT, signal_handler)
'''
def calcAngle(xVal, yVal):
    if xVal >= 0.0 and yVal >= 0.0:
        return math.atan(yVal/xVal)
    elif((xVal < 0.0 and yVal >= 0.0) or (xVal < 0.0 and yVal < 0.0)):
        return math.atan(yVal/xVal) + 3.14
    else
        return math.atan(yVal/xVal) + 2*3.14
'''
while True:
    start = time.time()
    accel = mpu9250.readAccel()
    gyro = mpu9250.readGyro()
    mag = mpu9250.readMagnet()
    accel_ali = (accel['x'], accel['y'], accel['z'])
    gyro_ali = (gyro['x'], gyro['y'], gyro['z'])
    mag_ali = (mag['x'], mag['y'], mag['z'])    
    fuse.update(accel_ali, gyro_ali, mag_ali) # Note blocking mag read
    feedback_angle_pitch = fuse.pitch
    pid_pitch_magnitude.update(feedback_angle_pitch)
    output_pitch_Vy = map_angle_pitch(pid_pitch_magnitude.output)
    
    feedback_angle_roll = fuse.roll
    pid_roll_magnitude.update(feedback_angle_roll)
    output_roll_Vx = map_angle_roll(pid_roll_magnitude.output)
    
    
    
    
    
    magnitude = map_magnitude((math.sqrt((output_roll_Vx*output_roll_Vx)+(output_pitch_Vy*output_pitch_Vy))))
    theta = math.atan2(output_pitch_Vy,output_roll_Vx)
    
    Vx = -(math.cos(theta)*magnitude)
    Vy = -(math.sin(theta)*magnitude)
    
    
    speed_wheel_1 =  map((-Vx))
    speed_wheel_2 =  map((0.5*Vx)-((math.sqrt(3)/2)*Vy))
    speed_wheel_3 =  map((0.5*Vx)+((math.sqrt(3)/2)*Vy))
    pi.set_servo_pulsewidth(SERVO_1, clamp(int(speed_wheel_1),1000,2000))
    pi.set_servo_pulsewidth(SERVO_2, clamp(int(speed_wheel_2),1000,2000))
    pi.set_servo_pulsewidth(SERVO_3, clamp(int(speed_wheel_3),1000,2000))
    end = time.time()
    #pi.set_servo_pulsewidth(SERVO_1, 1700)
    print("Pitch, pwm_output: {:7.3f} {:7.3f} {:7.3f}".format(fuse.roll, speed_wheel_2, speed_wheel_3))
