#!/usr/bin/env python

# esc_start.py

# 2015-04-14
# Public Domain
#
# Sends the servo pulses needed to initialise some ESCs
#
# Requires the pigpio daemon to be running
#
# sudo pigpiod

import time

import pigpio

SERVO_0 = 16
SERVO_1 = 20
SERVO_2 = 21

pi = pigpio.pi() # Connect to local Pi.

pi.set_servo_pulsewidth(SERVO_0, 1000) # Minimum throttle.'''
pi.set_servo_pulsewidth(SERVO_1, 1000) # Minimum throttle.'''
pi.set_servo_pulsewidth(SERVO_2, 1000) # Minimum throttle.'''

time.sleep(1)

pi.set_servo_pulsewidth(SERVO_0, 2000) # Maximum throttle.'''
pi.set_servo_pulsewidth(SERVO_1, 2000) # Minimum throttle.'''
pi.set_servo_pulsewidth(SERVO_2, 2000) # Minimum throttle.'''


time.sleep(0.1)

pi.set_servo_pulsewidth(SERVO_0, 1535) # Slightly open throttle.

time.sleep(5)

pi.set_servo_pulsewidth(SERVO_0, 1000) # Stop servo pulses.
pi.set_servo_pulsewidth(SERVO_1, 1000) # Stop servo pulses.
pi.set_servo_pulsewidth(SERVO_2, 1000) # Stop servo pulses.

pi.stop() # Disconnect from local Raspberry Pi.