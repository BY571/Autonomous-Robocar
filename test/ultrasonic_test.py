#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Fri Jan 12 19:08:02 2018

@author: sebastian dittert
"""

import RPi.GPIO as gpio
import time

def distance(measure = "cm"):
    gpio.setmode(gpio.BCM)
    ECHO = 18
    TRIG = 4
    gpio.setup(ECHO,gpio.IN) 
    gpio.setup(TRIG,gpio.OUT)  
    
    gpio.output(TRIG, True)
    time.sleep(0.0001)
    gpio.output(TRIG, False)
    # Measure the time
    while gpio.input(ECHO) == False:
        no_signal =time.time()
    while gpio.input(ECHO) == True:
        signal =time.time()
    # calculate the time between no_signal an signal
    t1 = signal - no_signal
    if measure == "cm":
        distance = t1/0.000058
    elif measure == "mm":
        distance = t1/0.0000058
    else:
        print("Proper choice of measure (mm or cm)")
        distance = None
    gpio.cleanup()
    return distance
try:
    while True:
        print(distance("cm"))

except KeyboardInterrupt:
    pass 
