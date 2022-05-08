# -*- coding: utf-8 -*-
"""
Created on Fri Apr 22 23:21:45 2022

@author: Stoneandbeach
"""

import serial
import time

ser = serial.Serial(port="COM3", baudrate=115200, timeout=10)
time.sleep(1)
i = 0

while i < 10:
    print("Writing")
    ser.write(b"A")
    print("Sleeping")
    time.sleep(1)
    print("Reading")
    var = ser.read_all()
    print("Read", var)
    i = i + 1
    print("Loop number", i)
    
ser.close()