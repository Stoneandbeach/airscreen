# -*- coding: utf-8 -*-
"""
Created on Thu Apr 28 20:08:28 2022

@author: Stoneandbeach
"""

import serial
import time
import letterToBits

ser = serial.Serial(port="COM3", baudrate=115200, timeout=5)
time.sleep(1)

ser.write(1)
time.sleep(1)
response = ser.read()
print("Response:", response)
print("Type:", type(response))
time.sleep(1)

# =============================================================================
# ser.write(b"1")
# time.sleep(1)
# respone = ser.read()
# print("Response:", response)
# print("Type:", type(response))
# =============================================================================
    
ser.close()