# -*- coding: utf-8 -*-
"""
Created on Wed May  4 20:18:53 2022

@author: Stoneandbeach
"""

import serial
import time
import numpy as np

ser = serial.Serial(port="COM3", baudrate=115200, timeout=2)

time.sleep(.5)

value = bytes([3])
#value = b'a'

print('Value:', value)
print('Type:', type(value))

ser.write(value)

time.sleep(.5)

response = ser.read()
print(type(response))

i = 1

print('%i' % response[0])
print('Resp', i, ':', response)

while response != b'':

    i += 1
    response = ser.read()
    print('Resp', i, ':', response)

ser.close()