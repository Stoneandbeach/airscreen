# -*- coding: utf-8 -*-
"""
Created on Wed May  4 22:33:05 2022

@author: Stoneandbeach
"""

import serial
import time

COM_REQUEST_FRAME = 1
COM_FRAME_RECEIVED = 2
COM_FRAME_AVAILABLE = 1
COM_READY = 3

ready = False
frameReceived = False

text = [0b00011110,
        0b00001001,
        0b00011110,
        0b00000000,
        0b00000001,
        0b00011111,
        0b00000001,
        0b00000000,
        0b00010111,
        0b00000000]

textBytes = bytearray(text)

with serial.Serial(port="COM3", baudrate=115200, timeout=2) as ser:
         
    while True:
    
        cmd = ser.read()
        print(cmd)
# =============================================================================
#         if len(cmd) != 0:
#             if not ready:
#                 if cmd[0] == COM_READY:
#                     ready = True
#             else:
#                 if cmd[0] == COM_REQUEST_FRAME:
#                     print('Frame request received. Sending frame...')
#                     for i in range(len(text)):
#                         ser.write([textBytes[i]])
#                         print("Transmitting", textBytes[i])
#                     
#                     response = ser.read()
#                     if response[0] == COM_FRAME_RECEIVED:
#                         frameReceived = True
#                         print("Arduino reports frame received!")
#                     else:
#                         print("Wrongo respongo:")
#                         print('%i' % response[0])
#         elif ready and frameReceived:
#             ser.write([COM_FRAME_AVAILABLE])
#             frameReceived = False
# =============================================================================

