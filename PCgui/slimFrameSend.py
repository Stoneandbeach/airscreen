# -*- coding: utf-8 -*-
"""
Created on Wed May  4 22:33:05 2022

@author: Stoneandbeach
"""

import serial
import time

# Incoming from Arduino
COM_READY = 1
COM_REQUEST_FRAME = 2
COM_FRAME_RECEIVED = 3

# Outgoing to Arduino
COM_FRAME_AVAILABLE = 1


ready = False
frameReceived = False

frameNr = 0;

text = [0b00011110,
        0b00001001,
        0b00011110,
        0b00000000,
        0b00000001,
        0b00011111,
        0b00000001,
        0b00000000,
        0b00010111,
        0b00000000,
        0b00000000,
        0b00000000]

textBytes = bytearray(text)

with serial.Serial(port="COM3", baudrate=115200, timeout=1) as ser:
         
    while True:
    
        if ser.in_waiting:    
            cmd = ser.read()
            print(cmd)
        
            if not ready:
                if cmd[0] == COM_READY:
                    ready = True
                    print("Sending FRAME AVAILABLE message.")
                    ser.write([COM_FRAME_AVAILABLE])
            else:
                if cmd[0] == COM_REQUEST_FRAME:
                    print('Frame request received. Sending frame...', frameNr)
                    frameNr += 1
                    for i in range(len(text)):
                        ser.write([textBytes[i]])
                        print("Transmitting", textBytes[i])
                    
                    response = ser.read()
                    
                    if len(response) != 0: 
                        if response[0] == COM_FRAME_RECEIVED:
                            frameReceived = True
                            print("Arduino reports frame received!")
                        else:
                            print("Wrongo respongo:")
                            print('%i' % response[0])
                    else:
                        print("Response timeout!")
                    
        elif ready and frameReceived:
            print("Sending FRAME AVAILABLE message.")
            ser.write([COM_FRAME_AVAILABLE])
            frameReceived = False

