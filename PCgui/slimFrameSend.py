# -*- coding: utf-8 -*-
"""
Created on Wed May  4 22:33:05 2022

@author: Stoneandbeach
"""

import serial
import time
from letterToBits import *

DISPLAY_NR_COLS = 12

def transmitText(text):
    byteList = []
    spaceByte, _ = letterToBits(' ')
    spaceByte = spaceByte[0]
    for letter in text:
        letterBytes, _ = letterToBits(letter)
        for byte in letterBytes:
            byteList.append(byte)
        byteList.append(spaceByte)
    
    byteList = byteList[:DISPLAY_NR_COLS]
    
    print(byteList)
    
    textBytes = bytearray(byteList)
    for i in range(len(textBytes)):
        ser.write([textBytes[i]])
        print("Transmitting", textBytes[i])

# Incoming from Arduino
COM_READY = 1
COM_REQUEST_FRAME = 2
COM_FRAME_RECEIVED = 3

# Outgoing to Arduino
COM_FRAME_AVAILABLE = 1


ready = False
frameReceived = True

frameNr = 0;

defaultText =  [0b00000000,
                0b00000000,
                0b00010000,
                0b00000000,
                0b00000000,
                0b00010000,
                0b00000000,
                0b00000000,
                0b00010000,
                0b00000000,
                0b00000000,
                0b00000000]

text = defaultText

textBytes = bytearray(text)

dummyFrame = 4;

print('Arduino loading, please wait...')

with serial.Serial(port="COM3", baudrate=115200, timeout=1) as ser:
         
    while not ready:
        if ser.in_waiting:
            cmd = ser.read()
            if cmd[0] == COM_READY:
                ready = True
                print('Arduino reports READY')
            else:
                print('Unknown command:', cmd[0])
    
    while True:
        
        if ser.in_waiting:
            cmd = ser.read()
            print(cmd)
        
            if cmd[0] == COM_REQUEST_FRAME:
                print('Frame request received. Sending frame...', frameNr)
                frameNr += 1
                
                transmitText(text)
                
            elif cmd[0] == COM_FRAME_RECEIVED:
                frameReceived = True
                print("Arduino reports frame received!")
            
            else:
                print("Wrongo respongo:")
                print('%i' % cmd[0])
                #else:
                #    print("Response timeout!")
                    
        if frameReceived:
            text = input('Please enter text to transmit:')
            #text = 'ROOMBA'
            #time.sleep(.1)
            print("Sending FRAME AVAILABLE message.")
            ser.write([COM_FRAME_AVAILABLE])
            frameReceived = False