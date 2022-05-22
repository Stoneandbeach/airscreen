# -*- coding: utf-8 -*-
"""
Created on Wed May  4 22:33:05 2022

@author: Stoneandbeach
"""

import serial
import time
from letterToBits import *

DISPLAY_NR_COLS = 50

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
    
    while len(byteList) < DISPLAY_NR_COLS:
        byteList.append(spaceByte)
    
    print(byteList)
    
    textBytes = bytearray(byteList)
    for i in range(len(textBytes)):
        ser.write([textBytes[i]])
        print("Transmitting", textBytes[i])
        
def checkText(text):
    textOk = True
    okLetterUnicodes = [ord(' ')]
    for i in range(ord('A'), ord('Z') + 1):
        okLetterUnicodes.append(i)
    for letter in text:
        if ord(letter) not in okLetterUnicodes:
            textOk = False
            break
    return textOk

# Incoming from Arduino
COM_READY = 0b01000000
COM_REQUEST_FRAME = 0b01000001
COM_FRAME_RECEIVED = 0b01000010

# Outgoing to Arduino
COM_FRAME_AVAILABLE = 0b01000000


ready = False
frameReceived = True
comsProcessed = False

frameNr = 0;

print('Arduino loading, please wait...')

with serial.Serial(port="COM3", baudrate=115200, timeout=1) as ser:
         
    while not ready:
        if ser.in_waiting:
            cmd = ser.read()
            print(cmd)
            if cmd[0] == COM_READY:
                ready = True
                print('Arduino reports READY')
            else:
                print('Unknown command:', cmd[0])
    
    while True:
        
        while ser.in_waiting:
            cmd = ser.read()
            print(cmd)
        
            if cmd[0] == COM_REQUEST_FRAME:
                print('Frame request received. Sending frame...', frameNr)
                frameNr += 1
                try:
                    transmitText(text)
                except KeyError:
                    print("Cannot send input! Only use capital letters.")
                    frameReceived = True # Note this is not good practice!
                comsProcessed = True
                
            elif cmd[0] == COM_FRAME_RECEIVED:
                frameReceived = True
                print("Arduino reports frame received!")
                comsProcessed = True
            
            else:
                print("Wrongo respongo:")
                print('%i' % cmd[0])
                print()
                comsProcessed = True
        
        if comsProcessed:
            print('Coms', cmd, 'processed.')
            print()
            comsProcessed = False
                    
        if frameReceived:
            textOk = False
            while not textOk:
                text = input('Please enter text to transmit:')
                textOk = checkText(text)
                if not textOk:
                    print('Text not ok:', text)
                
            #text = 'ROOMBA'
            #time.sleep(.1)
            print("Sending FRAME AVAILABLE message.")
            ser.write([COM_FRAME_AVAILABLE])
            frameReceived = False