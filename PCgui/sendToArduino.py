# -*- coding: utf-8 -*-
"""
Created on Fri Apr 22 23:21:45 2022

@author: Stoneandbeach
"""

import serial
import time
import letterToBits

ser = serial.Serial(port="COM3", baudrate=115200, timeout=5)
time.sleep(1)

def sendSpinup():
    
    print("Sending spinup command")
    ser.write(0x01)
    ser.write(0x01)
    time.sleep(1)
    response = ser.readline()
    print("Arduino response: '{}'".format(response))
    return

def sendFrame():
    
    frameText = input("Input frame text: ")
    frameBytes, frameHeight = letterToBits(frameText)
    frameLen = len(frameBytes)
    print("Sending frame")
    ser.write(0x02)
    ser.write(0x01)
    ser.write(frameLen)
    for byte in frameBytes:
        ser.write(byte)
    time.sleep(1)
    response = ser.readline()
    print("Arduino response: '{}'".format(response))

def sendRestart():
    
    print("Sending restart command")
    ser.write(0x03)
    ser.write(0x01)
    time.sleep(1)
    response = ser.readline()
    print("Arduino response: '{}'".format(response))
    return

def sendTest():
    print("Sending test command")
    ser.write(0x04)
    ser.write(0x01)
    time.sleep(1)
    response = ser.readline()
    print("Arduino response: '{}'".format(response))
    
def sendWord():
    word = input("Input word to send: ").encode()
    print("Sending word:", word)
    ser.write(word)
    time.sleep(1)
    response = ser.readline()
    print("Arduino response: '{}'".format(response))
    
def sendByte():
    byte = int(input("Input byte to send: "))
    print("Sending byte:", byte)
    ser.write(byte)
    time.sleep(3)
    response = ser.read_all()
    print("Arduino response: '{}'".format(response))


commandDict = {"spinup" : sendSpinup,
               "frame" : sendFrame,
               "restart" : sendRestart,
               "test" : sendTest,
               "word" : sendWord,
               "byte" : sendByte}

userInput = ""

while userInput != "exit":
    
    userInput = input("Enter command [spinup, frame, restart, test, word, byte, exit]: ")
    
    if userInput in commandDict.keys():
        cmd = commandDict[userInput]        
        cmd()
    elif userInput != "exit":
        print("Unknown command: ", userInput)
    
ser.close()