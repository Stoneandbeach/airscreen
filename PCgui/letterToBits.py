# -*- coding: utf-8 -*-
"""
Created on Tue Apr 19 19:16:59 2022

@author: Stoneandbeach
"""

import numpy as np
import matplotlib.pyplot as plt

def letterToBits(letter):
    # Format of dict is "LETTER" : tuple(bits, height of letter in pixels)
    bitsDict = {"A" : ([0b00011110,
                        0b00001001,
                        0b00011110], 5),
                "B" : ([0b00011111,
                        0b00010101,
                        0b00001010], 5),
                "C" : ([0b00001110,
                        0b00010001,
                        0b00001010], 5),
                "D" : ([0b00011111,
                        0b00010001,
                        0b00001110], 5),
                "E" : ([0b00011111,
                        0b00010101,
                        0b00010101], 5),
                "F" : ([0b00011111,
                        0b00001001,
                        0b00001001], 5),
                "G" : ([0b00011111,
                        0b00010001,
                        0b00011101], 5),
                "H" : ([0b00011111,
                        0b00000100,
                        0b00011111], 5),
                "I" : ([0b00011111], 5),
                "J" : ([0b00001101,
                        0b00010001,
                        0b00001111], 5),
                "K" : ([0b00011111,
                        0b00000100,
                        0b00011011], 5),
                "L" : ([0b00011111,
                        0b00010000,
                        0b00010000], 5),
                "M" : ([0b00011111,
                        0b00000010,
                        0b00000100,
                        0b00000010,
                        0b00011111], 5),
                "N" : ([0b00011111,
                        0b00000001,
                        0b00011110], 5),
                "O" : ([0b00001110,
                        0b00010001,
                        0b00001110], 5),
                "P" : ([0b00011111,
                        0b00000101,
                        0b00000111], 5),
                "Q" : ([0b00000110,
                        0b00001001,
                        0b00010110], 5),
                "R" : ([0b00011111,
                        0b00000101,
                        0b00011010], 5),
                "S" : ([0b00010111,
                        0b00010101,
                        0b00011101], 5),
                "T" : ([0b00000001,
                        0b00011111,
                        0b00000001], 5),
                "U" : ([0b00011111,
                        0b00010000,
                        0b00011111], 5),
                "V" : ([0b00001111,
                        0b00010000,
                        0b00001111], 5),
                "W" : ([0b00011111,
                        0b00010000,
                        0b00001000,
                        0b00010000,
                        0b00011111], 5),
                "X" : ([0b00011011,
                        0b00000100,
                        0b00011011], 5),
                "Y" : ([0b00000111,
                        0b00011000,
                        0b00000111], 5),
                "Z" : ([0b00011001,
                        0b00010101,
                        0b00010011], 5),
                "Å" : ([0b00011100,
                        0b00001011,
                        0b00011100], 5),
                "Ä" : ([0b00011101,
                        0b00001010,
                        0b00011101], 5),
                "Ö" : ([0b00001101,
                        0b00010010,
                        0b00001101], 5),
                " " : ([0b00000000], 5)}
    return bitsDict[letter]

def bitsToMatrix(bits):
    bits, height = bits
    length = len(bits)
    X = np.zeros([height, length])
    for column, value in enumerate(bits):
        bitmask = 0b00000001
        for row in range(height):
            bit = (value & (bitmask << row)) >> row
            X[row, column] = bit
    return X

def imageText(text): 
    X = None
    spaceMatrix = bitsToMatrix(letterToBits(" "))
    for letter in text:
        if X is not None:
            letterMatrix = bitsToMatrix(letterToBits(letter))
            X = np.hstack([X, letterMatrix])
        else:
            X = bitsToMatrix(letterToBits(letter))
        X = np.hstack([X, spaceMatrix])
    print(X)
    fig = plt.figure()
    plt.imshow(X)