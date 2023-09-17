import numpy as np
import matplotlib.pyplot as plt

class Spot():
    def __init__(self, angle=0., offset=0.):
        self.angle = angle
        self.offset = offset
        self.x = offset * np.cos(angle*2*np.pi/360)
        self.y = offset * np.sin(angle*2*np.pi/360)
    
    def __str__(self):
        return str(self.angle) + ", " + str(self.offset) + ", " + str(self.x) + ", " + str(self.y)
    
    def __add__(self, other):
        x = self.x + other.x
        y = self.y + other.y
        offset = np.sqrt(x**2 + y**2)
        angle = np.arccos(x / offset) * 360 / (2 * np.pi) * (-1 if y < 0 else 1)
        spot = Spot(angle, offset)
        return spot
    
    def getAngleRads(self):
        return self.angle * 2 * np.pi / 360

def calculateScore(spots):
    angleScore = 0
    offsetScore = 0
    for spot in spots:
        angleScore += spot.angle**2
        offsetScore += spot.offset**2
    totalScore = angleScore + offsetScore
    totalScore = np.sqrt(totalScore)
    return totalScore

def plotSpots(spots, color=None, fig=None, label=None):
    if not fig:
        fig = plt.figure()
    plt.plot(0, 0, "o", color="k", figure=fig)
    kwargs = {}
    kwargs["figure"] = fig
    if color:
        kwargs["color"] = color
    if label:
        kwargs["label"] = label
    for spot in spots:
            plt.plot(spot.x, spot.y, '.', **kwargs)

def loadSpotsFromFile(filename):
    with open(filename, "r") as f:
        lines = f.readlines()

    spots = []
    for line in lines:
        parts = line.split("\t")
        angle = float(parts[0])
        offset = float(parts[1].strip("\n"))
        spot = Spot(angle, offset)
        spots.append(spot)
    return spots