import numpy as np
import matplotlib.pyplot as plt

class Spot():
    def __init__(self, angle=0., offset=0., id=None):
        self.angle = angle
        self.offset = offset
        self.x = offset * np.cos(angle*2*np.pi/360)
        self.y = offset * np.sin(angle*2*np.pi/360)
        self.id = id
    
    def __str__(self):
        return str(self.angle) + ", " + str(self.offset) + ", " + str(self.x) + ", " + str(self.y)
    
    def __add__(self, other):
        x = self.x + other.x
        y = self.y + other.y
        offset = np.sqrt(x**2 + y**2)
        ratio = x / offset
        angle = np.arccos(ratio) * 360 / (2 * np.pi) * (-1 if y < 0 else 1)
        spot = Spot(angle, offset)
        return spot
    
    def getAngleRads(self):
        return self.angle * 2 * np.pi / 360

def calculateColumnOffsetScoreForOne(columnSpots, index):
    spot = columnSpots[index]
    columnMean = np.mean([spot.offset for spot in columnSpots])
    columnOffsetScore = np.sqrt((spot.offset - columnMean)**2)
    return columnOffsetScore

def calculateScore(spots):
    angleScore = 0
    offsetScore = 0
    columnScore = 0
    for column in range(5):
        columnSpots = spots[column : column+6]
        columnMean = np.mean([spot.offset for spot in columnSpots])
        columnOffsetRMS = 0
        for spot in columnSpots:
            columnOffsetRMS += (spot.offset - columnMean)**2
        columnScore += columnOffsetRMS

    for spot in spots:
        angleScore += (spot.angle / 360)**2
        offsetScore += spot.offset**2
    
    totalScore = columnOffsetRMS
    totalScore = np.sqrt(totalScore)
    return totalScore

def plotSpots(spots, color=None, fig=None, label=None, basePositions=None):
    if not fig:
        fig = plt.figure()
    plt.scatter(0, 0, marker="o", figure=fig, facecolor="none", edgecolor="k")
    kwargs = {}
    kwargs["figure"] = fig
    if color:
        kwargs["color"] = color
    if label:
        kwargs["label"] = label
    for s, spot in enumerate(spots):
        x = spot.x
        y = spot.y
        if basePositions:
            x *= 20
            y *= 20
            x += basePositions[s][0]
            y += basePositions[s][1]
            marker = ["o", "x", "p", "v", "s", "^"][basePositions[s][0] // 7]
            kwargs["marker"] = marker
        plt.plot(x, y, **kwargs)

def loadSpotsFromFile(filename):
    with open(filename, "r") as f:
        lines = f.readlines()

    spots = []
    for line in lines:
        parts = line.split("\t")
        angle = float(parts[0])
        offset = float(parts[1].strip("\n"))
        spot = Spot(angle, offset)
        if len(parts) > 2:
            id = parts[2].strip("\n")
            spot.id = id
        spots.append(spot)
    return spots