import matplotlib.pyplot as plt
import numpy as np
from helpers import *
import random

diodes = loadSpotsFromFile("diodesWithIDs.txt")
holes = loadSpotsFromFile("holes.txt")

# Hole positions
positionDict = {}
for hole in holes:
    row = 5 - (int(hole.id) - 1) % 6
    column = (int(hole.id) - 1) // 6
    position = (column * 7, row * 7)
    positionDict[hole.id] = position

order = list(range(len(holes)))

bestOrder = order
colors = ["k", "blue", "red", "green", "grey"]

lowScore = np.inf

for i in range(10000):
    holes = [hole for hole, _ in sorted(zip(holes, order), key=lambda x : x[1])]
    spots = []
    positions = []
    for diode, hole in zip(diodes, holes):
        ratio = (hole.offset*np.sin(hole.getAngleRads())) / diode.offset
        if ratio**2 < 1:
            angle = np.arcsin(-ratio) * 360 / (2 * np.pi)
        else:
            angle = (hole.angle + 180) % 360
        rotatedDiode = Spot(angle, diode.offset, id=diode.id)
        spot = rotatedDiode + hole
        spots.append(spot)
        positions.append(positionDict[hole.id])
    score = calculateScore(spots)
    if score < lowScore:
        print("Low score:", score)
        lowScore = score
        bestOrder = order
    random.shuffle(order)

order = bestOrder
holes = [hole for hole, _ in sorted(zip(holes, order), key=lambda x : x[1])]
spots = []
positions = []
for diode, hole in zip(diodes, holes):
    ratio = (hole.offset*np.sin(hole.getAngleRads())) / diode.offset
    if ratio**2 < 1:
        angle = np.arcsin(-ratio) * 360 / (2 * np.pi)
    else:
        angle = (hole.angle + 180) % 360
    rotatedDiode = Spot(angle, diode.offset, id=diode.id)
    spot = rotatedDiode + hole
    spot.id = hole.id
    spots.append(spot)
    positions.append(positionDict[hole.id])
plotSpots(spots, basePositions=positions, color="k")
print(bestOrder)
print()
print("Best score:", lowScore)

plt.show()
