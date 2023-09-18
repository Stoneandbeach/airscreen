import matplotlib.pyplot as plt
import numpy as np
from helpers import *

measurement = loadSpotsFromFile("measurement1.txt")
diodes = loadSpotsFromFile("diodesWithIDs.txt")
holes = loadSpotsFromFile("holes.txt")

# Hole positions
positions1 = []
for hole in holes:
    row = 5 - (int(hole.id) - 1) % 6
    column = (int(hole.id) - 1) // 6
    position = (column * 7, row * 7)
    positions1.append(position)

holes.sort(key=lambda x : x.offset)

spots = []
for diode, hole in zip(diodes, holes):
    spot = diode + hole
    #print(diode, hole, spot)
    spots.append(spot)

# Hole positions
positions2 = []
for hole in holes:
    row = 5 - (int(hole.id) - 1) % 6
    column = (int(hole.id) - 1) // 6
    position = (column * 7, row * 7)
    positions2.append(position)

trimmedSpots = []
rotatedDiodes = []
for diode, hole in zip(diodes, holes):
    angle = np.arcsin(-(hole.offset*np.sin(hole.getAngleRads())) / diode.offset) * 360 / (2 * np.pi)
    rotatedDiode = Spot(angle, diode.offset, id=diode.id)
    print(rotatedDiode)
    print(diode)
    print(hole)
    print()
    trimmedSpot = rotatedDiode + hole
    trimmedSpots.append(trimmedSpot)
    rotatedDiodes.append(rotatedDiode)

fig = plt.figure()
plotSpots(measurement, color="k", fig=fig, label="Measurement", basePositions=positions1)
#plotSpots(spots, color="yellow", fig=fig, label="Diodes + sorted holes")
plotSpots(trimmedSpots, color="red", fig=fig, label="Trimmed combo", basePositions=positions2)
#plotSpots(rotatedDiodes, color="blue", fig=fig, label="Rotated diodes")

plt.show()

pairings = []
for diode, hole in zip(rotatedDiodes, holes):
    pair = (diode, hole)
    pairings.append(pair)

pairings.sort(key=lambda x : int(x[1].id))
for pair in pairings:
    print(f"{pair[0].id:3} {pair[1].id:3} {pair[0].angle:.2f}")