import matplotlib.pyplot as plt
import numpy as np
from helpers import *

measurement = loadSpotsFromFile("measurement1.txt")
diodes = loadSpotsFromFile("diodes.txt")
holes = loadSpotsFromFile("holes.txt")

holes.sort(key=lambda x : x.offset)

spots = []
for diode, hole in zip(diodes, holes):
    spot = diode + hole
    #print(diode, hole, spot)
    spots.append(spot)

trimmedSpots = []
rotatedDiodes = []
for diode, hole in zip(diodes, holes):
    angle = np.arcsin(-(hole.offset*np.sin(hole.getAngleRads())) / diode.offset) * 360 / (2 * np.pi)
    rotatedDiode = Spot(angle, diode.offset)
    print(rotatedDiode)
    print(diode)
    print(hole)
    print()
    trimmedSpot = rotatedDiode + hole
    trimmedSpots.append(trimmedSpot)
    rotatedDiodes.append(rotatedDiode)

fig = plt.figure()
plotSpots(measurement, color="k", fig=fig, label="Measurement")
#plotSpots(spots, color="yellow", fig=fig, label="Diodes + sorted holes")
plotSpots(trimmedSpots, color="red", fig=fig, label="Trimmed combo")
plotSpots(rotatedDiodes, color="blue", fig=fig, label="Rotated diodes")

plt.show()