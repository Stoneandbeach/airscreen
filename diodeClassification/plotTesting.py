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

fig = plt.figure()
plotSpots(measurement, color="k", fig=fig)
plotSpots(spots, color="blue", fig=fig)

plt.show()