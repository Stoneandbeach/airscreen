import matplotlib.pyplot as plt
import numpy as np

class Spot():
    def __init__(self, angle=0., offset=0.):
        self.angle = angle
        self.offset = offset
        self.x = offset * np.cos(angle*2*np.pi/360)
        self.y = offset * np.sin(angle*2*np.pi/360)
    
    def __str__(self):
        return str(self.angle) + ", " + str(self.offset) + ", " + str(self.x) + ", " + str(self.y)
    
    def __add__(self, other):
        self.x += other.x
        self.y += other.y
        self.offset = np.sqrt(self.x**2 + self.y**2)
        self.angle = np.arccos(self.x / self.offset) * 360 / (2 * np.pi) * (self.y < 0) * (-1)
        return self

with open("measurement1.txt", "r") as f:
    lines = f.readlines()

spots = []
for line in lines:
    parts = line.split("\t")
    angle = float(parts[0]) * 2 * np.pi / 360
    offset = float(parts[1].strip("\n"))
    spot = Spot(angle, offset)
    spots.append(spot)

diode = Spot(0., 0.28)
hole = Spot(-60., 0.08)
meas = diode + hole
print(meas)

fig = plt.figure()
plt.plot(0, 0, "o", color="k")
for spot in spots:
    #print(spot)
    plt.plot(spot.x, spot.y, 'x')
plt.show()