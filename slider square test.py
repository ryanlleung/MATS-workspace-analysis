import matplotlib.pyplot as plt
from matplotlib.widgets import Slider
from mpl_toolkits import mplot3d
import numpy as np

def rotate(origin, point, angle):
    """
    Rotate a point counterclockwise by a given angle around a given origin.
    """
    ox, oy, oz = origin
    px, py, pz = point

    qx = ox + np.cos(angle) * (px - ox) - np.sin(angle) * (py - oy)
    qy = oy + np.sin(angle) * (px - ox) + np.cos(angle) * (py - oy)
    qz = oz + pz - oz

    return qx, qy, qz

def translate(x, y, z, dx, dy, dz):
    """
    Translate a point by a given amount in each dimension.
    """
    return x + dx, y + dy, z + dz

fig = plt.figure()
ax = plt.axes(projection='3d')

# Define the vertices of a square
origin = [0, 0, 0]
v1 = [1, 1, 0]
v2 = [-1, 1, 0]
v3 = [-1, -1, 0]
v4 = [1, -1, 0]

# Translate the square to its initial position
dx, dy, dz = [0], [0], [0]
v1 = translate(v1[0], v1[1], v1[2], dx[0], dy[0], dz[0])
v2 = translate(v2[0], v2[1], v2[2], dx[0], dy[0], dz[0])
v3 = translate(v3[0], v3[1], v3[2], dx[0], dy[0], dz[0])
v4 = translate(v4[0], v4[1], v4[2], dx[0], dy[0], dz[0])

# Rotate the square to its initial orientation
angle = [0]
v1 = rotate(origin, v1, angle[0])
v2 = rotate(origin, v2, angle[0])
v3 = rotate(origin, v3, angle[0])
v4 = rotate(origin, v4, angle[0])

square = [v1, v2, v3, v4, v1]
square = np.array(square)
x, y, z = square[:,0], square[:,1], square[:,2]
square_line = ax.plot(x, y, z, color='red')

ax.set_xlim([-10, 10])
ax.set_ylim([-10, 10])
ax.set_zlim([-10, 10])

# Create sliders for translation and rotation
ax_dx = plt.axes([0.25, 0.1, 0.65, 0.03])
ax_dy = plt.axes([0.25, 0.15, 0.65, 0.03])

plt.show()