import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d import Axes3D

# define the bounds of the plane
xmin, xmax = -1, 1
ymin, ymax = -1, 1

# create a meshgrid of the x and y coordinates
x, y = np.meshgrid(np.linspace(xmin, xmax, 10), np.linspace(ymin, ymax, 10))

# define the equation of the plane
a, b, c, d = 1, 2, 3, 4
z = -(a * x + b * y + d) / c

# create a 3D plot
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# plot the surface
ax.plot_surface(x, y, z, alpha=0.5)

# set the limits of the axes
ax.set_xlim3d(xmin, xmax)
ax.set_ylim3d(ymin, ymax)

# add labels for the axes
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')

plt.show()