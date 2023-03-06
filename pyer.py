
import time
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider, TextBox, Button

from scipy.spatial.transform import Rotation
from scipy.optimize import fsolve


class Ball:
    def __init__(self, center, diameter, color='blue'):
        self.center = center
        self.radius = diameter / 2
        self.color = color
        self.plot()

    def plot(self):
        global ax
        self.circle = plt.Circle(self.center, self.radius, color=self.color, fill=False)
        ax.add_patch(self.circle)

    def moveX(self, x):
        self.circle.remove()
        self.center = (x, 0)
        self.plot()

class Line:
    def __init__(self, L0, L, slopeAngle, color='blue'):
        self.L0 = L0
        slopeAngle = np.deg2rad(slopeAngle)
        self.L1 = (L0[0]+L*np.cos(slopeAngle), L0[1]+L*np.sin(slopeAngle))
        self.color = color
        self.l0 = self.L0
        self.l1 = self.L1
        self.plot()

    def plot(self):
        global ax
        self.line = plt.Line2D((self.l0[0], self.l1[0]), (self.l0[1], self.l1[1]), color=self.color)
        ax.add_line(self.line)

    def rotate(self, angle):
        self.line.remove()
        angle = np.deg2rad(angle)
        R = np.array([[np.cos(angle), -np.sin(angle)], [np.sin(angle), np.cos(angle)]])
        self.l0 = np.matmul(R, self.L0)
        self.l1 = np.matmul(R, self.L1)
        self.plot()

class Rectangle:
    def __init__(self, origin, width, height, color='blue'):
        self.origin = origin
        self.width = width
        self.height = height
        self.color = color
        self.plot()

    def plot(self):
        global ax
        self.rectangle = plt.Rectangle(self.origin-0.5*self.width, self.width, self.height, color=self.color, fill=False)
        ax.add_patch(self.rectangle)


# Create a new figure and axis
fig, ax = plt.subplots()
ax.set_title('Circle')
ax.set_xlabel('X')
ax.set_ylabel('Y')
xlim = np.array([-35, 200])
ylim = np.array([-50, 50])
# ax.set_xlim(xlim)
# ax.set_ylim(ylim)
plt.axhline(y=0, color='black', linewidth=0.5)
plt.axvline(x=0, color='black', linewidth=0.5)
plt.axhline(y=-12.5, color='black', linewidth=0.5)
plt.axhline(y=12.5, color='black', linewidth=0.5)
ax.set_aspect('equal', adjustable='box')
ax.grid()

ax_cx = plt.axes([0.2, 0.05, 0.65, 0.03])
ax_t = plt.axes([0.2, 0.1, 0.65, 0.03])

slider_cx = Slider(ax_cx, 'cx', 0, 150, valinit=100, valstep=0.2)
slider_t = Slider(ax_t, 'theta', -2, 2, valinit=0, valstep=0.1)

fixed_ball = Ball((0, 0), 25, 'black')
moving_ball = Ball((150, 0), 25)
# slope = Line((100, 5), 50, 37.5)


# move slope so that it is tangent to the circle




last_cx = 100
last_t = 0
def update(val=None):

    cx = slider_cx.val
    t = slider_t.val

    if cx == last_cx:
        moved_cx = False
        moved_t = True
    elif t == last_t:
        moved_cx = True
        moved_t = False
    
    last_cx = cx
    last_t = t

    if moved_cx:
        print('cx moved')

    moving_ball.moveX(cx)


    fig.canvas.draw_idle()

update() # Initialise the plot
slider_cx.on_changed(update)
slider_t.on_changed(update)




"""
# cx = 150
cy = 0
r = 12.5
lx = 81.5
ly = -5
phi = 60
# theta = 0

phi = np.deg2rad(phi)
# theta = np.deg2rad(theta)

def equations(vars):
    x, y, lamb, cx, theta = vars
    eq1 = (x-cx)**2 + (y-cy)**2 - r**2
    eq2 = -(lamb*np.sin(phi)+ly)*np.sin(theta) + (lamb*np.cos(phi)+lx)*np.cos(theta) - x
    eq3 = (lamb*np.sin(phi)+ly)*np.cos(theta) + (lamb*np.cos(phi)+lx)*np.sin(theta) - y
    eq4 = x + r*np.sin(phi+theta) - cx
    eq5 = y - r*np.cos(phi+theta) - cy
    return [eq1, eq2, eq3, eq4, eq5]

x0 = [0, 0, 0, 0, 0]
x, y, lamb, cx, theta = fsolve(equations, x0)
print(x, y, lamb, cx, np.rad2deg(theta))

ax.scatter(x, y, s=3, color='red')

moving_ball.moveX(cx)
slope = Line((lx, ly), 100, np.rad2deg(phi))
slope.rotate(np.rad2deg(theta))

plt.show()
"""