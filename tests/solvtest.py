
import numpy as np
from scipy.optimize import fsolve

cx = 150
cy = 0
r = 12.5
lx = 100
ly = 5
phi = 37.5
theta = 0

phi = np.deg2rad(phi)
theta = np.deg2rad(theta)

def equations(vars):
    x, y, lamb = vars
    eq1 = (x-cx)**2 + (y-cy)**2 - r**2
    eq2 = -(lamb*np.sin(phi)+ly)*np.sin(theta) + (lamb*np.cos(phi)+lx)*np.cos(theta) - x
    eq3 = (lamb*np.sin(phi)+ly)*np.cos(theta) + (lamb*np.cos(phi)+lx)*np.sin(theta) - y
    return [eq1, eq2, eq3]

x0 = [0, 0, 0]
x = fsolve(equations, x0)
print(x)

