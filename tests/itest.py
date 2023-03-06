
import numpy as np
import sympy
from sympy import pprint, nsolve
from scipy.spatial.transform import Rotation   


# Set the symbolic variables
x, y, theta, phi, lamb, lx, ly, cx, cy = sympy.symbols('x y theta phi lambda lx ly cx cy')

# Matrix
L = sympy.Matrix([[lx + lamb*sympy.cos(phi)], 
                  [ly + lamb*sympy.sin(phi)]])
R = sympy.Matrix([[sympy.cos(theta), -sympy.sin(theta)], 
                  [sympy.sin(theta), sympy.cos(theta)]])

# Multiply matrices
M = R*L

# print the m
pprint(R)
pprint(L)
pprint(M)

