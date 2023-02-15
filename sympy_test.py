
import numpy as np
import sympy
from sympy import pprint, nsolve
from scipy.spatial.transform import Rotation   


# Define the relative stage geometries
p = np.array([[0,50,0],
              [0,50,0],
              [0,50,0],
              [-300,300,0],
              [50,0,0],
              [250,0,0]])

# Set the symbolic variables
X, Y, Z, Rx, Ry = sympy.symbols('X Y Z Rx Ry')

# Matrix multiplication
T01 = sympy.Matrix([[1, 0, 0, p[0][0]+X],
                    [0, 1, 0, p[0][1]],
                    [0, 0, 1, p[0][2]],
                    [0, 0, 0, 1]])

T12 = sympy.Matrix([[1, 0, 0, p[1][0]],
                    [0, 1, 0, p[1][1]],
                    [0, 0, 1, p[1][2]+Z],
                    [0, 0, 0, 1]])

T23 = sympy.Matrix([[sympy.cos(Ry), 0, sympy.sin(Ry), p[2][0]],
                    [0, 1, 0, p[2][1]],
                    [-sympy.sin(Ry), 0, sympy.cos(Ry), p[2][2]],
                    [0, 0, 0, 1]])

T34 = sympy.Matrix([[1, 0, 0, p[3][0]],
                    [0, 1, 0, p[3][1]+Y],
                    [0, 0, 1, p[3][2]],
                    [0, 0, 0, 1]])

T45 = sympy.Matrix([[1, 0, 0, p[4][0]],
                    [0, sympy.cos(Rx), -sympy.sin(Rx), p[4][1]],
                    [0, sympy.sin(Rx), sympy.cos(Rx), p[4][2]],
                    [0, 0, 0, 1]])

T56 = sympy.Matrix([[1, 0, 0, p[5][0]],
                    [0, 1, 0, p[5][1]],
                    [0, 0, 1, p[5][2]],
                    [0, 0, 0, 1]])


# Set target point coordinates relative to target centroid
TPx = 0
TPy = 125
T6T = sympy.Matrix([[1, 0, 0, TPx],
                    [0, 1, 0, TPy],
                    [0, 0, 1, 0],
                    [0, 0, 0, 1]])


# Set target point coordinates relative to space frame

Tx, Ty, Tz, Trx, Try = 0, 0, 0, 0, 0

R = Rotation.from_euler('xyz', [Trx, Try, 0], degrees=True)
T = sympy.Matrix([[R.as_matrix()[0][0], R.as_matrix()[0][1], R.as_matrix()[0][2], Tx],
                    [R.as_matrix()[1][0], R.as_matrix()[1][1], R.as_matrix()[1][2], Ty],
                    [R.as_matrix()[2][0], R.as_matrix()[2][1], R.as_matrix()[2][2], Tz],
                    [0, 0, 0, 1]])

T0T = T01*T12*T23*T34*T45*T56*T6T


# Solve for T = T0T
solved = nsolve((T[0,0]-T0T[0,0], T[0,1]-T0T[0,1], T[0,2]-T0T[0,2], T[0,3]-T0T[0,3],
                    T[1,0]-T0T[1,0], T[1,1]-T0T[1,1], T[1,2]-T0T[1,2], T[1,3]-T0T[1,3],
                    T[2,0]-T0T[2,0], T[2,1]-T0T[2,1], T[2,2]-T0T[2,2], T[2,3]-T0T[2,3]), (X, Y, Z, Rx, Ry), (0, 0, 0, 0, 0))
solved = np.array(solved).reshape(1,5).squeeze().astype(np.float64)
solved[3] = np.rad2deg(solved[3])
solved[4] = np.rad2deg(solved[4])

print(solved)
