
import sympy
from sympy import pprint, nsolve
from scipy.spatial.transform import Rotation   
import numpy as np

import matplotlib.pyplot as plt
from matplotlib.widgets import Slider, Button
from mpl_toolkits import mplot3d
from mpl_toolkits.mplot3d import Axes3D


# Define the relative stage positions #
P = np.array([[0,50,0],
              [0,50,0],
              [0,50,0],
              [-300,300,0],
              [50,0,0],
              [250,0,0]])

# Function to calculate the transformation matrix 
# from the ground to the target centroid
def get_transformation(X=0,Y=0,Z=0,Rx=0,Ry=0):

    # Convert to radians
    Rx = np.deg2rad(Rx)
    Ry = np.deg2rad(Ry)

    ## Define transformation matrices ##

    # Ground to stage X
    T01 = np.array([[1,0,0,P[0][0]+X],
                    [0,1,0,P[0][1]],
                    [0,0,1,P[0][2]],
                    [0,0,0,1]])

    # Stage X to stage Z
    T12 = np.array([[1,0,0,P[1][0]],
                    [0,1,0,P[1][1]],
                    [0,0,1,P[1][2]+Z],
                    [0,0,0,1]])

    # Stage Z to stage Ry
    T23 = np.array([[np.cos(Ry),0,np.sin(Ry),P[2][0]],
                    [0,1,0,P[2][1]],                                
                    [-np.sin(Ry),0,np.cos(Ry),P[2][2]],
                    [0,0,0,1]])

    # Stage Ry to stage Y
    T34 = np.array([[1,0,0,P[3][0]],
                    [0,1,0,P[3][1]+Y],
                    [0,0,1,P[3][2]],
                    [0,0,0,1]])

    # Stage Y to stage Rx
    T45 = np.array([[1,0,0,P[4][0]],
                    [0,np.cos(Rx),-np.sin(Rx),P[4][1]],
                    [0,np.sin(Rx),np.cos(Rx),P[4][2]],
                    [0,0,0,1]])

    # Stage Rx to target centroid
    T56 = np.array([[1,0,0,P[5][0]],
                    [0,1,0,P[5][1]],
                    [0,0,1,P[5][2]],
                    [0,0,0,1]])

    # Calculate the transformation matrices
    T02 = np.dot(T01,T12)
    T03 = np.dot(T02,T23)
    T04 = np.dot(T03,T34)
    T05 = np.dot(T04,T45)
    T06 = np.dot(T05,T56)

    return T06


# Function to calculate transformation from ground to target point
def get_transformation_TP(X=0,Y=0,Z=0,Rx=0,Ry=0,TPx=0,TPy=0):

    T06 = get_transformation(X,Y,Z,Rx,Ry)
    T6TP = np.array([[1,0,0,TPx],
                     [0,1,0,TPy],
                     [0,0,1,0],
                     [0,0,0,1]])
    T0TP = np.dot(T06,T6TP)
    return T0TP


# Function to generate target plane surface
def get_target_plane(X=0,Y=0,Z=0,Rx=0,Ry=0,width=250,height=250,density=2):

    Tx = np.linspace(-0.5*width,0.5*width,density)
    Ty = np.linspace(-0.5*height,0.5*height,density)
    TS = np.zeros((density,density,3))

    for i in range(0,density):
        for j in range(0,density):
            # Abuse get_transformation_TP to get the plane mesh coordinates
            T0TP = get_transformation_TP(X,Y,Z,Rx,Ry,Tx[i],Ty[j])
            for k in range(0,3):
                # Extract the coordinates
                TS[i][j][k] = T0TP[k][3]

    return TS


# Function to calculate X,Y,Z,Rx,Ry from the target point
def get_stage_positions(TPx=0,TPy=0,TSx=0,TSy=0,TSz=0,TSrx=0,TSry=0):

    # Set the symbolic variables
    X, Y, Z, Rx, Ry = sympy.symbols('X Y Z Rx Ry')

    # Matrix multiplication
    T01 = sympy.Matrix([[1, 0, 0, P[0][0]+X],
                        [0, 1, 0, P[0][1]],
                        [0, 0, 1, P[0][2]],
                        [0, 0, 0, 1]])

    T12 = sympy.Matrix([[1, 0, 0, P[1][0]],
                        [0, 1, 0, P[1][1]],
                        [0, 0, 1, P[1][2]+Z],
                        [0, 0, 0, 1]])

    T23 = sympy.Matrix([[sympy.cos(Ry), 0, sympy.sin(Ry), P[2][0]],
                        [0, 1, 0, P[2][1]],
                        [-sympy.sin(Ry), 0, sympy.cos(Ry), P[2][2]],
                        [0, 0, 0, 1]])

    T34 = sympy.Matrix([[1, 0, 0, P[3][0]],
                        [0, 1, 0, P[3][1]+Y],
                        [0, 0, 1, P[3][2]],
                        [0, 0, 0, 1]])

    T45 = sympy.Matrix([[1, 0, 0, P[4][0]],
                        [0, sympy.cos(Rx), -sympy.sin(Rx), P[4][1]],
                        [0, sympy.sin(Rx), sympy.cos(Rx), P[4][2]],
                        [0, 0, 0, 1]])

    T56 = sympy.Matrix([[1, 0, 0, P[5][0]],
                        [0, 1, 0, P[5][1]],
                        [0, 0, 1, P[5][2]],
                        [0, 0, 0, 1]])

    T6TP = sympy.Matrix([[1, 0, 0, TPx],
                        [0, 1, 0, TPy],
                        [0, 0, 1, 0],
                        [0, 0, 0, 1]])

    # Define the target set pose relative to space frame
    TSR = Rotation.from_euler('xyz', [TSrx,TSry,0], degrees=True).as_matrix()

 



