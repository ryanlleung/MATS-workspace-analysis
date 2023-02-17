
import numpy as np
from scipy.spatial.transform import Rotation
from scipy.optimize import fsolve

import matplotlib.pyplot as plt
from matplotlib.widgets import Slider, Button
from mpl_toolkits import mplot3d
from mpl_toolkits.mplot3d import Axes3D


# Define the relative stage base positions #
P = np.array([[0,50,0],
              [0,50,0],
              [0,50,0],
              [-300,300,0],
              [50,0,0],
              [250,0,0]])

# Function to calculate the transformation matrix of a PPRPR robot
# from the ground to the target centroid, based on stage positions
# Rx and Ry in degrees
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

    return T01,T02,T03,T04,T05,T06


# Function to calculate transformation from ground to target point
# Rx and Ry in degrees
def get_transformation_TP(X=0,Y=0,Z=0,Rx=0,Ry=0,TPx=0,TPy=0):

    T06 = get_transformation(X,Y,Z,Rx,Ry)[-1]
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


##################################################################### 



# Function to calculate the error between the current position and the setpoint
def error_function(q=[0,0,0,0,0],TSx=0,TSy=0,TSz=0,TSrx=0,TSry=0,TPx=0,TPy=0):

    T_guess = get_transformation_TP(q[0],q[1],q[2],q[3],q[4],TPx,TPy)
    
    # Get angles from T_guess
    R_guess = Rotation.from_matrix(T_guess[0:3,0:3])
    angles_guess = R_guess.as_euler('xyz', degrees=True)

    # Calculate the error
    error = np.array([T_guess[0,3]-TSx,
                      T_guess[1,3]-TSy,
                      T_guess[2,3]-TSz,
                      angles_guess[0]-TSrx,
                      angles_guess[1]-TSry])
    return error


# Function to calculate inverse kinematics
# TSx, TSy, TSz, TSrx, TSry are the target set point in space frame
# TPx, TPy are the target point coordinates in the target plane frame
def get_stage_positions(TSx,TSy,TSz,TSrx,TSry,TPx,TPy):

    q_solution = fsolve(error_function,[0,0,0,0,0],
                        args=(TSx,TSy,TSz,TSrx,TSry,TPx,TPy))

    return q_solution


print(get_stage_positions(0,450,0,0,30,25,0))



































































































# # Plot the workspace
# fig = plt.figure()
# ax = plt.axes(projection='3d')
# ax.view_init(elev=0,azim=-90)

# # Set the limits of the plot
# xlim = np.array([-500, 500])
# ylim = np.array([0, 800])
# zlim = np.array([-100, 100])

# ax.set_xlim(xlim)
# ax.set_ylim(ylim)
# ax.set_zlim(zlim)
# lims = np.array([xlim, ylim, zlim])
# min_value = np.min(lims)
# max_value = np.max(lims)
# norm_lims = (lims - min_value) / (max_value - min_value)
# ax.set_box_aspect(np.diff(norm_lims, axis=1).flatten())

# ax.set_xlabel('X')
# ax.set_ylabel('Y')
# ax.set_zlabel('Z')


# # Add sliders
# ax_x = plt.axes([0.25, 0.05, 0.65, 0.03])
# ax_y = plt.axes([0.25, 0.1, 0.65, 0.03])
# ax_z = plt.axes([0.25, 0.15, 0.65, 0.03])
# ax_rx = plt.axes([0.25, 0.2, 0.65, 0.03])
# ax_ry = plt.axes([0.25, 0.25, 0.65, 0.03])

# slider_x = Slider(ax_x, 'X', -125, 125, valinit=0, valstep=1)
# slider_y = Slider(ax_y, 'Y', -200, 200, valinit=0, valstep=1)
# slider_z = Slider(ax_z, 'Z', -50, 50, valinit=0, valstep=1)
# slider_rx = Slider(ax_rx, 'Rx', -45, 45, valinit=0, valstep=1)
# slider_ry = Slider(ax_ry, 'Ry', -45, 45, valinit=0, valstep=1)


# # Initialise the dots
# dot0 = ax.scatter([0], [0], [0], color='g', marker='X')
# dot1 = ax.scatter([0], [0], [0], color='b', marker='s')
# dot2 = ax.scatter([0], [0], [0], color='b', marker='s')
# dot3 = ax.scatter([0], [0], [0], color='b', marker='o')
# dot4 = ax.scatter([0], [0], [0], color='b', marker='s')
# dot5 = ax.scatter([0], [0], [0], color='b', marker='o')
# dot6 = ax.scatter([0], [0], [0], color='b', marker='.')
# dotTP = ax.scatter([0], [0], [0], color='r', marker='.')

# # Initialise the surface
# X = np.arange(0, 1, 0.1)
# Y = np.arange(0, 1, 0.1)
# X, Y = np.meshgrid(X, Y)
# Z = np.zeros_like(X)
# surf = ax.plot_surface(X, Y, Z, visible=False)


# def update(X=0,Y=0,Z=0,Rx=0,Ry=0,TPx=0,TPy=100):

#     # Get values from sliders
#     X = slider_x.val
#     Y = slider_y.val
#     Z = slider_z.val
#     Rx = slider_rx.val
#     Ry = slider_ry.val

#     # Get the transformation matrix
#     T01,T02,T03,T04,T05,T06 = get_transformation(X,Y,Z,Rx,Ry)
#     T0TP = get_transformation_TP(X,Y,Z,Rx,Ry,TPx,TPy)

#     # Set the position of dots
#     dot1._offsets3d = ([T01[0][3]],[T01[1][3]],[T01[2][3]])
#     dot2._offsets3d = ([T02[0][3]],[T02[1][3]],[T02[2][3]])
#     dot3._offsets3d = ([T03[0][3]],[T03[1][3]],[T03[2][3]])
#     dot4._offsets3d = ([T04[0][3]],[T04[1][3]],[T04[2][3]])
#     dot5._offsets3d = ([T05[0][3]],[T05[1][3]],[T05[2][3]])
#     dot6._offsets3d = ([T06[0][3]],[T06[1][3]],[T06[2][3]])
#     dotTP._offsets3d = ([T0TP[0][3]],[T0TP[1][3]],[T0TP[2][3]])

#     # Set the position of the plane
#     TS = get_target_plane(X,Y,Z,Rx,Ry)
#     print(TS)

#     # Remove the previous plane and plot the new one
#     global surf
#     surf.remove()
#     surf = ax.plot_surface(TS[:,:,0], TS[:,:,1], TS[:,:,2], color='r', alpha=0.2)
    
#     fig.canvas.draw_idle()

#     return


# # Call the update function when sliders are changed
# update() # Initialise the plot
# slider_x.on_changed(update)
# slider_y.on_changed(update)
# slider_z.on_changed(update)
# slider_rx.on_changed(update)
# slider_ry.on_changed(update)

# plt.show()
