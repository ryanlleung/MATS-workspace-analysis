
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider, Button
from mpl_toolkits import mplot3d
from mpl_toolkits.mplot3d import Axes3D
from scipy.spatial.transform import Rotation   


# XX, YY represents a variable target centroid position
# Should be named something else like ptx, pty to avoid confusion
# with the target point coordinates


# Plot the workspace
fig = plt.figure()
ax = plt.axes(projection='3d')
ax.view_init(elev=0,azim=-90)
ax.set_xlim3d(-500,500)
ax.set_ylim3d(0,1000)
ax.set_zlim3d(-100,100)
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
ax.set_box_aspect([1, 1, 0.2])

# Add sliders
ax_x = plt.axes([0.25, 0.05, 0.65, 0.03])
ax_y = plt.axes([0.25, 0.1, 0.65, 0.03])
ax_z = plt.axes([0.25, 0.15, 0.65, 0.03])
ax_rx = plt.axes([0.25, 0.2, 0.65, 0.03])
ax_ry = plt.axes([0.25, 0.25, 0.65, 0.03])
ax_xx = plt.axes([0.25, 0.3, 0.65, 0.03])
ax_yy = plt.axes([0.25, 0.35, 0.65, 0.03])

slider_x = Slider(ax_x, 'X', -125, 125, valinit=0, valstep=1)
slider_y = Slider(ax_y, 'Y', -200, 200, valinit=0, valstep=1)
slider_z = Slider(ax_z, 'Z', -50, 50, valinit=0, valstep=1)
slider_rx = Slider(ax_rx, 'Rx', -45, 45, valinit=0, valstep=1)
slider_ry = Slider(ax_ry, 'Ry', -45, 45, valinit=0, valstep=1)
slider_xx = Slider(ax_xx, 'XX', -125, 125, valinit=0, valstep=1)
slider_yy = Slider(ax_yy, 'YY', -200, 200, valinit=0, valstep=1)

# Initialise the dots
dot0 = ax.scatter([0], [0], [0], color='g', marker='X')
dot1 = ax.scatter([0], [0], [0], color='b', marker='s')
dot2 = ax.scatter([0], [0], [0], color='b', marker='s')
dot3 = ax.scatter([0], [0], [0], color='b', marker='o')
dot4 = ax.scatter([0], [0], [0], color='b', marker='s')
dot5 = ax.scatter([0], [0], [0], color='b', marker='o')
dot6 = ax.scatter([0], [0], [0], color='r', marker='.')

# Initialise the surface
X = np.arange(0, 1, 0.1)
Y = np.arange(0, 1, 0.1)
X, Y = np.meshgrid(X, Y)
Z = np.zeros_like(X)
surf = ax.plot_surface(X, Y, Z, visible=False)

# Define the relative stage geometries
p = np.array([[0,50,0],
              [0,50,0],
              [0,50,0],
              [-300,300,0],
              [50,0,0],
              [250,0,0]])

# Define the update function
def update(X=0,Y=0,Z=0,Rx=0,Ry=0,XX=0,YY=0):

    # Get values from sliders
    X = slider_x.val
    Y = slider_y.val
    Z = slider_z.val
    Rx = slider_rx.val
    Ry = slider_ry.val
    XX = slider_xx.val
    YY = slider_yy.val

    # Convert the stage rotations to radians
    Rx = np.deg2rad(Rx)
    Ry = np.deg2rad(Ry)

    # Define transformation matrices

    T01 = np.array([[1,0,0,p[0][0]+X],
                    [0,1,0,p[0][1]],
                    [0,0,1,p[0][2]],
                    [0,0,0,1]])

    T12 = np.array([[1,0,0,p[1][0]],
                    [0,1,0,p[1][1]],
                    [0,0,1,p[1][2]+Z],
                    [0,0,0,1]])

    T23 = np.array([[np.cos(Ry),0,np.sin(Ry),p[2][0]],
                    [0,1,0,p[2][1]],                                
                    [-np.sin(Ry),0,np.cos(Ry),p[2][2]],
                    [0,0,0,1]])

    T34 = np.array([[1,0,0,p[3][0]],
                    [0,1,0,p[3][1]+Y],
                    [0,0,1,p[3][2]],
                    [0,0,0,1]])

    T45 = np.array([[1,0,0,p[4][0]],
                    [0,np.cos(Rx),-np.sin(Rx),p[4][1]],
                    [0,np.sin(Rx),np.cos(Rx),p[4][2]],
                    [0,0,0,1]])

    # Define target centroid
    T56 = np.array([[1,0,0,p[5][0]+XX],
                    [0,1,0,p[5][1]+YY],
                    [0,0,1,p[5][2]],
                    [0,0,0,1]])

    # Calculate the transformation matrices
    T02 = np.dot(T01,T12)
    T03 = np.dot(T02,T23)
    T04 = np.dot(T03,T34)
    T05 = np.dot(T04,T45)
    T06 = np.dot(T05,T56)

    # Define target plane
    density = 2
    Tx = np.linspace(-125,125,density)
    Ty = np.linspace(-125,125,density)
    TS = np.zeros((density,density,3))

    for i in range(0,density):
        for j in range(0,density):
            T6T = np.array([[1,0,0,Tx[i]],
                            [0,1,0,Ty[j]],
                            [0,0,1,0],
                            [0,0,0,1]])
            T0T = np.dot(T06,T6T)
            for k in range(0,3):
                TS[i][j][k] = T0T[k][3]


    # Print the position of dots in integer values
    # print('X: ', int(T01[0][3]), 'Y: ', int(T01[1][3]), 'Z: ', int(T01[2][3]))
    # print('X: ', int(T02[0][3]), 'Y: ', int(T02[1][3]), 'Z: ', int(T02[2][3]))
    # print('X: ', int(T03[0][3]), 'Y: ', int(T03[1][3]), 'Z: ', int(T03[2][3]))
    # print('X: ', int(T04[0][3]), 'Y: ', int(T04[1][3]), 'Z: ', int(T04[2][3]))
    # print('X: ', int(T05[0][3]), 'Y: ', int(T05[1][3]), 'Z: ', int(T05[2][3]))
    # print('X: ', int(T06[0][3]), 'Y: ', int(T06[1][3]), 'Z: ', int(T06[2][3]))
    # print('\n')

    # print T0T in integer values
    # print([np.round(T06[0][0],2),np.round(T06[0][1],2),np.round(T06[0][2],2),np.round(T06[0][3],2)])
    # print([np.round(T06[1][0],2),np.round(T06[1][1],2),np.round(T06[1][2],2),np.round(T06[1][3],2)])
    # print([np.round(T06[2][0],2),np.round(T06[2][1],2),np.round(T06[2][2],2),np.round(T06[2][3],2)])
    # print([np.round(T06[3][0],2),np.round(T06[3][1],2),np.round(T06[3][2],2),np.round(T06[3][3],2)])


    # Calculate the angles of the target centroid
    R6 =  Rotation.from_matrix(T06[:3,:3])
    angles = R6.as_euler("xyz",degrees=True)
    print(np.round(angles,2))

    # Set the position of dots
    dot1._offsets3d = ([T01[0][3]],[T01[1][3]],[T01[2][3]])
    dot2._offsets3d = ([T02[0][3]],[T02[1][3]],[T02[2][3]])
    dot3._offsets3d = ([T03[0][3]],[T03[1][3]],[T03[2][3]])
    dot4._offsets3d = ([T04[0][3]],[T04[1][3]],[T04[2][3]])
    dot5._offsets3d = ([T05[0][3]],[T05[1][3]],[T05[2][3]])
    dot6._offsets3d = ([T06[0][3]],[T06[1][3]],[T06[2][3]])

    # Plot the target plane
    global surf
    surf.remove()
    surf = ax.plot_surface(TS[:,:,0], TS[:,:,1], TS[:,:,2], color='r', alpha=0.2)
    
    fig.canvas.draw_idle()

    return


# Call the update function when sliders are changed
update() # Initialise the plot
slider_x.on_changed(update)
slider_y.on_changed(update)
slider_z.on_changed(update)
slider_rx.on_changed(update)
slider_ry.on_changed(update)
slider_xx.on_changed(update)
slider_yy.on_changed(update)

plt.show()
