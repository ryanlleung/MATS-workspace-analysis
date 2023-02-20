
# Import libraries
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from scipy.spatial.transform import Rotation


## Geometries are Z-up, this program is not used ##

# Class to plot a 3D linear stage
class LinearStage:
    
    # Initialize the stage
    def __init__(self, ax, dims=[400,50,50], thickness=None, color=0, alpha=0.25):
        self.ax = ax
        self.l, self.w, self.h = dims
        self.color = color
        self.alpha = alpha

        self.thickness = thickness
        if thickness == None: self.thickness = self.h/6
            
        self.plot_called = False

        return
        
    # Function to plot the stage
    def plot(self, S=None, pose=None):
        
        # Check if last S and pose are defined
        if not hasattr(self, 'last_S'):
            self.last_S = 0
        if not hasattr(self, 'last_pose'):
            self.last_pose = [0,0,0,0,0,0]

        # If no S is given, use the last S
        if S == None:
            S = self.last_S
        # If no pose is given, use the last pose
        if pose == None:
            pose = self.last_pose
        
        # Unpack the pose
        self.cx, self.cy, self.cz, self.rx, self.ry, self.rz = pose
        
        # The rotation is inverted
        self.rx = -np.deg2rad(self.rx)
        self.ry = -np.deg2rad(self.ry)
        self.rz = -np.deg2rad(self.rz)

        # Plot base
        self.base_height = self.h - self.thickness
        self.base = plot_cuboid(self.ax, self.l, self.w, self.base_height, pose=pose, color=self.color, alpha=self.alpha)
        self.base_dot = plot_dot(self.ax, position=[self.cx, self.cy, self.cz], color='g')

        # Convert base to Euler angles and to transformation matrix rotating in xyz
        T0b = np.array([
            [np.cos(self.rz)*np.cos(self.ry), np.cos(self.rz)*np.sin(self.ry)*np.sin(self.rx)-np.sin(self.rz)*np.cos(self.rx), np.cos(self.rz)*np.sin(self.ry)*np.cos(self.rx)+np.sin(self.rz)*np.sin(self.rx), self.cx],
            [np.sin(self.rz)*np.cos(self.ry), np.sin(self.rz)*np.sin(self.ry)*np.sin(self.rx)+np.cos(self.rz)*np.cos(self.rx), np.sin(self.rz)*np.sin(self.ry)*np.cos(self.rx)-np.cos(self.rz)*np.sin(self.rx), self.cy],
            [-np.sin(self.ry), np.cos(self.ry)*np.sin(self.rx), np.cos(self.ry)*np.cos(self.rx), self.cz],
            [0,0,0,1]
        ])

        Tbp = np.array([
            [1,0,0,S],
            [0,1,0,0],
            [0,0,1,self.base_height],
            [0,0,0,1]
        ])

        T0p = np.dot(T0b, Tbp)

        # Plot the platform
        plat_width = plat_length = self.w - 10
        R =  Rotation.from_matrix(T0p[:3,:3])
        plat_angles = R.as_euler("xyz",degrees=True)

        # The rotation is inverted
        self.platform_pose = [T0p[0,3], T0p[1,3], T0p[2,3], -plat_angles[0], -plat_angles[1], -plat_angles[2]]

        self.plat = plot_cuboid(self.ax, plat_length, plat_width, self.thickness, pose=self.platform_pose, color=self.color, alpha=.5)
        self.plat_dot = plot_dot(self.ax, position=[T0p[0,3], T0p[1,3], T0p[2,3]], color=self.color)

        # Set flags
        self.plot_called = True
        self.last_pose = pose
        self.last_S = S

        return
    
    # Function to update the stage platform position
    def movePlatform(self,S=0):
        
        # Run the plot function if it hasn't been called yet
        if not self.plot_called: self.plot(S=S)

        self.base.remove()
        self.base_dot.remove()
        self.plat.remove()
        self.plat_dot.remove()

        self.S = S
        self.plot(S=S)

        return
    
    # Function to update the stage base position
    def moveBase(self,pose=[0,0,0,0,0,0]):

        # Run the plot function if it hasn't been called yet
        if not self.plot_called: self.plot(pose=pose)

        self.base.remove()
        self.base_dot.remove()
        self.plat.remove()
        self.plat_dot.remove()

        self.plot(pose=pose)

        return
    

# Class to plot a 3D rotary stage
class RotaryStage:
        
    # Initialize the stage
    def __init__(self, ax, dims=[400,50,50], thickness=None, color=0, alpha=0.25):
        
        self.ax = ax
        self.l, self.w, self.h = dims
        self.color = color
        self.alpha = alpha
        
        self.thickness = thickness
        if thickness == None: self.thickness = self.h/6

        self.plot_called = False

        return
    
        
    # Function to plot the stage
    def plot(self, S=None, pose=None):
        
        # Check if last S and pose are defined
        if not hasattr(self, 'last_S'):
            self.last_S = 0
        if not hasattr(self, 'last_pose'):
            self.last_pose = [0,0,0,0,0,0]

        # If no S is given, use the last S
        if S == None:
            S = self.last_S
        # If no pose is given, use the last pose
        if pose == None:
            pose = self.last_pose
        
        # Unpack the pose
        self.cx, self.cy, self.cz, self.rx, self.ry, self.rz = pose
        
        # The rotation is inverted
        self.rx = -np.deg2rad(self.rx)
        self.ry = -np.deg2rad(self.ry)
        self.rz = -np.deg2rad(self.rz)

        # Plot base
        self.base_height = self.h - self.thickness
        self.base = plot_cuboid(self.ax, self.l, self.w, self.base_height, pose=pose, color=self.color, alpha=self.alpha)
        self.base_dot = plot_dot(self.ax, position=[self.cx, self.cy, self.cz], color='g')

        # Convert base to Euler angles and to transformation matrix rotating in xyz
        T0b = np.array([
            [np.cos(self.rz)*np.cos(self.ry), np.cos(self.rz)*np.sin(self.ry)*np.sin(self.rx)-np.sin(self.rz)*np.cos(self.rx), np.cos(self.rz)*np.sin(self.ry)*np.cos(self.rx)+np.sin(self.rz)*np.sin(self.rx), self.cx],
            [np.sin(self.rz)*np.cos(self.ry), np.sin(self.rz)*np.sin(self.ry)*np.sin(self.rx)+np.cos(self.rz)*np.cos(self.rx), np.sin(self.rz)*np.sin(self.ry)*np.cos(self.rx)-np.cos(self.rz)*np.sin(self.rx), self.cy],
            [-np.sin(self.ry), np.cos(self.ry)*np.sin(self.rx), np.cos(self.ry)*np.cos(self.rx), self.cz],
            [0,0,0,1]
        ])

        Tbp = np.array([
            [np.cos(S), -np.sin(S), 0, 0],
            [np.sin(S), np.cos(S), 0, 0],
            [0,0,1,self.base_height],
            [0,0,0,1]
        ])

        T0p = np.dot(T0b, Tbp)

        # Plot the platform
        plat_r = (self.w - 2.5)/2
        R =  Rotation.from_matrix(T0p[:3,:3])
        plat_angles = R.as_euler("xyz",degrees=True)
        # The rotation is inverted
        self.platform_pose = [T0p[0,3], T0p[1,3], T0p[2,3], -plat_angles[0], -plat_angles[1], -plat_angles[2]]

        self.plat0, self.plat1 = plot_cylinder(self.ax, plat_r, self.thickness, pose=self.platform_pose, color=self.color, alpha=.5)
        self.plat_dot = plot_dot(self.ax, position=[T0p[0,3], T0p[1,3], T0p[2,3]], color=self.color)

        # Set flags
        self.plot_called = True
        self.last_pose = pose
        self.last_S = S

        return
    
    # Function to update the stage platform position
    def movePlatform(self,S=0):
        
        # Run the plot function if it hasn't been called yet
        if not self.plot_called: self.plot(S=S)

        self.base.remove()
        self.base_dot.remove()
        self.plat0.remove()
        self.plat1.remove()
        self.plat_dot.remove()

        self.S = S
        self.plot(S=S)

        return
    
    # Function to update the stage base position
    def moveBase(self,pose=[0,0,0,0,0,0]):

        # Run the plot function if it hasn't been called yet
        if not self.plot_called: self.plot(pose=pose)

        self.base.remove()
        self.base_dot.remove()
        self.plat0.remove()
        self.plat1.remove()
        self.plat_dot.remove()

        self.plot(pose=pose)

        return
    

# Function to plot a dot
def plot_dot(ax, position=[], color='r', alpha=1):
    if type(color) != str:
        color = f"C{color}"
    dot = ax.scatter(position[0], position[1], position[2], color=color, alpha=alpha)
    return dot


# Function to plot a line
def plot_line(ax, start=[0,0,0], end=[0,0,0], color='r', alpha=1):
    if type(color) != str:
        color = f"C{color}"
    line = ax.plot([start[0], end[0]], [start[1], end[1]], [start[2], end[2]], color=color, alpha=alpha)
    return line


# Function to plot a cuboid with origin at centroid of bottom face
def plot_cuboid(ax, l, w, h, pose=[0,0,0,0,0,0], color=0, alpha=0.25):
    
    # Process the arguments
    cx, cy, cz, rx, ry, rz = pose
    if type(color) != str:
        color = f"C{color}"

    vertices = np.array([
        [-l/2, -w/2, 0],
        [-l/2, w/2, 0],
        [l/2, w/2, 0],
        [l/2, -w/2, 0],
        [-l/2, -w/2, h],
        [-l/2, w/2, h],
        [l/2, w/2, h],
        [l/2, -w/2, h]
    ])

    # Define the rotation matrix based on the rx, ry, rz angles
    rx = np.radians(rx)
    ry = np.radians(ry)
    rz = np.radians(rz)
    rotation_matrix = np.array([
        [np.cos(ry)*np.cos(rz), -np.cos(ry)*np.sin(rz), np.sin(ry)],
        [np.sin(rx)*np.sin(ry)*np.cos(rz) + np.cos(rx)*np.sin(rz), -np.sin(rx)*np.sin(ry)*np.sin(rz) + np.cos(rx)*np.cos(rz), -np.sin(rx)*np.cos(ry)],
        [-np.cos(rx)*np.sin(ry)*np.cos(rz) + np.sin(rx)*np.sin(rz), np.cos(rx)*np.sin(ry)*np.sin(rz) + np.sin(rx)*np.cos(rz), np.cos(rx)*np.cos(ry)]
    ])

    # Rotate the vertices and translate to the desired position
    vertices = np.dot(vertices, rotation_matrix) + np.array([cx, cy, cz])

    # Define the faces of the cuboid
    faces = [
        [vertices[0], vertices[1], vertices[2], vertices[3]],
        [vertices[0], vertices[1], vertices[5], vertices[4]],
        [vertices[1], vertices[2], vertices[6], vertices[5]],
        [vertices[2], vertices[3], vertices[7], vertices[6]],
        [vertices[3], vertices[0], vertices[4], vertices[7]],
        [vertices[4], vertices[5], vertices[6], vertices[7]]
    ]

    # Create a Poly3DCollection with a solid face color and a wireframe
    collection = Poly3DCollection(faces, alpha=alpha, facecolor=color, edgecolor='black')
    collection.set_linewidth(0.5)
    cuboid = ax.add_collection3d(collection)

    return cuboid


# Function to plot a cylinder with origin at centroid of bottom face
def plot_cylinder(ax, r, h, pose=[0,0,0,0,0,0], color=0, alpha=0.25):
    
    # Process the arguments
    cx, cy, cz, rx, ry, rz = pose
    if type(color) != str:
        color = f"C{color}"

    # Define the side faces of the cylinder
    sides = 32
    theta = np.linspace(0, 2*np.pi, sides)
    x_side = r * np.cos(theta)
    y_side = r * np.sin(theta)
    z_side = np.linspace(0, h, sides)
    side_vertices = np.vstack([x_side, y_side, z_side]).T

    # Define the top and bottom faces of the cylinder
    top_vertices = side_vertices.copy()
    bottom_vertices = side_vertices.copy()
    top_vertices[:,2] = h
    bottom_vertices[:,2] = 0

    # Define the rotation matrix based on the rx, ry, rz angles
    rx = np.radians(rx)
    ry = np.radians(ry)
    rz = np.radians(rz)
    rotation_matrix = np.array([
        [np.cos(ry)*np.cos(rz), -np.cos(ry)*np.sin(rz), np.sin(ry)],
        [np.sin(rx)*np.sin(ry)*np.cos(rz) + np.cos(rx)*np.sin(rz), -np.sin(rx)*np.sin(ry)*np.sin(rz) + np.cos(rx)*np.cos(rz), -np.sin(rx)*np.cos(ry)],
        [-np.cos(rx)*np.sin(ry)*np.cos(rz) + np.sin(rx)*np.sin(rz), np.cos(rx)*np.sin(ry)*np.sin(rz) + np.sin(rx)*np.cos(rz), np.cos(rx)*np.cos(ry)]
    ])

    # Rotate the vertices and translate to the desired position
    side_vertices = np.dot(side_vertices, rotation_matrix) + np.array([cx, cy, cz])
    top_vertices = np.dot(top_vertices, rotation_matrix) + np.array([cx, cy, cz])
    bottom_vertices = np.dot(bottom_vertices, rotation_matrix) + np.array([cx, cy, cz])

    # Define the faces of the cylinder
    faces = []
    for i in range(sides):
        j = (i+1) % sides
        side_face = [side_vertices[i], side_vertices[j], top_vertices[j], top_vertices[i]]
        faces.append(side_face)
        side_face = [side_vertices[i], side_vertices[j], bottom_vertices[j], bottom_vertices[i]]
        faces.append(side_face)
    
    faces_tb = []
    top_face = top_vertices[::-1]
    bottom_face = bottom_vertices
    faces_tb.append(top_face)
    faces_tb.append(bottom_face)

    # Create a Poly3DCollection with a solid face color for side faces
    collection = Poly3DCollection(faces, alpha=alpha, facecolor=color)
    cylinside = ax.add_collection3d(collection)

    # Add the top and bottom faces with wireframe
    collection = Poly3DCollection(faces_tb, alpha=alpha, facecolor=color, edgecolor='black')
    collection.set_linewidth(0.5)
    cylinder = ax.add_collection3d(collection)

    return cylinder, cylinside


# Example
if __name__ == '__main__':

    #### Create the figure object ####

    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    # Set the limits of the plot
    xlim = np.array([-250, 250])
    ylim = np.array([-50, 50])
    zlim = np.array([0, 100])

    ax.set_xlim(xlim)
    ax.set_ylim(ylim)
    ax.set_zlim(zlim)

    # Create a 2x3 array from the limits
    lims = np.array([xlim, ylim, zlim])
    min_value = np.min(lims)
    max_value = np.max(lims)
    norm_lims = (lims - min_value) / (max_value - min_value)

    ax.set_box_aspect(np.diff(norm_lims, axis=1).flatten())

    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')


    # plot_cuboid(ax, 300, 75, 50, [0, 0, 0, 0, 0, 0], color=0)
    # plot_cylinder(ax, 25, 25, [0, 0, 50, 0, 45, 0], color=0)

    X_stage = LinearStage(ax, dims=[400,50,30], color=0)
    X_stage.plot()

    Rx_stage = RotaryStage(ax, dims=[50,50,30], color=1)
    Rx_stage.plot()

    # for i in range(-150,150,10):
    #     print(i)
    #     X_stage.movePlatform(i)
    #     plt.pause(.1)

    for i in range(-150,150,10):
        print(i)
        X_stage.moveBase([i,0,0,0,0,0])
        Rx_stage.moveBase([i,0,30,0,0,0])
        plt.pause(.1)

    plt.show()

