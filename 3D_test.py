import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

## Geometries are Z-up ##

# Class to plot a 3D linear stage
class LinearStage:
    
    # Initialize the stage
    def __init__(self, ax, dims=[400,50,50], pose=[0,0,0,0,0,0], S=0, color=0, alpha=0.25):
        self.ax = ax
        self.l, self.w, self.h = dims
        self.cx, self.cy, self.cz, self.rx, self.ry, self.rz = pose
        self.base_pose = pose
        self.S = S
        self.color = color
        self.alpha = alpha
        self.thickness = 15

        self.rx = np.deg2rad(self.rx)
        self.ry = np.deg2rad(self.ry)
        self.rz = np.deg2rad(self.rz)

    # Function to plot the stage
    def plot(self):

        # Plot the base      
        self.base_height = self.h - self.thickness  
        plot_cuboid(self.ax, self.l, self.w, self.base_height, self.base_pose, self.color, self.alpha)

        # Transform the base pose to the platform pose
        Tbp = np.array([
            [1,0,0,self.S],
            [0,1,0,0],
            [0,0,1,self.base_height],
            [0,0,0,1]
        ])
        
        # Define base pose in matrix form
        print(self.rx, self.ry, self.rz)
        T0b = np.array([[np.cos(self.ry)*np.cos(self.rz), -np.cos(self.ry)*np.sin(self.rz), np.sin(self.ry), self.cx],
                        [np.cos(self.rx)*np.sin(self.rz) + np.cos(self.rz)*np.sin(self.rx)*np.sin(self.ry), np.cos(self.rx)*np.cos(self.rz) - np.sin(self.rx)*np.sin(self.ry)*np.sin(self.rz), -np.cos(self.ry)*np.sin(self.rx), self.cy],
                        [np.sin(self.rx)*np.sin(self.rz) - np.cos(self.rx)*np.cos(self.rz)*np.sin(self.ry), np.cos(self.rz)*np.sin(self.rx) + np.cos(self.rx)*np.sin(self.ry)*np.sin(self.rz), np.cos(self.rx)*np.cos(self.ry), self.cz],
                        [0,0,0,1]])
        
        # Calculate the platform pose
        T0p = np.dot(T0b, Tbp)

        # Convert the platform pose to a list
        self.rx = np.rad2deg(np.arctan2(T0p[2,1], T0p[2,2]))
        self.ry = np.rad2deg(np.arctan2(-T0p[2,0], np.sqrt(T0p[2,1]**2 + T0p[2,2]**2)))
        self.rz = np.rad2deg(np.arctan2(T0p[1,0], T0p[0,0]))
        self.plat_pose = [T0p[0,3], T0p[1,3], T0p[2,3], self.rx, self.ry, self.rz]
        print(self.plat_pose)
        print('-----------------')

        # Plot the platform
        plat_l = plat_w = self.w - 5
        plot_cuboid(self.ax, plat_l, plat_w, self.thickness, self.plat_pose, self.color, self.alpha)




# Function to plot a cuboid with origin at centroid of bottom face
def plot_cuboid(ax, l, w, h, pose=[0,0,0,0,0,0], color=0, alpha=0.25):
    
    # Process the arguments
    cx, cy, cz, rx, ry, rz = pose
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
    collection = Poly3DCollection(faces, alpha=alpha, facecolor=color, edgecolor=color)
    collection.set_linewidth(2)
    ax.add_collection3d(collection)

    return


# Function to plot a cylinder with origin at centroid of bottom face
def plot_cylinder(ax, r, h, pose=[0,0,0,0,0,0], color=0, alpha=0.25):
    
    # Process the arguments
    cx, cy, cz, rx, ry, rz = pose
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
    ax.add_collection3d(collection)

    # Add the top and bottom faces with wireframe
    collection = Poly3DCollection(faces_tb, alpha=alpha, facecolor=color, edgecolor=color)
    collection.set_linewidth(2)
    ax.add_collection3d(collection)

    return


fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

ax.set_box_aspect([1, 0.25, 0.25])

# Set the limits of the plot
ax.set_xlim([-150, 150])
ax.set_ylim([-37.5, 37.5])
ax.set_zlim([0, 75])

ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')

# plot_cuboid(ax, 300, 75, 50, [0, 0, 0, 0, 0, 0], color=0)
# plot_cylinder(ax, 25, 25, [0, 0, 50, 0, 45, 0], color=0)

# X_stage = LinearStage(ax, dims=[400,50,50], pose=[0,0,0,0,45,0], S=0, color=0)
# X_stage.plot()

for i in range(0,90,10):
    X_stage = LinearStage(ax, dims=[400,50,50], pose=[-50,20*i,0,45,i,0], S=50, color=i)
    X_stage.plot()
    # plot_cuboid(ax, 400, 50, 50, [-50,20*i,0,0,i,0], color=i//10)
plt.show()

