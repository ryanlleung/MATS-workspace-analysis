import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

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

plot_cuboid(ax, 300, 75, 50, [0, 0, 0, 0, 0, 0], color=0)
plot_cylinder(ax, 25, 25, [0, 0, 50, 0, 45, 0], color=0)

plt.show()
