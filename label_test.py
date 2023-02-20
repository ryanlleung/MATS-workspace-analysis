
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.widgets import Slider, TextBox

# Define the function to update the dot position
def update_dot(x):
    dot._offsets3d = ([x], [0], [0])
    x_textbox.set_val(x)
    return dot,

# Create the figure and 3D axes
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# Set the axis limits
ax.set_xlim(-1, 1)
ax.set_ylim(-1, 1)
ax.set_zlim(-1, 1)

# Create the dot
x_init = 0
y_init = 0
z_init = 0
dot = ax.scatter([x_init], [y_init], [z_init], marker='o', s=100)

# Create the slider
axslider = plt.axes([0.2, 0.1, 0.6, 0.03])
slider = Slider(axslider, 'X position', -1, 1, valinit=x_init)
slider.on_changed(update_dot)

# Create the textbox
axtextbox = plt.axes([0.2, 0.05, 0.2, 0.03])
x_textbox = TextBox(axtextbox, 'X value', initial=str(x_init))
x_textbox.on_submit(lambda x: update_dot(float(x)))

# Show the plot
plt.show()


