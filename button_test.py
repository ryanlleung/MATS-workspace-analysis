
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D,axes3d
from matplotlib.widgets import Button

# Create a 3D figure and axis
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# Generate some data to plot
X, Y, Z = axes3d.get_test_data(0.05)

# Plot the data as a surface
ax.plot_surface(X, Y, Z, cmap='plasma')

# Define the function to reset the view
def reset_view(event):
    ax.view_init(elev=30, azim=-60)

# Create the "Reset View" button and connect it to the reset_view function
reset_button_ax = fig.add_axes([0.85, 0.05, 0.1, 0.075])
reset_button = Button(reset_button_ax, 'Reset View')
reset_button.on_clicked(reset_view)

# Show the plot
plt.show()
