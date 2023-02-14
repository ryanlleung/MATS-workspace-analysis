import matplotlib.pyplot as plt
from matplotlib.widgets import Slider
from mpl_toolkits import mplot3d

fig = plt.figure()
ax = plt.axes(projection='3d')

x, y, z = [0], [0], [0]
dot = ax.scatter(x, y, z, color='red')

ax.set_xlim([-10, 10])
ax.set_ylim([-10, 10])
ax.set_zlim([-10, 10])

ax_x = plt.axes([0.25, 0.1, 0.65, 0.03])
ax_y = plt.axes([0.25, 0.15, 0.65, 0.03])
ax_z = plt.axes([0.25, 0.2, 0.65, 0.03])

slider_x = Slider(ax_x, 'X', -10, 10, valinit=0)
slider_y = Slider(ax_y, 'Y', -10, 10, valinit=0)
slider_z = Slider(ax_z, 'Z', -10, 10, valinit=0)

def update(val):
    x[0] = slider_x.val
    y[0] = slider_y.val
    z[0] = slider_z.val
    dot._offsets3d = (x, y, z)
    fig.canvas.draw_idle()

slider_x.on_changed(update)
slider_y.on_changed(update)
slider_z.on_changed(update)

plt.show()
