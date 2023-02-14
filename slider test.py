import matplotlib.pyplot as plt
from matplotlib.widgets import Slider

fig, ax = plt.subplots()
plt.subplots_adjust(left=0.25, bottom=0.25)

dot, = plt.plot(0, 0, 'ro')

ax.set_xlim([-10, 10])
ax.set_ylim([-10, 10])

ax_x = plt.axes([0.25, 0.1, 0.65, 0.03])
ax_y = plt.axes([0.25, 0.15, 0.65, 0.03])

slider_x = Slider(ax_x, 'X', -10, 10, valinit=0)
slider_y = Slider(ax_y, 'Y', -10, 10, valinit=0)

def update(val):
    x = slider_x.val
    y = slider_y.val
    dot.set_data([x], [y])
    fig.canvas.draw_idle()

slider_x.on_changed(update)
slider_y.on_changed(update)

plt.show()
