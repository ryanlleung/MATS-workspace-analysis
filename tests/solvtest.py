
import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import fsolve

cx = 157.29
cy = 0
r = 12.5
lx = 135
ly = -1.34
phi = 37.5
# theta = 0

phi = np.deg2rad(phi)
# theta = np.deg2rad(theta)

def equations(vars):
    x, y, lamb, theta = vars
    eq1 = (x-cx)**2 + (y-cy)**2 - r**2
    eq2 = -(lamb*np.sin(phi)+ly)*np.sin(theta) + (lamb*np.cos(phi)+lx)*np.cos(theta) - x
    eq3 = (lamb*np.sin(phi)+ly)*np.cos(theta) + (lamb*np.cos(phi)+lx)*np.sin(theta) - y
    # eq4 = x + r*np.sin(phi+theta) - cx
    eq5 = y - r*np.cos(phi+theta) - cy
    return [eq1, eq2, eq3, eq5]

x0 = [0, 0, 0, 0]
x, y, lamb, theta = fsolve(equations, x0)
print(x, y, lamb, cx, np.rad2deg(theta))


solve_list = []
cx_range = np.arange(150.53, 164.89, 0.1)
for cx_ in cx_range:
    cx = cx_
    x, y, lamb, theta = fsolve(equations, x0)
    solve_list.append([x, y, lamb, cx, np.rad2deg(theta)])

plt.plot(cx_range, [i[4] for i in solve_list])

# Linear regression
from scipy import stats
slope, intercept, r_value, p_value, std_err = stats.linregress(cx_range, [i[4] for i in solve_list])
print(slope, intercept, r_value, p_value, std_err)

# Plot the linear regression line
#plt.plot(cx_range, intercept + slope*cx_range, 'r', label='fitted line')

# Fit a polynomial
z = np.polyfit(cx_range, [i[4] for i in solve_list], 2)
p = np.poly1d(z)

# Plot the polynomial
#plt.plot(cx_range, p(cx_range), 'g', label='fitted line')

plt.grid()
plt.ylabel('theta')
plt.xlabel('cx')
plt.title('theta vs cx')
plt.show()
