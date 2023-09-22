import numpy as np
import matplotlib.pyplot as plt

# Inputs
xo = float(input("xo: "))
yo = float(input("yo: "))
zo = float(input("zo: "))

xf = float(input("xf: "))
yf = float(input("yf: "))
zf = float(input("zf: "))
n = 1

while True:  # Replace this with your Taylor Criterion condition
    n += 1
    step_x = (xf - xo) / (n - 1)
    step_y = (yf - yo) / (n - 1)
    step_z = (zf - zo) / (n - 1)
    points = np.zeros((n, 3))

    # Contour Calculation
    i = 0
    while i < n:
        points[i, 0] = (xo + i * step_x) ** 2  # test function x(x_relative)=x_relative^2
        points[i, 1] = yo + i * step_y       # test function y(y_relative)=y_relative
        points[i, 2] = zo + i * step_z       # test function z(z_relative)=z_relative
        i += 1

    # Your Part of the algorithm for inverse kinematics calculation
    # Median point from points, median point for inverse kinematic p_avg(q_avg)

    # Plot the trajectory
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.plot(points[:, 0], points[:, 1], points[:, 2])
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    plt.show()

    # Break out of the loop or add your Taylor Criterion check here
    break  # Remove this line if you have a Taylor Criterion check

# Rest of your code for inverse kinematics and other calculations
