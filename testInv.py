import KinematicsSymbolic as ks
import Kinematics as k
import csvExport as csv
import TrajectoryGenerator as tg
import numpy as np
import matplotlib.pyplot as plt
import sys
sys.setrecursionlimit(3000)
data = csv.CSVWriter("~/Documents/ned/controllers/test/solutions.csv",["θ1","θ2","θ3","θ4","θ5","θ6"])
sym = ks.KinematicsSymbolic()
sym.calculateWristPositionVector()
np.set_printoptions(suppress=True)
traj = tg.CalculateTrajectory()

robot = k.Kinematics()

# Inputs
xo = float(input("xo: "))
yo = float(input("yo: "))
zo = float(input("zo: "))

xf = float(input("xf: "))
yf = float(input("yf: "))
zf = float(input("zf: "))
n = 1
dis = 1
R_max = 1e6

while R_max > dis:
    n += 1
    step_x = (xf - xo) / (n - 1)
    step_y = (yf - yo) / (n - 1)
    step_z = (zf - zo) / (n - 1)
    param_matrix = np.zeros((n, 3))
    points = np.zeros((n, 3))
    mid_points = np.zeros((n - 1, 3))
    mid_curves = np.zeros((n - 1, 3))
    R = np.zeros((n - 1, 1))
    
    for i in range(n):
        # Parameters Calculation
        param_matrix[i, 0] = xo + (i - 1) * step_x
        param_matrix[i, 1] = yo + (i - 1) * step_y
        param_matrix[i, 2] = zo + (i - 1) * step_z

        # Contour Calculation
        points[i, 0] = (param_matrix[i, 0]) ** 2  # test function x(x_relative)=x_relative^2
        points[i, 1] = (param_matrix[i, 1])  # test function y(y_relative)=y_relative
        points[i, 2] = (param_matrix[i, 2])  # test function z(z_relative)=z_relative

    for i in range(n - 1):
        # Middle of line
        mid_points[i, 0] = 0.5 * (points[i, 0] + points[i + 1, 0])
        mid_points[i, 1] = 0.5 * (points[i, 1] + points[i + 1, 1])
        mid_points[i, 2] = 0.5 * (points[i, 2] + points[i + 1, 2])

        # Curve at mid_point
        mid_curves[i, 0] = (0.5 * (param_matrix[i, 0] + param_matrix[i + 1, 0])) ** 2  # mid_point of function
        mid_curves[i, 1] = (0.5 * (param_matrix[i, 1] + param_matrix[i + 1, 1]))
        mid_curves[i, 2] = (0.5 * (param_matrix[i, 2] + param_matrix[i + 1, 2]))

        R[i, 0] = np.sqrt(
            (mid_curves[i, 0] - mid_points[i, 0]) ** 2 + (mid_curves[i, 1] - mid_points[i, 1]) ** 2 + (
                        mid_curves[i, 2] - mid_points[i, 2]) ** 2)

        if i > 2:
            if R[i, 0] > R[i - 1, 0]:
                R_max = R[i, 0]
    htm = np.array([[0,1,0,0],
                    [1,0,0,0],
                    [0,0,-1,0],
                    [0,0,0,1]])



# Print the curve data (X, Y, Z) as an array
print("Curve Data (X, Y, Z):")
print(curve_data)

for i_index,init_point in enumerate(points):
            # print("init_point= ",init_point)
            htm_i = htm.copy()
            htm_i[:3,3] = init_point


            f_index = i_index+1


            if(f_index<points.shape[0]):
                final_point = points[f_index]
                htm_f = htm.copy()
                htm_f[:3,3] = final_point
                # print("final_point= ", final_point)
            else:
                htm_f = htm_i.copy()
            # print("htm_i =\n",htm_i)
            # print("htm_f =\n",htm_f)
            if(i_index==0):
                traj.taylorLinearInterpolationAlgorithm(htm_i,htm_f,[0.1,0.1,0.1])
            else:
                traj.taylorLinearInterpolationAlgorithm(htm_i,htm_f,[0.1,0.1,0.1])

traj.writeKinematicSolutions()
# Plot 3D trajectory
plt.plot(points[:,:])
plt.xlabel("X")
plt.ylabel("Y")
plt.show()
