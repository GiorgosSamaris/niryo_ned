import numpy as np
import matplotlib.pyplot as plt
import TrajectoryGenerator as t






# Inputs
# R_circle = float(input("R: "))
# PC = np.array([float(input("x center: ")), float(input("y center: ")), float(input("z center: "))])
# P1 = np.array([float(input("x P1: ")), float(input("y P1: ")), float(input("z P1: "))])
# P2 = np.array([float(input("x P2: ")), float(input("y P2: ")), float(input("z P2: "))])

R_circle = 40
PC = np.array([0,200,200])
P1 = np.array([0,240,200])
P2 = np.array([40,200,200])

# Creation of Plane and local coordinate system
a = P1 - PC
b = P2 - PC
plane_vec = np.cross(a, b)
unity_plane_vec = plane_vec / np.linalg.norm(plane_vec)
local_x_axis = a / np.linalg.norm(a)
local_y_axis = np.cross(unity_plane_vec, local_x_axis)

# Creation of Circle in Plane 
n_poly = 2
dis = 0.01
s = 1e6
while(abs(R_circle - s) > dis):
    n_poly += 1
    theta_turn = 2 * np.pi / n_poly
    local_points = np.zeros((n_poly, 3))
    for i in range(n_poly):
        local_points[i, 0] = R_circle * np.cos(i * theta_turn)
        local_points[i, 1] = R_circle * np.sin(i * theta_turn)
    mid_point = 0.5 * (local_points[0, :2] + local_points[1, :2])
    s = np.sqrt(mid_point[0]**2 + mid_point[1]**2)

# Transformation to Local Coordinate system Coordinates
local_trans = np.zeros((n_poly, 3))
for i in range(n_poly):
    local_trans[i, 0] = np.dot(local_points[i, :], local_x_axis)
    local_trans[i, 1] = np.dot(local_points[i, :], local_y_axis)
    local_trans[i, 2] = np.dot(local_points[i, :], unity_plane_vec)

# Transformation to General Coordinate system
Trans_vectors = np.tile(PC, (n_poly, 1))
global_points = Trans_vectors + local_trans
wrt = False
htm = np.array([[0,1,0,0],
                [1,0,0,0],
                [0,0,-1,0],
                [0,0,0,1]])

test = t.CalculateTrajectory()

for i_index,init_point in enumerate(global_points):
    # print("init_point= ",init_point)
    htm_i = htm.copy()
    htm_i[:3,3] = init_point


    f_index = i_index+1


    if(f_index<global_points.shape[0]):
        final_point = global_points[f_index]
        htm_f = htm.copy()
        htm_f[:3,3] = final_point
        # print("final_point= ", final_point)
    else:
        htm_f = htm_i.copy()
    # print("htm_i =\n",htm_i)
    # print("htm_f =\n",htm_f)
    if(i_index==0):
        test.taylorLinearInterpolationAlgorithm(htm_i,htm_f,[0.01,0.01,0.01],csv_mode='w')
    else:
        test.taylorLinearInterpolationAlgorithm(htm_i,htm_f,[0.01,0.01,0.01],csv_mode='a')



