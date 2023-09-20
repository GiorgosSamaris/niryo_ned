import numpy as np
import math

# Inputs
R_circle = float(input("R: "))
PC = np.array([float(input("x center: ")), float(input("y center: ")), float(input("z center: "))])
P1 = np.array([float(input("x P1: ")), float(input("y P1: ")), float(input("z P1: "))])
P2 = np.array([float(input("x P2: ")), float(input("y P2: ")), float(input("z P2: "))])

# Creation of Plane and local coordinate system
a = P1 - PC
b = P2 - PC
plane_vec = np.cross(a, b)
unity_plane_vec = plane_vec / np.linalg.norm(plane_vec)
local_x_axis = a / np.linalg.norm(a)
local_y_axis = np.cross(unity_plane_vec, local_x_axis)

# Creation of Circle in Plane
n_poly = 2
# Use of Taylor_Criterion to choose number of polygons
    n_poly += 1
    theta_turn = 2 * math.pi / n_poly
    local_points = np.zeros((n_poly, 3))
    for i in range(n_poly):
        local_points[i, 0] = R_circle * math.cos(i * theta_turn)
        local_points[i, 1] = R_circle * math.sin(i * theta_turn)
    local_trans = np.zeros((n_poly, 3))
    for i in range(n_poly):
        local_trans[i, 0] = np.dot(local_points[i, :], local_x_axis)
        local_trans[i, 1] = np.dot(local_points[i, :], local_y_axis)
        local_trans[i, 2] = np.dot(local_points[i, :], unity_plane_vec)
    Trans_vectors = np.zeros((n_poly, 3))
    for i in range(n_poly):
        Trans_vectors[i, :] = PC
    global_points = Trans_vectors + local_trans
    # Your Part of the algorithm for inverse kinematics calculation, median
    # point from points, median point for inverse kinematic p_avg(q_avg)
    n_poly_final = n_poly
The part of the code for circular movement is commented out in the MATLAB script, so it is not included in the Python script
