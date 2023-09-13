import Kinematics as k
import numpy as np


robot = k.Kinematics()

m_i = np.array([[0,1,0,320],
                [1,0,0,210],
                [0,0,-1,100],
                [0,0,0,1]])


q = robot.nedInverseKinematics(m_i)

# q_f = [q[0][0],q[1][1],q[2][0],q[3][1],q[4][1],q[5][1]]
q_f = [0,0,0,0,0,0]
print(q_f)
pos = robot.nedForwardKinematrics(q_f)
print("Robot joint angles: ")
print(q)
print("\n")
print("Robot end effector pos: ")
print(pos)