import Kinematics as k
import numpy as np
import KinematicsSymbolic as ks

sym = ks.KinematicsSymbolic()
sym.calculateWristPositionVector()
np.set_printoptions(suppress=True)

robot = k.Kinematics()

m_i = np.array([[0,1,0,0],
                [-1,0,0, -32.5],
                [0,0,1,727.5],
                [0,0,0,1]])


q = robot.nedInverseKinematics(m_i)

q_f = [q[0][1],q[1][0],q[2][0],q[3][1],q[4][0],q[5][1]]
# q_f = [0,0,np.pi/2,0,0,0]
print(q_f)
pos = robot.nedForwardKinematrics(q_f)
print("Robot joint angles: ")
print(q)
print("\n")
print("Robot end effector pos: ")
print(pos)