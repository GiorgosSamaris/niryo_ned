import InverseKinematics as ik
import numpy as np


robot = ik.InverseKinematics()

m_i = np.array([[1,0,0,-320],
                [0,1,0,210],
                [0,0,1,280],
                [0,0,0,1]])


q = robot.nedInverseKinematics(m_i)


print("Robot joint angles: ")
print(q)