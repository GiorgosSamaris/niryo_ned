import Kinematics as k
import numpy as np
import KinematicsSymbolic as ks
import csvExport as csv
data = csv.CSVWriter("~/Documents/ned/controllers/test/solutions.csv",["θ1","θ2","θ3","θ4","θ5","θ6"])
sym = ks.KinematicsSymbolic()
sym.calculateWristPositionVector()
np.set_printoptions(suppress=True)

robot = k.Kinematics()



# m_i = np.array([[0,1,0,320],
#                 [1,0,0,210],
#                 [0,0,-1,100],
#                 [0,0,0,1]])
# q = robot.nedInverseKinematics(m_i)



# print("Robot joint angles: ")
# print(q)
# print("\n")
# q_f = [q[0][0],q[1][1],q[2][1],q[3][1],q[4][1],q[5][1]]
# # m_i = np.array([[0,1,0,-320],
# #                 [-1,0,0,210],
# #                 [0,0,1,280],
# #                 [0,0,0,1]])
# # q = robot.nedInverseKinematics(m_i)
# # q_f = [q[0][0],q[1][1],q[2][1],q[3][1],q[4][1],q[5][1]]
# # data.add_row(q_f)

# # q_f = [0,0,0,0,-np.pi/2,0]

# data.add_row(q_f)
# data.write_csv()
# data.clear_data()
# print(q_f)
# pos = robot.nedForwardKinematics(q_f)
# print("Robot end effector pos: ")
# print(pos)