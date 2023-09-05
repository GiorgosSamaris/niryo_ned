import InverseKinematics as ik
import numpy as np



class CalculateTrajectory:
    def __init__(self):
        self.ned = ik.InverseKinematics()        


    def calculateMidJointVector(self, init_angles, fin_angles):
        q_f = fin_angles
        q_i = init_angles


        q_m = q_f - (q_f -q_i)/2

        return q_m
    

    def calculateMidPositionVector(self, init_pos, fin_pos):
        pass

    def taylorLinearInterpolationAlgorithm(self,init_pos, fin_pos):
        q_i = self.ned.nedInverseKinematics(init_pos)
        q_f = self.ned.nedInverseKinematics(fin_pos)

        q_m = self.calculateMidJointVector(q_i,q_f)

        



t = CalculateTrajectory()


m_i = np.array([[1,0,0,-320],
                [0,1,0,210],
                [0,0,1,280],
                [0,0,0,1]])


m_f = np.array([[1,0,0,320 ],
                [0,1,0,210],
                [0,0,1,100],
                [0,0,0,1]])

t.taylorLinearInterpolationAlgorithm(m_i,m_f)