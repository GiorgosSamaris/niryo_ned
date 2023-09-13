import Kinematics as ik
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

    def taylorLinearInterpolationAlgorithm(self,init_pos, fin_pos, delta_p_max):
        #step 1
        q_i = self.ned.nedInverseKinematics(init_pos)
        q_f = self.ned.nedInverseKinematics(fin_pos)
        print("[q_i]: ")
        print(q_i)
        print("[q_f]:")
        print(q_f)
        print("\n")

        #step 2

        #calculate the middle of all joint anlges
        q_m = self.calculateMidJointVector(q_i,q_f)
        print("[q_m]:")
        print(q_m)
        print("\n")


        htm_m = self.ned.calculateForwardKinematics(q_m[:,1])

        print(htm_m)

        p_i = init_pos[:3,3]
        p_f = fin_pos[:3,3]
        print("[p_i]: ")
        print(p_i)
        print("\n")
        
        print("[p_f]: ")
        print(p_f)
        print("\n")
        
        #calculate the middle between the initial and the final point
        p_x =  np.add(p_i,p_f)/2

        print("[p_x]: ")
        print(p_x)
        print("\n")

        #get initial and final rotation
        r_i = init_pos[:3,:3]
        r_f = fin_pos[:3,:3]

        #rotation matrix
        #rot = np.matmul(r_i.T,r_f) 

        # # Find eigenvalues and eigenvectors
        # eigenvalues, eigenvectors = np.linalg.eig(rot)
        # print(eigenvectors)

        # # Find the eigenvector corresponding to the eigenvalue of 1
        # axis_of_rotation = eigenvectors[:, np.isclose(eigenvalues, 1)]

        # print("Axis of rotation:", axis_of_rotation)
        
        #step 3
        p_m = htm_m[:3,3]
        print("[p_m]: ")
        print(p_m)
        print("\n")

        delta_p = np.abs(p_m - p_x)

        print(delta_p)

        # if((delta_p < delta_p_max).all()):
        #     return
        # else:
        #     self.taylorLinearInterpolationAlgorithm(init_pos,htm_m,delta_p_max)
        #     self.taylorLinearInterpolationAlgorithm(htm_m, fin_pos,delta_p_max)

        



t = CalculateTrajectory()


m_i = np.array([[1,0,0,-320],
                [0,1,0,210],
                [0,0,1,280],
                [0,0,0,1]])


m_f = np.array([[1,0,0,320 ],
                [0,1,0,210],
                [0,0,1,100],
                [0,0,0,1]])

t.taylorLinearInterpolationAlgorithm(m_i,m_f,[2,2,2])