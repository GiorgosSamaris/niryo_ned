import Kinematics as ik
import numpy as np
import csvExport as csv



class CalculateTrajectory:
    def __init__(self):
        self.ned = ik.Kinematics()        
        np.set_printoptions(suppress=True)
        self.data = csv.CSVWriter("C:/Users/Γιώργος Σάμαρης/Documents/niryo_ned/controllers/test/solutions.csv",["θ1","θ2","θ3","θ4","θ5","θ6"])

    def calculateMidJointVector(self, init_angles, fin_angles):
        q_f = fin_angles
        q_i = init_angles


        q_m = q_f - (q_f -q_i)/2

        return q_m
    

    def calculateMidPositionVector(self, init_pos, fin_pos):
        pass

    def taylorLinearInterpolationAlgorithm(self,init_pos, fin_pos, delta_p_max,recursion_lvl = 0,write_csv = False):
        #step 1
        q_i_arr = self.ned.nedInverseKinematics(init_pos)
        q_f_arr = self.ned.nedInverseKinematics(fin_pos)
        # print("[q_i]: ")
        # print(q_i)
        # print("[q_f]:")
        # print(q_f)
        # print("\n")
        q_i = np.array([q_i_arr[0][0],q_i_arr[1][1],q_i_arr[2][1],q_i_arr[3][1],q_i_arr[4][1],q_i_arr[5][1]])
        q_f = np.array([q_f_arr[0][0],q_f_arr[1][1],q_f_arr[2][1],q_f_arr[3][1],q_f_arr[4][1],q_f_arr[5][1]])
        if(recursion_lvl == 0):
            self.data.add_row(q_i)
        # print(q_i)
        #step 2 

        #calculate the middle of all joint anlges
        q_m = self.calculateMidJointVector(q_i,q_f)
        # print("[q_m]:")
        # print(q_m)
        # print("\n")


        htm_m = self.ned.nedForwardKinematics(q_m)

        # print(htm_m)

        p_i = init_pos[:3,3]
        p_f = fin_pos[:3,3]
        # print("[p_i]: ")
        # print(p_i)
        # print("\n")
        
        # print("[p_f]: ")
        # print(p_f)
        # print("\n")
        
        #calculate the middle between the initial and the final point
        p_x =  (p_i+p_f)/2

        print("[p_x]: ")
        print(p_x)
        print("\n")

        #get initial and final rotation
        r_i = init_pos[:3,:3]
        r_f = fin_pos[:3,:3]

        # rotation matrix
        rot = np.matmul(r_i.T,r_f) 

        # Find eigenvalues and eigenvectors
        eigenvalues, eigenvectors = np.linalg.eig(rot)
        # print(eigenvectors)

        # Find the eigenvector corresponding to the eigenvalue of 1
        axis_of_rotation = eigenvectors[:, np.isclose(eigenvalues, 1)]

        # print("Axis of rotation:", axis_of_rotation)
        
        #step 3
        p_m = htm_m[:3,3]
        print("[p_m]: ")
        print(p_m)
        print("\n")

        delta_p = np.abs(p_m - p_x)

        # print(delta_p)
        htm_x = np.array([[0,0,0,0],
                            [0,0,0,0],
                            [0,0,0,0],
                            [0,0,0,1]])
        htm_x[:3,:3] = r_i
        htm_x[:3,3] = p_x
        if((delta_p < delta_p_max).all()):
            
            q_m_arr= self.ned.nedInverseKinematics(htm_x)
            q_p = np.array([q_m_arr[0][0],q_m_arr[1][1],q_m_arr[2][1],q_m_arr[3][1],q_m_arr[4][1],q_m_arr[5][1]])
            self.data.add_row(q_p)
            return
        else:
            self.taylorLinearInterpolationAlgorithm(init_pos,htm_x,delta_p_max,recursion_lvl+1)
            self.taylorLinearInterpolationAlgorithm(htm_x, fin_pos,delta_p_max,recursion_lvl+1)
        if(recursion_lvl == 0):
            self.data.add_row(q_f)
            if(write_csv):
                self.data.write_csv()
                self.data.clear_data()
    
        



t = CalculateTrajectory()


m_first = np.array([[0,1,0,150],
                [1,0,0,210],
                [0,0,-1,100],
                [0,0,0,1]])

m_second = np.array([[0,1,0,-150 ],
                [1,0,0,210],
                [0,0,-1,100],
                [0,0,0,1]])

t.taylorLinearInterpolationAlgorithm(m_first,m_second,[0.01,0.01,0.01])

m_third = np.array([[0,1,0,-150],
                [1,0,0,210],
                [0,0,-1,300],
                [0,0,0,1]])

t.taylorLinearInterpolationAlgorithm(m_second,m_third,[0.01,0.01,0.01])

m_fourth = np.array([[0,1,0,150],
                [1,0,0,210],
                [0,0,-1,300],
                [0,0,0,1]])

t.taylorLinearInterpolationAlgorithm(m_third,m_fourth,[0.01,0.01,0.01])

m_first = np.array([[0,1,0,150],
                [1,0,0,210],
                [0,0,-1,100],
                [0,0,0,1]])

t.taylorLinearInterpolationAlgorithm(m_fourth,m_first,[0.01,0.01,0.01],write_csv=True)