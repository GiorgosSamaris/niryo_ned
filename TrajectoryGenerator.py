import Kinematics as ik
import numpy as np
import csvExport as csv
from dataclasses import dataclass

@dataclass

class Circle3D:
    R: float
    xyz_c : np.ndarray
    plane_x_axis: np.ndarray
    plane_y_axis: np.ndarray

    

class CalculateTrajectory:

    #Constructor 
    def __init__(self):
        self.ned = ik.Kinematics()        
        np.set_printoptions(suppress=True)
        self.data = csv.CSVWriter("C:/Users/Γιώργος Σάμαρης/Documents/niryo_ned/controllers/test/solutions.csv",["θ1","θ2","θ3","θ4","θ5","θ6"])
        self.data_v = csv.CSVWriter("C:/Users/Γιώργος Σάμαρης/Documents/niryo_ned/controllers/test/solutions_v.csv",["dθ1","dθ2","dθ3"])

    #Calculates the middle of the joint angles
    def __calculateMidJointVector(self, init_angles, fin_angles):
        q_f = fin_angles
        q_i = init_angles

        q_m = q_f - (q_f -q_i)/2

        return q_m
    

    
    


    def taylorLinearInterpolationAlgorithm(self,target_velocity,init_pos, fin_pos, delta_p_max,recursion_lvl = 0,csv_mode = 'a'):
        #step 1
        q_i_arr = self.ned.nedInverseKinematics(init_pos)
        q_f_arr = self.ned.nedInverseKinematics(fin_pos)
       
        q_i = np.array([q_i_arr[0][0],q_i_arr[1][1],q_i_arr[2][1],q_i_arr[3][1],q_i_arr[4][1],q_i_arr[5][1]])
        q_f = np.array([q_f_arr[0][0],q_f_arr[1][1],q_f_arr[2][1],q_f_arr[3][1],q_f_arr[4][1],q_f_arr[5][1]])
        if(recursion_lvl == 0):
            self.data.add_row(q_i)
            dq = self.ned.nedEndEffectorVelocity(target_velocity, q_i)
            self.data_v.add_row(dq)

        #step 2 

        #calculate the middle of all joint anlges
        q_m = self.__calculateMidJointVector(q_i,q_f)
        

        htm_m = self.ned.nedForwardKinematics(q_m)

        p_i = init_pos[:3,3]
        p_f = fin_pos[:3,3]
       
        
        #calculate the middle between the initial and the final point
        p_x =  (p_i+p_f)/2

     

        #get initial and final rotation
        r_i = init_pos[:3,:3]
        # r_f = fin_pos[:3,:3]

        # # rotation matrix
        # rot = np.matmul(r_i.T,r_f) 

        # # Find eigenvalues and eigenvectors
        # eigenvalues, eigenvectors = np.linalg.eig(rot)
        # # print(eigenvectors)

        # # Find the eigenvector corresponding to the eigenvalue of 1
        # axis_of_rotation = eigenvectors[:, np.isclose(eigenvalues, 1)]

        # # print("Axis of rotation:", axis_of_rotation)
        
        #step 3
        p_m = htm_m[:3,3]
        # print("[p_m]: ")
        # print(p_m)
        # print("\n")

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
            # print("[q_p]:",q_p)
            self.data.add_row(q_p)
            dq_1 = self.ned.nedEndEffectorVelocity(target_velocity, q_p)
            print(dq_1)
            self.data_v.add_row(dq_1)
            # if(recursion_lvl == 1):
            #     self.data.add_row(q_f)     
            #     dq_2 = self.ned.nedEndEffectorVelocity(target_velocity, q_f)
            #     self.data_v.add_row(dq_2)        
            #     self.data.write_csv(csv_mode)
            #     self.data_v.write_csv(csv_mode)
            return
        else:
            self.taylorLinearInterpolationAlgorithm(target_velocity,init_pos,htm_x,delta_p_max,recursion_lvl+1)
            self.taylorLinearInterpolationAlgorithm(target_velocity,htm_x, fin_pos,delta_p_max,recursion_lvl+1)
        if(recursion_lvl == 0):
                self.data.add_row(q_f)     
                dq_2 = self.ned.nedEndEffectorVelocity(target_velocity, q_f)
                self.data_v.add_row(dq_2)        
                self.data.write_csv(csv_mode)
                self.data_v.write_csv(csv_mode)


    def circularInterpolationAlgorithm(self):
        R_circle = 100
        PC = np.array([0,300,200])
        P1 = np.array([0,300,400])
        P2 = np.array([40,300,400])

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
                print("created csv")
                self.taylorLinearInterpolationAlgorithm(htm_i,htm_f,[0.01,0.01,0.01],csv_mode='w')
            else:
                self.taylorLinearInterpolationAlgorithm(htm_i,htm_f,[0.01,0.01,0.01],csv_mode='a')


        





t = CalculateTrajectory()
# t.circularInterpolationAlgorithm()
target_v = np.array([50,50,50])
m_first = np.array([[0,1,0,150],
                [1,0,0,210],
                [0,0,-1,100],
                [0,0,0,1]])

m_second = np.array([[0,1,0,-150],
                [1,0,0,210],
                [0,0,-1,100],
                [0,0,0,1]])

t.taylorLinearInterpolationAlgorithm(target_v,m_first,m_second,[0.01,0.01,0.01],csv_mode='w')

m_third = np.array([[0,1,0,-150],
                [1,0,0,210],
                [0,0,-1,300],
                [0,0,0,1]])

t.taylorLinearInterpolationAlgorithm(target_v,m_second,m_third,[0.01,0.01,0.01])

m_fourth = np.array([[0,1,0,150],
                [1,0,0,210],
                [0,0,-1,300],
                [0,0,0,1]])

t.taylorLinearInterpolationAlgorithm(target_v,m_third,m_fourth,[0.01,0.01,0.01])

m_first = np.array([[0,1,0,150],
                [1,0,0,210],
                [0,0,-1,100],
                [0,0,0,1]])

t.taylorLinearInterpolationAlgorithm(target_v,m_fourth,m_first,[0.01,0.01,0.01])