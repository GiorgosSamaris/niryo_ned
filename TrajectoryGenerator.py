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

    #Calculates the middle of the joint angles
    def __calculateMidJointVector(self, init_angles, fin_angles):
        q_f = fin_angles
        q_i = init_angles

        q_m = q_f - (q_f -q_i)/2

        return q_m
    

    #Generates a 3d circle based on 3 points
    def __generate3dCircle(self,center, radius, point_a, point_b):
        # Creation of Plane and local coordinate system
        a = point_a - center
        b = point_b - center
        plane_vec = np.cross(a, b)
        unity_plane_vec = plane_vec / np.linalg.norm(plane_vec)
        plane_x_axis = a / np.linalg.norm(a)
        plane_y_axis = np.cross(unity_plane_vec, plane_x_axis)

        #create a circle datatype and return it
        circ = Circle3D(radius,center,plane_x_axis,plane_y_axis)
        return circ


    #find the vector that has half the angle of two other vectors
    def __middleVector(self, vector_a,vector_b):
        norm_a = np.linalg.norm(vector_a)
        norm_b = np.linalg.norm(vector_b)
        theta = np.arccos((vector_a*vector_b)/(norm_a*norm_b))
        
        new_x = x * np.cos(theta/2) - y * np.sin(theta/2)
        new_y = x * np.sin(theta/2) + y * np.cos(theta/2)

        vector_c = np.array([new_x,new_y])
        return vector_c
    


    def taylorLinearInterpolationAlgorithm(self,circle ,init_pos, fin_pos, delta_p_max,recursion_lvl = 0, write_csv = False):
        #step 1
        q_i_arr = self.ned.nedInverseKinematics(init_pos)
        q_f_arr = self.ned.nedInverseKinematics(fin_pos)
       
        q_i = np.array([q_i_arr[0][0],q_i_arr[1][1],q_i_arr[2][1],q_i_arr[3][1],q_i_arr[4][1],q_i_arr[5][1]])
        q_f = np.array([q_f_arr[0][0],q_f_arr[1][1],q_f_arr[2][1],q_f_arr[3][1],q_f_arr[4][1],q_f_arr[5][1]])
        if(recursion_lvl == 0):
            self.data.add_row(q_i)
  
        #step 2 

        #calculate the middle of all joint anlges
        q_m = self.__calculateMidJointVector(q_i,q_f)
       

        #find the htm of the q_m joint vector
        htm_m = self.ned.nedForwardKinematics(q_m)

   
        #get position vector of init and fin pos
        p_i = init_pos[:3,3]
        p_f = fin_pos[:3,3]
        
        
        #calculate the middle between the initial and the final point
        p_x =  (p_i+p_f)/2



        #get initial and final rotation
        # r_i = init_pos[:3,:3]
        # r_f = fin_pos[:3,:3]

        # # rotation matrix
        # rot = np.matmul(r_i.T,r_f) 

        # # Find eigenvalues and eigenvectors
        # eigenvalues, eigenvectors = np.linalg.eig(rot)
    

        # # Find the eigenvector corresponding to the eigenvalue of 1
        # axis_of_rotation = eigenvectors[:, np.isclose(eigenvalues, 1)]

    
        
        #step 3
        p_m = htm_m[:3,3]
        
        #calculate distance of actual middle from theoretical middle
        delta_p = np.abs(p_m - p_x)

        #init an a zero htm
        htm_x = np.array([[0,0,0,0],
                            [0,0,0,0],
                            [0,0,0,0],
                            [0,0,0,1]])
        # htm_x[:3,:3] = r_i
        htm_x[:3,3] = p_x

        #check if delta_p is smaller than the maximum value
        if((delta_p < delta_p_max).all()):
            
            q_m_arr= self.ned.nedInverseKinematics(htm_x)
            q_p = np.array([q_m_arr[0][0],q_m_arr[1][1],q_m_arr[2][1],q_m_arr[3][1],q_m_arr[4][1],q_m_arr[5][1]])
            self.data.add_row(q_p)
            return
        else:#if not start two ricursions for each half
            self.taylorLinearInterpolationAlgorithm(init_pos,htm_x,delta_p_max,recursion_lvl+1)
            self.taylorLinearInterpolationAlgorithm(htm_x, fin_pos,delta_p_max,recursion_lvl+1)

        #check if we are on the first level of recursion so as to write the csv file 
        if(recursion_lvl == 0):
            self.data.add_row(q_f)
            if(write_csv):
                self.data.write_csv()
                self.data.clear_data()







    def taylorCirclularInterpolationAlgorithm(self,init_pos, fin_pos, delta_p_max,recursion_lvl = 0):
        #step 1
        q_i_arr = self.ned.nedInverseKinematics(init_pos)
        q_f_arr = self.ned.nedInverseKinematics(fin_pos)
       
        q_i = np.array([q_i_arr[0][0],q_i_arr[1][1],q_i_arr[2][1],q_i_arr[3][1],q_i_arr[4][1],q_i_arr[5][1]])
        q_f = np.array([q_f_arr[0][0],q_f_arr[1][1],q_f_arr[2][1],q_f_arr[3][1],q_f_arr[4][1],q_f_arr[5][1]])
        if(recursion_lvl == 0):
            self.data.add_row(q_i)
  
        #step 2 

        #calculate the middle of all joint anlges
        q_m = self.__calculateMidJointVector(q_i,q_f)
       


        htm_m = self.ned.nedForwardKinematics(q_m)

   

        p_i = init_pos[:3,3]
        p_f = fin_pos[:3,3]
        
        
        #calculate the middle between the initial and the final point
        p_x =  (p_i+p_f)/2

        print("[p_x]: ")
        print(p_x)
        print("\n")

        #get initial and final rotation
        # r_i = init_pos[:3,:3]
        # r_f = fin_pos[:3,:3]

        # # rotation matrix
        # rot = np.matmul(r_i.T,r_f) 

        # # Find eigenvalues and eigenvectors
        # eigenvalues, eigenvectors = np.linalg.eig(rot)
    

        # # Find the eigenvector corresponding to the eigenvalue of 1
        # axis_of_rotation = eigenvectors[:, np.isclose(eigenvalues, 1)]

    
        
        #step 3
        p_m = htm_m[:3,3]
        

        delta_p = np.abs(p_m - p_x)


        htm_x = np.array([[0,0,0,0],
                            [0,0,0,0],
                            [0,0,0,0],
                            [0,0,0,1]])
        # htm_x[:3,:3] = r_i
        htm_x[:3,3] = p_x

        #check if delta_p is smaller than the maximum value
        if((delta_p < delta_p_max).all()):
            
            q_m_arr= self.ned.nedInverseKinematics(htm_x)
            q_p = np.array([q_m_arr[0][0],q_m_arr[1][1],q_m_arr[2][1],q_m_arr[3][1],q_m_arr[4][1],q_m_arr[5][1]])
            self.data.add_row(q_p)
            return
        else:#if not start two ricursions for each half
            self.taylorLinearInterpolationAlgorithm(init_pos,htm_x,delta_p_max,recursion_lvl+1)
            self.taylorLinearInterpolationAlgorithm(htm_x, fin_pos,delta_p_max,recursion_lvl+1)

        #check if we are on the first level of recursion so as to write the csv file 
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