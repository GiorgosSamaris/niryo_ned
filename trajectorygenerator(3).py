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
    

    
    


    def taylorLinearInterpolationAlgorithm(self,init_pos, fin_pos, delta_p_max,recursion_lvl = 0):
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

    
        #get initial and final rotation
        r_i = init_pos[:3,:3]
      
        #step 3
        p_m = htm_m[:3,3]
     
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
                # print(self.data.getBuffer())         

    def writeKinematicSolutions(self,csv_mode='w'):
        self.data.write_csv(csv_mode)

    def circularInterpolationAlgorithm(self, R, center, point_1, point_2):
        R_circle = R
        PC = center
        P1 = point_1
        P2 = point_2

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
                self.taylorLinearInterpolationAlgorithm(htm_i,htm_f,[0.01,0.01,0.01])
            else:
                self.taylorLinearInterpolationAlgorithm(htm_i,htm_f,[0.01,0.01,0.01])


    def curveInterpolationAlgorithm(self,init_point,fin_point)
        to = input("to")
        tf = input("tf")
n = 2
dis = 0.1
R_max = 1e6
while R_max > dis:
    n = n + 1
    step_t = (tf - to) / (n - 1)
    points = [[0, 0, 0] for _ in range(n)]
    param_matrix = [[0] for _ in range(n)]
    mid_points = [[0, 0, 0] for _ in range(n - 1)]
    mid_curves = [[0, 0, 0] for _ in range(n - 1)]
    R = [[0] for _ in range(n - 1)]
    for i in range(n):
        param_matrix[i][0] = to + (i - 1) * step_t
        points[i][0] = param_matrix[i][0]
        points[i][1] = 100
        points[i][2] = (param_matrix[i][0]) ** 2 + 100
    for i in range(n - 1):
        mid_points[i][0] = 0.5 * (points[i][0] + points[i + 1][0])
        mid_points[i][1] = 0.5 * (points[i][1] + points[i + 1][1])
        mid_points[i][2] = 0.5 * (points[i][2] + points[i + 1][2])
        mid_curves[i][0] = 0.5 * (param_matrix[i][0] + param_matrix[i + 1][0])
        mid_curves[i][1] = 100
        mid_curves[i][2] = (0.5 * (param_matrix[i][0] + param_matrix[i + 1][0])) ** 2 + 100
        R[i][0] = ((mid_curves[i][0] - mid_points[i][0]) ** 2 + (mid_curves[i][1] - mid_points[i][1]) ** 2 + (mid_curves[i][2] - mid_points[i][2]) ** 2) ** 0.5
        if i > 2:
            if R[i][0] > R[i - 1][0]:
                R_max = R[i][0]
            htm = np.array([[0,1,0,0],
                            [1,0,0,0],
                            [0,0,-1,0],
                            [0,0,0,1]])


        for i_index,init_point in enumerate(points):
                    # print("init_point= ",init_point)
                    htm_i = htm.copy()
                    htm_i[:3,3] = init_point


                    f_index = i_index+1


                    if(f_index<points.shape[0]):
                        final_point = points[f_index]
                        htm_f = htm.copy()
                        htm_f[:3,3] = final_point
                        # print("final_point= ", final_point)
                    else:
                        htm_f = htm_i.copy()
                    # print("htm_i =\n",htm_i)
                    # print("htm_f =\n",htm_f)
                    if(i_index==0):
                        self.taylorLinearInterpolationAlgorithm(htm_i,htm_f,[0.1,0.1,0.1])
                    else:
                        self.taylorLinearInterpolationAlgorithm(htm_i,htm_f,[0.1,0.1,0.1])



        





# t = CalculateTrajectory()
# R_circle = 100
# PC = np.array([0,300,200])
# P1 = np.array([0,300,400])
# P2 = np.array([40,300,400])

# t.circularInterpolationAlgorithm(R_circle,PC,P1,P2)

# m_first = np.array([[0,1,0,150],
#                 [1,0,0,210],
#                 [0,0,-1,100],
#                 [0,0,0,1]])

# m_second = np.array([[0,1,0,-150],
#                     [1,0,0,210],
#                      [0,0,-1,100],
#                     [0,0,0,1]])

# t.taylorLinearInterpolationAlgorithm(m_first,m_second,[0.01,0.01,0.01])

# m_third = np.array([[0,1,0,-150],
#                 [1,0,0,210],
#                 [0,0,-1,300],
#                 [0,0,0,1]])

# t.taylorLinearInterpolationAlgorithm(m_second,m_third,[0.01,0.01,0.01])

# m_fourth = np.array([[0,1,0,150],
#                 [1,0,0,210],
#                 [0,0,-1,300],
#                 [0,0,0,1]])

# t.taylorLinearInterpolationAlgorithm(m_third,m_fourth,[0.01,0.01,0.01])

# m_first = np.array([[0,1,0,150],
#                 [1,0,0,210],
#                 [0,0,-1,100],
#                 [0,0,0,1]])

# t.taylorLinearInterpolationAlgorithm(m_fourth,m_first,[0.01,0.01,0.01])

# t.writeKinematicSolutions()