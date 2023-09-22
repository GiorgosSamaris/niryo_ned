import numpy as np
import sympy as sp
import KinematicsSymbolic as iks
import warnings


class Kinematics:

    def __init__(self):
        self.sym = iks.KinematicsSymbolic()

        #get rotation matrices to later calculate orientation of end effector
        self.r_0_3 = self.sym.calculateRotationMatricesSymbolic(False)
        warnings.filterwarnings("ignore", category=np.VisibleDeprecationWarning) 

        #get HTM of ned
        self.h_0_3 = self.sym.getForwardKinematicsHTM(False)
        
      

    def nedInverseKinematics(self, target_matrix):
        a_1 = 171.5
        a_2 = 221
        a_4 = 32.5
        a_5 = 235 #+10 #added a3 
        a_6 = 100 

        r_0_6 = target_matrix[:3,:3]

        w_x = target_matrix[0,3]
        w_y = target_matrix[1,3]
        w_z = target_matrix[2,3] - r_0_6[2,2]*a_6
        # print("target wrist pos")
        # print("x:",w_x)
        # print("y:",w_y)
        # print("z:",w_z)

        #used to denote a +- solution
        pm = np.array([1, -1])

        #position vectors from 4 to 1   
        p_z = w_z-a_1
        p_x = np.sqrt(w_x**2+w_y**2)
        ro = pm*np.sqrt(p_z**2 + p_x**2)


        #/////////////////////////////////find th_1 /////////////////////////////////////

        #we are subtracting π/2 because the simulators 0 deg pos is on the y axis and not the x
        th_1 = np.arctan2(w_y/p_x*pm,w_x/p_x*pm) -  np.pi/2
        

        #/////////////////////////////Elbow down solution////////////////////////////////
        #/////////////////////////////////find th_3 /////////////////////////////////////

        a_45 = np.sqrt(a_4**2+a_5**2)
        beta = np.arctan2(a_4,a_5)


        #find phi
        c_phi = ((a_45**2)+(a_2**2)-(ro[0]**2))/(2*a_2*a_45)
        s_phi = np.sqrt(1-c_phi**2)
        phi = np.arctan2(s_phi,c_phi)
       

        #find th_3 angle
        th_3_d = np.pi -phi -beta
       



        #/////////////////////////////////find th_2 /////////////////////////////////////

        
        psi = np.arctan2(p_z,p_x)
        alpha = np.arctan2(a_45*np.sin(np.pi-phi),a_45*np.cos(np.pi-phi)+a_2)
        
       
        th_2_d = psi - alpha
        

        #/////////////////////////////Elbow up solution////////////////////////////////
        #/////////////////////////////////find th_3 /////////////////////////////////////
        th_3_u = -(np.pi - phi + beta)

        #/////////////////////////////////find th_2 /////////////////////////////////////
       
        th_2_u = psi + alpha
      
        th_2 = np.array([th_2_d,th_2_u])
        th_3 = np.array([th_3_d,th_3_u])
        
        #/////////////////////////////////find th_5 /////////////////////////////////////
        e_sol = self.r_0_3(th_1,th_2,th_3)
        e_sol = np.matmul(e_sol.transpose(),r_0_6)
      
        c_5 = e_sol[2,2]
        s_5 = np.sqrt(1-c_5**2)
        # th_5 = np.arccos(c_5)
        th_5 = np.arctan2(s_5,c_5) 
        #/////////////////////////////////find th_6 /////////////////////////////////////
        if(np.sin(th_5).all()!= 0):
            s_6 = e_sol[2,1]/np.sin(th_5)
            c_6 = e_sol[2,0]/np.sin(th_5)
            th_6 = np.arctan2(-s_6,c_6)

            #/////////////////////////////////find th_4 /////////////////////////////////////
            s_4 = e_sol[1,2]/np.sin(th_5)
            c_4 = e_sol[0,2]/np.sin(th_5)
            th_4 = np.arctan2(s_4,c_4)

        else: 
            th_6 = [0,0]
            th_4 = [0,0]
            
        th_3 = np.pi/2 + th_3
        th_2 = -np.pi/2 + th_2
        
        return np.array([th_1,th_2,th_3,th_4,th_5,th_6])
    
    def nedForwardKinematics(self, joint_angles):
        th_1 = joint_angles[0]
        th_2 = joint_angles[1]
        th_3 = joint_angles[2]
        th_4 = joint_angles[3]
        th_5 = joint_angles[4]
        th_6 = joint_angles[5]
        a_1 = 171.5
        a_2 = 221
        a_4 = 32.5
        a_5 = 235# +10 #added a3 
        a_6 = 100 
        end_eff_pos = self.h_0_3(th_1,th_2,th_3,th_4,th_5,th_6,a_1,a_2,a_4,a_5,a_6)
        return end_eff_pos
    

    def nedEndEffectorVelocity(self, target_velocity, joint_angles):

        a_1 = 171.5
        a_2 = 221
        a_4 = 32.5
        a_5 = 235# +10 #added a3 
        a_6 = 100 

        th_1 = joint_angles[0]
        th_2 = joint_angles[1]
        th_3 = joint_angles[2]
        
        J_lambda = self.sym.calculateJacobian()
        J = np.array(J_lambda(th_1,th_2,th_3,a_1,a_2,a_4,a_5,a_6))
        inv_J = np.linalg.inv(J)
        # print(np.shape(inv_J))
        return np.matmul(inv_J,target_velocity)
       

   





 

