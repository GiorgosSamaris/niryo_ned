import numpy as np
import sympy as sp
import InverseKinematicsSymbolic as iks
import warnings


class CalulateInverseKinematics:

    def __init__(self):
        th_1 = sp.Symbol("θ1")
        th_2 = sp.Symbol("θ2")
        th_3 = sp.Symbol("θ3")
        th_4 = sp.Symbol("θ4")
        th_5 = sp.Symbol("θ5")
        th_6 = sp.Symbol("θ6")

        sym = iks.CalulateInverseKinematicSymbolic()

        r_0_3, self.r_3_6 = sym.calculateRotationMatricesSymbolic(False)
        warnings.filterwarnings("ignore", category=np.VisibleDeprecationWarning) 
        e = r_0_3
        
        s_1 = (th_1,th_2,th_3)
        self.nd_e = sp.lambdify(s_1, e,modules='numpy')
      

    def nedInverseKinematics(self, target_matrix):
        a_1 = 171.5
        a_2 = 221
        a_4 = 32.5
        a_5 = 245 #added a_3
        a_6 = 100


        r_0_6 = target_matrix[:3,:3]



        p_x = target_matrix[0,3]
        p_y = target_matrix[1,3]
        p_z = target_matrix[2,3]

        w_x = p_x
        w_y = p_y
        w_z = p_z+a_6

        #used to denote a +- solution
        pm = np.array([1, -1])

        #position vectors from 4 to 1
        p_z = pm*np.sqrt((w_z-a_1)**2)

        p_x = pm*np.sqrt(w_x**2+w_y**2)
        # print(p_z)

        ro = pm*np.sqrt(p_z**2 + p_x**2)
        #/////////////////////////////////find th_1 /////////////////////////////////////

        #we are subtracting π/2 because the simulators 0 deg pos is on the y axis and not the x
        th_1 = np.arctan2(w_y/p_x,w_x/p_x) - np.pi/2

        #/////////////////////////////////find th_3 /////////////////////////////////////

        a_45 = pm*np.sqrt(a_4**2+a_5**2)



        #find phi
        c_phi = (ro**2 - a_45**2-a_2**2)/-(2*a_2*a_45)


        phi = np.arccos(c_phi)


        #find th_a angle 
        th_a = np.arctan2(a_4,a_5)

        #find th_3 angle

        th_3 = np.pi -phi - th_a


        #/////////////////////////////////find th_2 /////////////////////////////////////

        s_2 = (p_x*(a_2+a_4*np.cos(th_3)+a_5*np.sin(th_3))-p_z*(a_4*np.sin(th_3)-a_5*np.cos(th_3)))/((a_5*np.cos(th_3)-a_4*np.sin(th_3))**2+(a_2+a_4*np.cos(th_3)+a_5*np.sin(th_3))**2)
        # c_2 = ()
        # print(s_2)
        th_2 = np.arcsin(s_2)

        #/////////////////////////////////find th_5 /////////////////////////////////////
        e_sol = self.nd_e(th_1,th_2,th_3)
        e_sol = np.matmul(e_sol.transpose(),r_0_6 )
        c_5 = e_sol[2,2]
        s_5 = pm*np.sqrt(1-c_5**2)
        # th_5 = np.arccos(c_5)
        th_5 = np.arctan2(s_5,c_5) 
        #/////////////////////////////////find th_6 /////////////////////////////////////
        if(np.sin(th_5).all!= 0):
            s_6 = e_sol[2,1]/np.sin(th_5)
            c_6 = e_sol[2,0]/np.sin(th_5)
            th_6 = np.arctan2(-s_6,c_6)
            #/////////////////////////////////find th_4 /////////////////////////////////////
            s_4 = e_sol[1,2]/np.sin(th_5)
            
            c_4 = e_sol[0,2]/np.sin(th_5)

            th_4 = np.arctan2(-s_4,c_4)

        else: 
            th_6 = [0,0]
            th_4 = [0,0]

        th_3 = np.pi/2 - th_3
        return np.array([th_1,th_2,th_3,th_4,th_5,th_6])


def main():

    t_m = np.array([[1,0,0,-230],
                [0,1,0,270],
                [0,0,1,15],
                [0,0,0,1]])
    ned = CalulateInverseKinematics()   
    
    for x in range(100):
        print(ned.nedInverseKinematics(t_m))
        


if __name__ == '__main__':
    main()

    # print("////////IN RADS////////")
    # print("θ1: ",str(th_1))
    # print("θ2: ",str(th_2))
    # print("θ3: ",str(th_3))
    # print("θ4: ",str(th_4))
    # print("θ5: ",str(th_5))
    # print("θ6: ",str(th_6))
    # th_1 = np.rad2deg(th_1)
    # th_2 = np.rad2deg(th_2)
    # th_3 = np.rad2deg(th_3)
    # th_4 = np.rad2deg(th_4)
    # th_5 = np.rad2deg(th_5)
    # th_6 = np.rad2deg(th_6)
    # print("////////IN DEGS////////")
    # print("θ1: ",str(th_1))
    # print("θ2: ",str(th_2))
    # print("θ3: ",str(th_3))
    # print("θ4: ",str(th_4))
    # print("θ5: ",str(th_5))
    # print("θ6: ",str(th_6))

