import pprint
import sympy as sp 
import numpy as np



class KinematicsSymbolic:
    def __init__(self):
        th_1 = sp.Symbol("θ1")
        th_2 = sp.Symbol("θ2")
        th_3 = sp.Symbol("θ3")
        th_4 = sp.Symbol("θ4")
        th_5 = sp.Symbol("θ5")
        th_6 = sp.Symbol("θ6")


        #group symbolic angle variables set 
        self.s_th = (th_1,th_2,th_3,th_4,th_5,th_6)

        
        a_1 = sp.Symbol("α1")
        a_2 = sp.Symbol("α2")
        #a_3 = sp.Symbol("α3")
        a_4 = sp.Symbol("α4")
        a_5 = sp.Symbol("α5") 
        a_6 = sp.Symbol("α6")
        #a_7 = sp.Symbol("α7")

        #group symbolic dimension variables set
        self.s_a = (a_1,a_2,a_4,a_5,a_6)

        self.h_0_1 = self.HTM(th_1+sp.pi/2, sp.pi/2, 0,a_1)
        self.h_1_2 = self.HTM(th_2+sp.pi/2, 0, a_2,0)
        self.h_2_3 = self.HTM(th_3, sp.pi/2, a_4,0)
        self.h_3_4 = self.HTM(th_4, -sp.pi/2, 0,a_5)
        self.h_4_5 = self.HTM(th_5, sp.pi/2, 0,0)
        self.h_5_6 = self.HTM(th_6, 0, 0,a_6)

        self.p_3_4 = self.h_3_4.col(3)


        #Calculated required HTMs
        self.h_0_3 = self.h_0_1*self.h_1_2*self.h_2_3
        #h_0_4 = sp.simplify(h_0_3*h_3_4)

        self.h_3_6 = self.h_3_4*self.h_4_5*self.h_5_6


        self.h_0_6  = self.h_0_3*self.h_3_6

        sp.init_printing(use_unicode = True)



    def HTM (self,theta_n, alpha_n, r_n, d_n):
        h_i_j=sp.Matrix([[sp.cos(theta_n), -sp.sin(theta_n)*sp.cos(alpha_n), sp.sin(theta_n)*sp.sin(alpha_n), r_n*sp.cos(theta_n)],
                        [sp.sin(theta_n), sp.cos(theta_n)*sp.cos(alpha_n), -sp.cos(theta_n)*sp.sin(alpha_n), r_n*sp.sin(theta_n)],
                        [0, sp.sin(alpha_n), sp.cos(alpha_n), d_n],
                        [0, 0, 0, 1]])
        return h_i_j


    def calculateWristPositionVector(self):
            p_0_4 = self.h_0_3 * self.p_3_4
            # sp.pprint(sp.simplify(p_0_4))
    

        
    def calculateRotationMatricesSymbolic(self,print_res=False):

        r_0_3 =  self.h_0_3[:,:]
        r_0_3.row_del(3)
        r_0_3.col_del(3)

        # r_0_3.transpose()
        
        r_3_6 =  self.h_3_6[:,:]
        r_3_6.row_del(3)
        r_3_6.col_del(3)
        r_0_3 = sp.simplify(r_0_3)
        r_3_6 = sp.simplify(r_3_6)


        r_0_6 =sp.Matrix([[0,1,0],
                [0,0,1],
                [1,0,0]])


        r_0_3 = sp.simplify(r_0_3)
        r_3_6 = sp.simplify(r_3_6)
        r_0_6 = sp.simplify(r_0_6)

        r_3_6_sol = r_0_3.transpose()*r_0_6

        
        nd_r_0_3 = sp.lambdify(self.s_th[:3], r_0_3,modules='numpy')

        if(print_res == True):
            print("\n###############################ROTATION MATRICES################################\n")
            print("\n###################################r_0_3########################################\n")
            sp.pprint(r_0_3)
            print("\n###################################r_3_6########################################\n")
            sp.pprint(r_3_6)
            print("\n###################################r_3_6(sol)########################################\n")
            sp.pprint(sp.simplify(r_3_6_sol))

        return nd_r_0_3

    def getForwardKinematicsHTM(self, print_res = False):
        s_u = self.s_th+self.s_a
        end_point = sp.lambdify(s_u,self.h_0_6,modules='numpy')
        if(print_res == True):
            print("\n###############################HOMOGENEOUS MATRICES################################\n")
            print("\n###################################h_0_3########################################\n")
            sp.pprint(self.h_0_6[0,3])

        return end_point



    def calculateJacobian(self):
        
        d_var = sp.Matrix([self.s_th[:3]])
        p_0_4 = self.h_0_3 * self.p_3_4

        new_p_0_4 = sp.Matrix(p_0_4[:-1])
        
        j_xyz = new_p_0_4.jacobian(d_var)     #X,Y,Z relative to θ1,θ2,θ3
     

        s_u = self.s_th[:3] +self.s_a
        v = sp.lambdify(s_u,j_xyz,modules='numpy')
        return v







