import sympy as sp 
import numpy as np


def HMT (theta_n, alpha_n, r_n, d_n):
    h_i_j=sp.Matrix([[sp.cos(theta_n), -sp.sin(theta_n)*sp.cos(alpha_n), sp.sin(theta_n)*sp.sin(alpha_n), r_n*sp.cos(theta_n)],
                     [sp.sin(theta_n), sp.cos(theta_n)*sp.cos(alpha_n), -sp.cos(theta_n)*sp.sin(alpha_n), r_n*sp.sin(theta_n)],
                     [0, sp.sin(alpha_n), sp.cos(alpha_n), d_n],
                     [0, 0, 0, 1]])
    return h_i_j


th_1 = sp.Symbol("θ1")
th_2 = sp.Symbol("θ2")
th_3 = sp.Symbol("θ3")
th_4 = sp.Symbol("θ4")
th_5 = sp.Symbol("θ5")
th_6 = sp.Symbol("θ6")

a_1 = sp.Symbol("α1")
a_2 = sp.Symbol("α2")
a_3 = sp.Symbol("α3")
a_4 = sp.Symbol("α4")
a_5 = sp.Symbol("α5")
a_6 = sp.Symbol("α6")
a_7 = sp.Symbol("α7")
x = sp.Symbol("x")
y = sp.Symbol("y")
z = sp.Symbol("z")

r = sp.Symbol("r")

phi = sp.Symbol("φ")

sp.init_printing(use_unicode = True)

a_3 = 0


h_0_1 = HMT(th_1+sp.pi/2, sp.pi/2, 0,a_1)
h_1_2 = HMT(th_2+sp.pi/2, 0, a_2,0)
h_2_3 = HMT(th_3, sp.pi/2, a_4,0)
h_3_4 = HMT(th_4, -sp.pi/2, 0,a_5+a_3)
h_4_5 = HMT(th_5, sp.pi/2, 0,0)
h_5_6 = HMT(th_6, 0, 0,a_6)

p_3_4 = h_3_4.col(3)



h_0_3 = h_0_1*h_1_2*h_2_3
#h_0_4 = sp.simplify(h_0_3*h_3_4)

h_3_6 = h_3_4*h_4_5*h_5_6

h_0_6  = h_0_3*h_3_6


#get the rotation matrices
r_0_6 =  sp.Matrix([[1,0,0],
                    [0,1,0],
                    [0,0,1]])

r_0_3 =  h_0_3[:,:]
r_0_3.row_del(3)
r_0_3.col_del(3)

r_3_6 =  h_3_6[:,:]
r_3_6.row_del(3)
r_3_6.col_del(3)


r_3_6_sol =  r_0_3.transpose()*r_0_6

p_0_4 = h_0_3 * p_3_4


h_4_6 = h_4_5*h_5_6

p_0_4_sol = h_4_6.transpose()*sp.Matrix([[x],[y],[z],[1]])

p_x = sp.simplify(p_0_4[0]**2 + p_0_4[1]**2)

p_z = p_0_4[2]-a_1

ro = sp.simplify(p_x + p_z**2)

print("#############################HOMOGENEOUS MATRICES###############################\n")
print("\n###################################h_0_3########################################\n")
sp.pprint(sp.simplify(h_0_3))
print("\n###################################h_0_6########################################\n")
sp.pprint(sp.simplify(h_0_6))
print("\n###############################ROTATION MATRICES################################\n")
sp.pprint(sp.simplify(r_3_6))
sp.pprint(sp.simplify(r_3_6_sol))
print("\n###############################POSITION MATRICES################################\n")
sp.pprint(sp.simplify(p_0_4))


# sp.pprint(sp.simplify(p_0_4_sol))





