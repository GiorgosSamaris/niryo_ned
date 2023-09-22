import numpy as np
import TrajectoryGenerator as tg
import Kinematics as kin
import csvExport as 


ned = kin.Kinematics()

print("1) Go to point and return to home")
print("2) Execute linear trajectory from point A to point B")
print("3) Execute orthogonal trajectory")
print("4) Execute circular trajectory")
choice = float(input("Choose demo by number:"))


if(choice == 1):
    m_target = np.array([[0,1,0,150],
                [1,0,0,210],
                [0,0,-1,300],
                [0,0,0,1]])
    q_sol = ned.nedInverseKinematics(m_target)
