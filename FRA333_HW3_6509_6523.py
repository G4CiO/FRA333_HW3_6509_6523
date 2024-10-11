# file สำหรับเขียนคำตอบ
# ในกรณีที่มีการสร้าง function อื่น ๆ ให้ระบุว่า input-output คืออะไรด้วย
'''
ชื่อ_รหัส(ธนวัฒน์_6461)
1.
2.
3.
'''
from HW3_utils import FKHW3
from math import pi,sin,cos
import numpy as np
from spatialmath import SE3
d_1 = 0.0892
a_2 = 0.425
a_3 = 0.39243
d_4 = 0.109
d_5 = 0.093
d_6 = 0.082

q_init = [0.0, 0.0, 0.0]
w_init = [0.0, 10.0, 0.0, 0.0, 0.0, 0.0] # force,moment
# R,P,R_e,p_e = FKHW3(q_init)
# print(len(R))
# print(f'R:\n{R}')
# print(f'R_e:\n{R_e}')
# print(f'p_e:{p_e}')
# print(f'P:\n{P}')
# print(f'R_e_t:\n{R_e.transpose()}')
# print(f'p_e:\n{p_e}')
# for i in range(len(R)):
#     R_i = R[:,:,i] #  เข้าถึง Rotation ของแต่ละ Joint (R[:,:,i] = เข้าถึง 2D ใน 3D เช่น จาก 3x3x4 เป็น 3x3)
#     P_i = P[:,i] #  เข้าถึง Position ของแต่ละ Joint (P[:,i] = เข้าถึงแต่ละ colum ของ 2D เช่น จาก 3X3 เป็น 3X1)
#     print(f'R_{1+i}:\n{R_i}')
#     print(f'P_{1+i}:\n{P_i}')
#=============================================<คำตอบข้อ 1>======================================================#
#code here
def endEffectorJacobianHW3(q:list[float])->list[float]:
    # Forward Kinematic
    R,P,R_e,p_e = FKHW3(q)
    # create empty matrix for add Jacobian
    J = np.zeros([6, len(q)]) # matrix 6x...
    # Find Jacobian
    for i in range(len(q)):
        P_i = P[:,i] # Position from {0} to {i}
        Z_i = R[:, 2, i] # Rotation z axis of each joint
        J[:3, i] = R_e.transpose() @ (np.cross(Z_i, p_e - P_i))  # Add Linear Jacobian from row 1 to row 3 that reference {e}
        J[3:, i] = R_e.transpose() @ Z_i # Add Angular Jacobian from row 4 to row 6
    return J
#==============================================================================================================#
#=============================================<คำตอบข้อ 2>======================================================#
#code here
def checkSingularityHW3(q:list[float])->bool:
    J = endEffectorJacobianHW3(q)
    J_linear = np.array(J[:3,:])
    M = abs(np.linalg.det(J_linear))
    print(f'Manipulability: {M}')
    if M < 0.001:
        return 1
    else:
        return 0
#==============================================================================================================#
#=============================================<คำตอบข้อ 3>======================================================#
#code here
def computeEffortHW3(q:list[float], w:list[float])->list[float]:
    J = endEffectorJacobianHW3(q)
    J_Reduce_Trans = np.transpose(J)
    effort = J_Reduce_Trans @ w
    return effort
#==============================================================================================================#