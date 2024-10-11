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

R,P,R_e,p_e = FKHW3(q_init)
# print(len(R))
print(f'R:\n{R}')
# print(f'p_e:{p_e}')
# print(f'P:\n{P}')
# print(f'R_e:\n{R_e}')
# print(f'R_e_t:\n{R_e.transpose()}')
# print(f'p_e:\n{p_e}')
# for i in range(len(R)):
#     R_i = R[:,:,i] #  เข้าถึง Rotation ของแต่ละ Joint (R[:,:,i] = เข้าถึง 2D ใน 3D เช่น จาก 3x3x4 เป็น 3x3)
#     P_i = P[:,i] #  เข้าถึง Position ของแต่ละ Joint (P[:,i] = เข้าถึงแต่ละ colum ของ 2D เช่น จาก 3X3 เป็น 3X1)
#     print(f'R_{1+i}:\n{R_i}')
#     print(f'P_{1+i}:\n{P_i}')

def create_mdh_matrix(a=0, alpha=0, d=0, theta=0):
    T_matrix = np.array([
        [cos(theta), -sin(theta), 0, a],  
        [sin(theta)*cos(alpha), cos(theta)*cos(alpha), -sin(alpha), -sin(alpha)*d],
        [sin(theta)*sin(alpha), cos(theta)*sin(alpha), cos(alpha), cos(alpha)*d],
        [0, 0, 0, 1]
    ])
    return SE3(T_matrix)
tool = SE3([
[0, 0, -1, -(a_3 + d_6)],
[0, 1, 0, -d_5],
[1, 0, 0, d_4],
[0, 0, 0, 1]])
T_01 = create_mdh_matrix(theta=q_init[0] + pi, d=0.0892)
T_12 = create_mdh_matrix(alpha=pi/2, theta=q_init[1])
T_23 = create_mdh_matrix(a=-0.425,theta=q_init[2])

T_02 = T_01 @ T_12
T_03 = T_02 @ T_23
T_0e = T_03 @ tool
z = np.array([0,0,1])
z_1 = T_01.R @ z
z_2 = T_02.R @ z
z_3 = T_03.R @ z
# Z_i = [z_1,z_2,z_3]
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
print('Jacob_sol')
print(endEffectorJacobianHW3(q_init))
#==============================================================================================================#
#=============================================<คำตอบข้อ 2>======================================================#
#code here
def checkSingularityHW3(q:list[float])->bool:
    J = endEffectorJacobianHW3(q)
    J_linear = np.array(J[:3, :3])
    
    JJT = J_linear @ J_linear.T
    M = np.sqrt(np.linalg.det(JJT))
    print(f'Manipulability: {M}')
    if M < 0.001:
        return 0
    return 1
#==============================================================================================================#
#=============================================<คำตอบข้อ 3>======================================================#
#code here
def computeEffortHW3(q:list[float], w:list[float])->list[float]:
    pass
#==============================================================================================================#