# file สำหรับเขียนคำตอบ
# ในกรณีที่มีการสร้าง function อื่น ๆ ให้ระบุว่า input-output คืออะไรด้วย
'''
ชื่อ_รหัส(ธนวัฒน์_6461)
1.ชญานิน_6509
2.ณัฐภัทร_6523
'''
from HW3_utils import FKHW3
import numpy as np

#=============================================<คำตอบข้อ 1>======================================================#
#code here
def endEffectorJacobianHW3(q:list[float])->list[float]:
    # Forward Kinematic
    R,P,R_e,p_e = FKHW3(q)
    # create empty matrix for add Jacobian
    J = np.zeros([6, len(q)]) # matrix 6x3
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
    J_linear = np.array(J[:3,:]) # ลดรูป jacobian ให้เหลือแค่ส่วนของ linear (3x3)
    S = abs(np.linalg.det(J_linear)) # หาสภาวะ Singularity โดยการใส่ det ใน jacobian ที่ลดรูป และ absolute
    print(f'Singularity: {S}')
    if S < 0.001:
        return 1 # ใกล้สภาวะ Singularity
    else:
        return 0 # สภาวะปกติ
#==============================================================================================================#
#=============================================<คำตอบข้อ 3>======================================================#
#code here
def computeEffortHW3(q:list[float], w:list[float])->list[float]:
    J = np.array(endEffectorJacobianHW3(q))
    J_Trans = J.transpose() # Transpose ฟังก์ชัน Jacobian
    effort = J_Trans @ w # หา effort
    return effort
#==============================================================================================================#