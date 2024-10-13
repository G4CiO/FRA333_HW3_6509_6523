# file สำหรับตรวจคำตอบ
# ในกรณีที่มีการสร้าง function อื่น ๆ ให้ระบุว่า input-output คืออะไรด้วย
'''
ชื่อ_รหัส(ex: ธนวัฒน์_6461)
1.ชญานิน_6509
2.ณัฐภัทร_6523
'''
from HW3_utils import FKHW3
from spatialmath import SE3
from math import pi,hypot,cos,atan2
from FRA333_HW3_6509_6523 import endEffectorJacobianHW3,checkSingularityHW3,computeEffortHW3
import roboticstoolbox as rtb
import numpy as np
#==============================================================================================================#
#================================<สร้าง MDH-Parameters ด้วย robotic toolbox>======================================#
d_1 = 0.0892
a_2 = 0.425
a_3 = 0.39243
d_4 = 0.109
d_5 = 0.093
d_6 = 0.082

robot = rtb.DHRobot(
    [
        rtb.RevoluteMDH(alpha = 0.0     ,a = 0.0      ,d = d_1    ,offset = pi ),
        rtb.RevoluteMDH(alpha = pi/2    ,a = 0.0      ,d = 0.0    ,offset = 0.0),
        rtb.RevoluteMDH(alpha = 0.0     ,a = -a_2     ,d = 0.0    ,offset = 0.0),
    ],
    tool = SE3([
    [0, 0, -1, -(a_3 + d_6)],
    [0, 1, 0, -d_5],
    [1, 0, 0, d_4],
    [0, 0, 0, 1]]),
    name = "3DOF_Robot"
)
#==============================================================================================================#
#=========================<สร้างตัวแปร joint เริ่มต้น(q_init) และ wrench เริ่มต้น(w_init)>==============================#
q_init = [0.0, 0.0, 0.0]
w_init = [10.0, 0.0, 0.0, 0.0, 0.0, 0.0] # force x,y,z ,moment x,y,z
#==============================================================================================================#
#=========================<เช็คว่า FK ของ roboticstoolbox กับ ของ FKHW3 ตรงกันหรือไม่>==============================#
print('#=========================<เช็คว่า FK ของ roboticstoolbox กับ ของ FKHW3 ตรงกันหรือไม่>===============================#')
print('FK ของ roboticstoolbox')
print(robot.fkine(q_init))
print('FK ของ FKHW3')
R,P,R_e,p_e = FKHW3(q_init)
print(SE3.Rt(R_e, p_e))
#==============================================================================================================#
print('#===========================================<ตรวจคำตอบข้อ 1>====================================================#')
#===========================================<ตรวจคำตอบข้อ 1>====================================================#
#code here
'''
ตรวจคำตอบโดยเปรียบเทียบคำตอบของฟังก์ชัน jacobian ที่ได้จากการคำนวณเอง กับที่ได้จาก robotics-toolbox
'''
J_HW3 = endEffectorJacobianHW3(q_init)

# Set precision for easier comparison
np.set_printoptions(precision=4, suppress=True)

print('Jacob_HW3:')
print(J_HW3) # แสดงคำตอบของฟังก์ชัน jacobian ที่ได้จากการคำนวณ
print('Jacob_RTB:')
print(robot.jacobe(q_init)) # แสดงคำตอบที่ได้จาก robotictoolsbox
#==============================================================================================================#
print('#===========================================<ตรวจคำตอบข้อ 2>====================================================#')
#===========================================<ตรวจคำตอบข้อ 2>====================================================#
#code here
'''
ตรวจการเช็ค singularity โดยเปรียบเทียบค่า det ที่ได้จากฟังก์ชัน jacobian ที่ได้จากการคำนวณเอง กับ ฟังก์ชัน jacobian ของ robotics-toolbox
'''
q1 = [0.0, 0.0, 0.0]
q2 = [0.0, 0.0, -pi/2]
q3 = [0.0, pi/4, pi/2]
qs1 = [-1.91970470e-15, -8.35883143e-01, 2.80232546e+00]
qs2 = [-0.24866892, 0.22598268, -0.19647569]
qs3 = [1.70275090e-17, -1.71791355e-01, -1.95756090e-01]
q_list = [q1,q2,q3,qs1,qs2,qs3]
print('-----------HW3-------------')
for i in q_list:
    flag = checkSingularityHW3(i)
    print(f'Flag:{flag}')
    # robot.plot(i,block=True)
print('-----------RTB-------------')
for i in q_list:
    J = robot.jacobe(i)
    J_linear = np.array(J[:3,:])
    S = abs(np.linalg.det(J_linear))
    print(f'Singularity: {S}')
    if S < 0.001:
        print(f'Flag:{1}')
    else:
        print(f'Flag:{0}')
#==============================================================================================================#
print('#===========================================<ตรวจคำตอบข้อ 3>====================================================#')
#===========================================<ตรวจคำตอบข้อ 3>====================================================#
#code here
'''
ตรวจค่า effort ที่ได้จากฟังก์ชัน jacobian ที่มาจากคำนวณเองกับจาก robotics-toolbox
'''
effort_HW3 = computeEffortHW3(q_init,w_init)
print(f'Effort_HW3: {effort_HW3}')

ef0 = effort_HW3[0]/(a_2+a_3+d_6)
ef1 = effort_HW3[1]/(a_2+a_3+d_6)
ef2 = effort_HW3[2]/(a_3+d_6)
print(f'T1: {ef0} N\nT2:{ef1} N\nT3:{ef2} N')
# -----------------------------------------------------------
J = robot.jacobe(q_init)
effort_RTB = robot.pay(W=w_init,J=J)
# joint_efforts = np.dot(J.T, w_init)
print(f'Effort_RTB: {effort_RTB}')

ef0 = effort_RTB[0]/(a_2+a_3+d_6)
ef1 = effort_RTB[1]/(a_2+a_3+d_6)
ef2 = effort_RTB[2]/(a_3+d_6)
print(f'T1: {ef0} N\nT2:{ef1} N\nT3:{ef2} N')

# fk = robot.fkine(q_init)
# J0 = robot.jacob0(q_init)
# w_0 = fk.R @ w_init[:3]
# w_00 = [w_0[0],w_0[1],w_0[2],0.0,0.0,0.0]
# joint_efforts = J0.T @ np.array(w_00).transpose()
# print(f'Effort_RTB_0: {joint_efforts}')
#==============================================================================================================#