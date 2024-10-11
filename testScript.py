# file สำหรับตรวจคำตอบ
# ในกรณีที่มีการสร้าง function อื่น ๆ ให้ระบุว่า input-output คืออะไรด้วย
'''
ชื่อ_รหัส(ex: ธนวัฒน์_6461)
1.
2.
3.
'''
from spatialmath import SE3
from HW3_utils import FKHW3
from math import pi
from FRA333_HW3_6509_6523 import q_init,w_init,endEffectorJacobianHW3,checkSingularityHW3,computeEffortHW3

import roboticstoolbox as rtb
import numpy as np

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
# robot.plot(q_init,block=True)
#===========================================<ตรวจคำตอบข้อ 1>====================================================#
#code here
print('Jacob_sol')
print(endEffectorJacobianHW3(q_init))
print('Jacobe_check')
print(robot.jacobe(q_init))
#==============================================================================================================#
#===========================================<ตรวจคำตอบข้อ 2>====================================================#
#code here
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

print('-----------RTB-------------')
for i in q_list:
    J = robot.jacobe(i)
    J_linear = np.array(J[:3,:])
    M_mo = abs(np.linalg.det(J_linear))
    print(f'Manipulability: {M_mo}')
    if M_mo < 0.001:
        print(f'Flag:{1}')
    else:
        print(f'Flag:{0}')
#==============================================================================================================#
#===========================================<ตรวจคำตอบข้อ 3>====================================================#
#code here
effort = computeEffortHW3(q_init,w_init)
print(f'Effort: {effort}')
#==============================================================================================================#