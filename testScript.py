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
from FRA333_HW3_6509_6523 import q_init,checkSingularityHW3

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
q = [0.0,0.0,-pi]
robot.plot(q,block=True)



# print('FK_Check')
# print(robot.fkine(q))
# print('FK_TA')
# R,P,R_e,p_e = FKHW3(q)
# print(p_e)
#===========================================<ตรวจคำตอบข้อ 1>====================================================#
#code here

print('Jacobe_check')
print(robot.jacobe(q_init))

#==============================================================================================================#
#===========================================<ตรวจคำตอบข้อ 2>====================================================#
#code here
flag = checkSingularityHW3(q)
print(f'Flag:{flag}')
# J_linear = robot.jacobe(q_init)
# print(np.linalg.det(J_linear))
#==============================================================================================================#
#===========================================<ตรวจคำตอบข้อ 3>====================================================#
#code here

#==============================================================================================================#