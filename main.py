import roboticstoolbox as rtb
import numpy as np

L1 = rtb.RevoluteDH(d=0.202, a=0, offset=0, alpha=np.pi/2, m=0, r=[0, 0, 0])
L2 = rtb.RevoluteDH(d=0, a=0.160, offset=0, alpha=0, m=1, r=[0, 0, 0.101])
L3 = rtb.RevoluteDH(d=0, a=0, offset=np.pi/2, alpha=np.pi/2, m=1, r=[0, 0, 0.080])
L4 = rtb.RevoluteDH(d=0.195, a=0, offset=0, alpha=-np.pi/2, m=1, r=[0, 0, 0.040])
L5 = rtb.RevoluteDH(d=0, a=0, offset=0, alpha=np.pi/2, m=1, r=[0, 0, 0.140])
L6 = rtb.RevoluteDH(d=0.06715, a=0, offset=0, alpha=0, m=1, r=[0, 0, 0.035])

math_robot = rtb.DHRobot([L1, L2, L3, L4, L5, L6])

L1_real = rtb.RevoluteDH(d=0.205, a=0, offset=0, alpha=np.pi/2, m=0, r=[0, 0, 0])
L2_real = rtb.RevoluteDH(d=0, a=0.158, offset=0, alpha=0, m=1, r=[0, 0, 0.101])
L3_real = rtb.RevoluteDH(d=0, a=0, offset=np.pi/2, alpha=np.pi/2, m=1, r=[0, 0, 0.080])
L4_real = rtb.RevoluteDH(d=0.194, a=0, offset=0, alpha=-np.pi/2, m=1, r=[0, 0, 0.040])
L5_real = rtb.RevoluteDH(d=0, a=0, offset=0, alpha=np.pi/2, m=1, r=[0, 0, 0.140])
L6_real = rtb.RevoluteDH(d=0.069, a=0, offset=0, alpha=0, m=1, r=[0, 0, 0.035])

real_robot=rtb.DHRobot([L1_real,L2_real,L3_real,L4_real,L5_real,L6_real])


q_zeros = [0, 0, 0, 0, 0, 0]
q0 = [np.pi/3, np.pi/6,np.pi/4,np.pi/6,np.pi/6,np.pi/4]
q_used = q0


real_robot.plot(q_used,jointaxes=True)


input()