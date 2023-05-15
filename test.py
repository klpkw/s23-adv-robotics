import numpy as np
from math import pi
import roboticstoolbox as rtb

# Define the robot links
L1 = rtb.RevoluteDH(d=0.1, a=0.8, alpha=-pi/2)
L2 = rtb.RevoluteDH(d=0, a=0.4, alpha=0)
L3 = rtb.RevoluteDH(d=0, a=0.6, alpha=0)
L4 = rtb.RevoluteDH(d=0, a=0.1, alpha=pi/2)

# Create the robot object
robot = rtb.SerialLink([L1, L2, L3, L4], name='my_robot')

# Define a robot configuration (in radians)
q = np.array([0, pi/2, -pi/4, pi/6])

# Plot the robot in this configuration
robot.plot(q)
input("Press Enter to close the plot window...")