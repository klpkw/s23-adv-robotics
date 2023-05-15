import numpy as np
import matplotlib.pyplot as plt
from spatialmath import SE3
import roboticstoolbox as rtb


def check_colission(p, obstacle_list):
    "Robot: "
    math_robot = rtb.models.Puma560()
    Nj = len(math_robot.qd)

    "Transformation matrix: "
    T = np.array([[0, 1, 0, p[0]], [0, 0, -1, p[1]], [-1, 0, 0, p[2]], [0, 0, 0, 1]])

    "Inverse kinematics: "
    q_set = math_robot.ik_lm_chan(T)[0]

    "Forward kinematics for all joints: "
    T_list = [np.asarray(math_robot.fkine_all(q_set)[i]) for i in range(Nj)]
    # print(T_list[3])

    "Robot geometry as a point cloud: "
    N1 = 20 # Number of points in each link
    Robot_points = []
    for j in range(Nj):
        link_points = np.zeros((N1, 3), dtype = np.float32)
        if j != 0:
            for i in range(N1):
                for k in range(3):
                    link_points[i, k] = T_list[j-1][k, 3] + (T_list[j][k, 3] - T_list[j-1][k, 3]) * i / N1
        else:
            for i in range(N1):
                for k in range(3):
                    link_points[i, k] = T_list[0][k, 3] * i / N1  

        Robot_points.append(link_points)

    "Check if robot point lies in obstacle: "
    for j in range(Nj):
        link_points = Robot_points[j]

        for i in range(N1):
                # some point of robot  
                x, y, z = link_points[i, 0], link_points[i, 1], link_points[i, 2]

                # check: 
                for obstacle in obstacle_list:
                    # if lies in circle:
                    if (x - obstacle[0])**2 + (y - obstacle[1])**2 <= obstacle[2]**2:
                        # if z coord smaller then height: 
                        if z >= 0:
                            if z <= obstacle[3]:
                                return False
                        else:
                            return False
                        
    return True


if __name__ == "__main__":
    # "Obstacles: " # xc, yc, rc, heigh
    # obstacle1 = np.array([0.1, -0.05, 0.2, 0.2])
    # obstacle_list = [obstacle1]
    #
    # "Coord: "
    # p = np.array([0.1, -0.05, 0.1])
    # print(check_colission(p, obstacle_list))
    # 
    math_robot = rtb.models.Puma560()
    print(math_robot.fkine_all([np.pi / 3 for j in range(6)]))
