#!/usr/bin/python3

import rospy
import numpy as np
from numpy import linalg


import cmath
import math
from math import cos as cos
from math import sin as sin
from math import atan2 as atan2
from math import acos as acos
from math import asin as asin
from math import sqrt as sqrt
from math import pi as pi

global mat
mat = np.matrix

class RobotKinematics(object): 

    def __init__(self):
        rospy.init_node("robot_kinematics")
        self.d = rospy.get_param(
            'D-H_d', [127.3, 0, 0, 163.9, 115.7,92.2])
        self.a = rospy.get_param(
            'D-H_a', [0, -612,-572.3,0,0,0])
        self.alph = rospy.get_param(
            'D-H_alph', [pi/2, 0, 0, pi/2, -pi/2, 0])
        self.target_pos= mat(rospy.get_param(
            'target_pos', [1,1,1,1]))
        self.D = mat(self.d)
        self.A = mat(self.a)
        self.Alph = mat(self.alph)


    # FORWARD KINEMATICS
    def AH(self,n, th, c):

        T_a = mat(np.identity(4), copy=False)
        T_a[0, 3] = self.A[0, n-1]
        T_d = mat(np.identity(4), copy=False)
        T_d[2, 3] = self.D[0, n-1]

        Rzt = mat([[cos(th[n-1, c]), -sin(th[n-1, c]), 0, 0],
                [sin(th[n-1, c]),  cos(th[n-1, c]), 0, 0],
                [0,               0,              1, 0],
                [0,               0,              0, 1]], copy=False)

        Rxa = mat([[1, 0,                 0,                  0],
                [0, cos(self.Alph[0, n-1]), -sin(self.Alph[0, n-1]),   0],
                [0, sin(self.Alph[0, n-1]),  cos(self.Alph[0, n-1]),   0],
                [0, 0,                 0,                  1]], copy=False)

        A_i = T_d * Rzt * T_a * Rxa

        return A_i


    def HTrans(th, c,self):
        A_1 = self.AH(1, th, c)
        A_2 = self.AH(2, th, c)
        A_3 = self.AH(3, th, c)
        A_4 = self.AH(4, th, c)
        A_5 = self.AH(5, th, c)
        A_6 = self.AH(6, th, c)

        T_06 = A_1*A_2*A_3*A_4*A_5*A_6

        # print Forward Kinematics Matrix
        print("Forward Kinematics Matrix:")
        print(T_06)

        return T_06

    # INVERSE KINEMATICS

    def invKine(self):

        th = mat(np.zeros((6, 8)))
        P_05 = (np.matmul(self.target_pos, np.array([0, 0, -self.d[5], 1]).reshape((4, 1))) - np.array([0, 0, 0, 1]).reshape((4, 1)))
        # **** theta1 ****

        psi = atan2(P_05[2-1, 0], P_05[1-1, 0])
        argument = self.d[3] / sqrt(P_05[2-1, 0]*P_05[2-1, 0] + P_05[1-1, 0]*P_05[1-1, 0])
        argument = max(min(argument, 1), -1)  # 确保它在[-1, 1]内
        phi = acos(argument)
        # The two solutions for theta1 correspond to the shoulder
        # being either left or right
        th[0, 0:4] = pi/2 + psi + phi
        th[0, 4:8] = pi/2 + psi - phi
        th = th.real

        # **** theta5 ****

        cl = [0, 4]  # wrist up or down
        for i in range(0, len(cl)):
            c = cl[i]
            T_10 = linalg.inv(self.AH(1, th, c))
            T_16 = T_10 * self.target_pos
            th[4, c:c+2] = + acos((T_16[2, 3]-self.d[3])/self.d[5])
            th[4, c+2:c+4] = - acos((T_16[2, 3]-self.d[3])/self.d[5])

        th = th.real

        cl = [0, 2, 4, 6]
        for i in range(0, len(cl)):
            c = cl[i]
            T_10 = linalg.inv(self.AH(1, th, c))
            T_16 = linalg.inv(T_10 * self.target_pos)
            th[5, c:c+2] = atan2((-T_16[1, 2]/sin(th[4, c])),
                                (T_16[0, 2]/sin(th[4, c])))

        th = th.real

        # **** theta3 ****
        cl = [0, 2, 4, 6]
        for i in range(0, len(cl)):
            c = cl[i]
            T_10 = linalg.inv(self.AH(1, th, c))
            T_65 = self.AH(6, th, c)
            T_54 = self.AH(5, th, c)
            T_14 = (T_10 * self.target_pos) * linalg.inv(T_54 * T_65)
            P_13 = T_14 * mat([0, -self.D[3], 0, 1]).T - mat([0, 0, 0, 1]).T
            t3 = cmath.acos((linalg.norm(P_13)**2 - self.A[1]**2 -
                            self.A[2]**2)/(2 * self.A[1] * self.A[2]))  # norm ?
            th[2, c] = t3.real
            th[2, c+1] = -t3.real

        # **** theta2 and theta 4 ****

        cl = [0, 1, 2, 3, 4, 5, 6, 7]
        for i in range(0, len(cl)):
            c = cl[i]
            T_10 = linalg.inv(self.AH(1, th, c))
            T_65 = linalg.inv(self.AH(6, th, c))
            T_54 = linalg.inv(self.AH(5, th, c))
            T_14 = (T_10 * self.target_pos) * T_65 * T_54
            P_13 = T_14 * mat([0, -self.D[3], 0, 1]).T - mat([0, 0, 0, 1]).T

            # theta 2
            th[1, c] = -atan2(P_13[1], -P_13[0]) + \
                asin(self.A[2] * sin(th[2, c])/linalg.norm(P_13))
            # theta 4
            T_32 = linalg.inv(self.AH(3, th, c))
            T_21 = linalg.inv(self.AH(2, th, c))
            T_34 = T_32 * T_21 * T_14
            th[3, c] = atan2(T_34[1, 0], T_34[0, 0])
        th = th.real

        #  print Inverse Kinematics Solutions
        print("Inverse Kinematics Solutions:")
        for i in range(th.shape[1]):
            print(f"Solution {i + 1}: {th[:, i]}")

        return th
    
    def flow(self):
        inverse_kinematics_result = self.invKine()
        print("inverse_kinematics_result",inverse_kinematics_result)
        self.HTrans(inverse_kinematics_result,6)
    
    def run(self):
        rate = rospy.Rate(20)
        self.flow()
        while not rospy.is_shutdown():
            self.flow()
            rate.sleep()


def main():
    node = RobotKinematics()
    node.run()


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        exit(0)

