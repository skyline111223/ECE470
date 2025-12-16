#!/usr/bin/env python

 

'''

 

lab5pkg_ik/lab5_func.py

 

@brief: functions for computing forward and inverse kinematics of UR3e robot arm

@author: Songjie Xiao

@date: Monday 2023/3/20

 

'''

 

import numpy as np

import math

from scipy.linalg import expm

 

PI = np.pi

 

"""

Use 'expm' for matrix exponential.

Angles are in radian, distance are in meters.

Add any helper functions as you need.

"""

def Get_MS():

       # =================== Your code starts here ====================#

       # Fill in the correct values for S1~6, as well as the M matrix

       # M = np.eye(4)

       # S = np.zeros((6,6))

       M = np.array([[0, 0, 1, 0.120 - 0.093 + 0.104 + 0.092 + 0.07],
                    [0, 1, 0, -0.542],
                    [-1, 0, 0, 0.152],
                    [0, 0, 0, 1]])

       S = np.zeros((6,6))

       w1 = np.array([0, 0, 1])
       w2 = np.array([1, 0, 0])
       w3 = np.array([1, 0, 0])
       w4 = np.array([1, 0, 0])
       w5 = np.array([0, -1, 0])
       w6 = np.array([1, 0, 0])      

       q1 = np.array([0, 0, 0.152])
       q2 = np.array([0.120, 0, 0.152])
       q3 = np.array([0.120, -0.244, 0.152])
       q4 = np.array([0.120 - 0.093, -0.244 - 0.213, 0.152])
       q5 = np.array([0.120 - 0.093 + 0.104, - 0.244 - 0.213, 0.152])
       q6 = np.array([0.120 - 0.093 + 0.104, - 0.542, 0.152])

       S[0] = np.hstack((w1, - np.cross(w1,q1)))
       S[1] = np.hstack((w2, - np.cross(w2, q2)))
       S[2] = np.hstack((w3, - np.cross(w3, q3)))
       S[3] = np.hstack((w4, - np.cross(w4, q4)))
       S[4] = np.hstack((w5, - np.cross(w5, q5)))
       S[5] = np.hstack((w6, - np.cross(w6, q6)))

       # ==============================================================#

       return M, S

 

 

"""

Function that calculates encoder numbers for each motor

"""

def lab_fk(theta1, theta2, theta3, theta4, theta5, theta6): 

       # =========== Implement joint angle to encoder expressions here ===========

       print("Forward kinematics calculated:\n")

 

       # =================== Your code starts here ====================#

       theta1-=PI/2
       theta4+=PI/2
       theta = np.array([theta1,theta2,theta3,theta4,theta5,theta6])

       M, S = Get_MS()
       
       T=compute_transformation(S,theta)@M

       # ==============================================================#

       print(str(T) + "\n")
       return T


def compute_transformation(S, theta):
       T = np.eye(4)
       
       for i in range(6):
              S_i = S[i].T
              r,v = S_i[:3], S_i[3:]
       
              S_bracket = np.array([
              [0, -r[2], r[1], v[0]],
              [r[2], 0, -r[0], v[1]],
              [-r[1], r[0], 0, v[2]],
              [0, 0, 0, 0]
              ])

              T=T@expm(S_bracket*theta[i])
       
       return T


"""

Function that calculates an elbow up Inverse Kinematic solution for the UR3

"""

def lab_invk(xWgrip, yWgrip, zWgrip, yaw_WgripDegree):

 

       theta = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

 

       L1 = 0.152
       L2 = 0.120
       L3 = 0.244
       L4 = 0.093
       L5 = 0.213
       L6 = 0.104
       L7 = 0.085
       L8 = 0.092
       L9 = 0
       L10 = 0.07
       D = 0.027

       xgrip = -yWgrip
       ygrip = xWgrip
       zgrip = zWgrip

       yaw_WgripRad = np.deg2rad(yaw_WgripDegree)
       xcen = xgrip - L9 * math.cos(yaw_WgripRad)
       ycen = ygrip - L9 * math.sin(yaw_WgripRad)
       zcen = zgrip

       theta[0] = math.atan2(ycen, xcen) - math.asin((D + L6) / (xcen ** 2 + ycen ** 2) ** 0.5)
       theta[5] = PI - yaw_WgripRad - (PI / 2 -  theta[0])

       x3end = xcen - L7 * math.cos(theta[0]) + (L6 + D) * math.sin(theta[0])
       y3end = ycen - L7 * math.sin(theta[0]) - (L6 + D) * math.cos(theta[0])
       z3end = zcen + L10 + L8

       D24 = (x3end ** 2 + y3end ** 2 + (z3end - L1) **2) ** 0.5
       theta[1] = - math.acos((L3 ** 2 + D24 ** 2 - L5 ** 2) / (2 * L3 * D24)) - math.atan2((z3end - L1), (x3end ** 2 + y3end ** 2) ** 0.5)  
       theta[2] = PI - math.acos((L3 ** 2 + L5 ** 2 - D24 ** 2) / (2 * L3 * L5))   
       theta[3] = - theta[1] - theta[2] - PI / 2
       theta[4] = - PI / 2     
       theta[0] += PI/2
       print("theta1 to theta6: " + str(theta) + "\n")

       return theta
