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
    """Compute forward kinematics using Product of Exponentials."""

    T = np.eye(4)

    for i in range(6):
        w = S[i, :3]        # angular part
        v = S[i, 3:]        # linear part

        # Construct se(3) bracket
        S_bracket = np.array([
            [0,     -w[2],  w[1],  v[0]],
            [w[2],   0,    -w[0],  v[1]],
            [-w[1],  w[0],  0,     v[2]],
            [0,      0,     0,     0    ]
        ])

        T = T @ expm(S_bracket * theta[i])

    return T


"""

Function that calculates an elbow up Inverse Kinematic solution for the UR3

"""

def lab_invk(xWgrip, yWgrip, zWgrip, yaw_WgripDegree):

    """
    Compute an elbow-up inverse kinematics solution for UR3e.
    Input is given in the WORLD frame (ECE470 Lab 5 specification).
    Output joint angles are in radians.
    """

    # Initialize joint angle array
    theta = [0.0] * 6

    # ---------------------- Link lengths (meters) ----------------------
    L1 = 0.152
    L2 = 0.120
    L3 = 0.244
    L4 = 0.093
    L5 = 0.213
    L6 = 0.104
    L7 = 0.085
    L8 = 0.092
    L9 = 0.0535   # Suction cup aluminum plate length
    L10 = 0.07
    D = 0.027     # Horizontal offset between joint1 axis and link6 axis

    # ---------------------- 1) World frame → Base frame ----------------------
    # Lab 5 coordinate definition: base frame is rotated 90° from world frame
    x_grip = -yWgrip
    y_grip =  xWgrip
    z_grip =  zWgrip

    # ---------------------- 2) Wrist center computation ----------------------
    yaw = np.deg2rad(yaw_WgripDegree)

    x_cen = x_grip - L9 * math.cos(yaw)
    y_cen = y_grip - L9 * math.sin(yaw)
    z_cen = z_grip

    r_cen = math.hypot(x_cen, y_cen)

    # ---------------------- 3) Joint 1 (θ1) and Joint 6 (θ6) ----------------------
    # θ1 aims toward the wrist center but must compensate lateral offset (D + L6)
    theta1_base = math.atan2(y_cen, x_cen)
    offset = math.asin((D + L6) / r_cen)
    theta[0] = theta1_base - offset

    # θ6 compensates for global yaw after considering θ1
    theta[5] = PI - yaw - (PI/2.0 - theta[0])

    # ---------------------- 4) Compute projected point (x3end, y3end, z3end) ----------------------
    # This point lies on the link-3 → link-5 plane (Fig. 5.2)
    cos_t1 = math.cos(theta[0])
    sin_t1 = math.sin(theta[0])

    x_3end = x_cen - L7 * cos_t1 + (L6 + D) * sin_t1
    y_3end = y_cen - L7 * sin_t1 - (L6 + D) * cos_t1
    z_3end = z_cen + L10 + L8

    # ---------------------- 5) Solve θ2, θ3, θ4 via 2R geometry ----------------------
    # Distance from joint2 to projection point
    r_24 = math.hypot(x_3end, y_3end)
    dz_24 = z_3end - L1
    D24 = math.sqrt(r_24**2 + dz_24**2)

    # θ2 (shoulder)
    cos_angle2 = (L3**2 + D24**2 - L5**2) / (2.0 * L3 * D24)
    theta2_part = math.acos(cos_angle2)
    theta2_tilt = math.atan2(dz_24, r_24)
    theta[1] = -theta2_part - theta2_tilt

    # θ3 (elbow) – use PI - acos() to select elbow-up configuration
    cos_angle3 = (L3**2 + L5**2 - D24**2) / (2.0 * L3 * L5)
    theta[2] = PI - math.acos(cos_angle3)

    # θ4 (wrist 1) ensures link7 remains parallel to the table plane
    theta[3] = -theta[1] - theta[2] - PI/2.0

    # θ5 is fixed at −90° per Lab 5 simplification
    theta[4] = -PI/2.0

    # Adjust θ1 to match FK joint offset (same as lab_fk)
    theta[0] += PI/2.0

    print("theta1 to theta6:", theta, "\n")
    return theta
