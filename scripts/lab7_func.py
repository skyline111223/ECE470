#!/usr/bin/env python


import numpy as np
import math
from scipy.linalg import expm

PI = np.pi

"""
You may write some helper functions as you need
Use 'expm' for matrix exponential
Angles are in radian, distance are in meters.
"""

def Get_MS():
	# =================== Your code starts here ====================#
	# Fill in the correct values for S1~6, as well as the M matrix
    S = np.zeros((6,6))
    M = np.eye(4) 
    
    M[0][0] = 0
    M[0][2] = 1 #f0 x axis in f_end
    M[0][3] = (120-93+104+92+70)/1000 #M_Px ?70
    M[1][3] = (-542)/1000 #M_Py
    M[2][0] = -1 #f0 z axis in f_end
    M[2][2] = 0
    M[2][3] = 152/1000
    
    q1 = np.array([0, 0, 152])/1000
    q2 = np.array([120, 0, 152])/1000
    q3 = np.array([120, -244, 152])/1000   
    q4 = np.array([120-93, -244-213, 152])/1000
    q5 = np.array([120-93+104, -244-213, 152])/1000
    q6 = np.array([120-93+104, -542, 152])/1000  
    
    w1= np.array([0,0,1])
    w2= np.array([1,0,0])    
    w3= np.array([1,0,0])  
    w4= np.array([1,0,0])
    w5= np.array([0,-1,0])
    w6= np.array([1,0,0])     
    
    v1 = -np.cross(w1,q1)
    v2 = -np.cross(w2,q2)
    v3 = -np.cross(w3,q3)
    v4 = -np.cross(w4,q4)
    v5 = -np.cross(w5,q5)
    v6 = -np.cross(w6,q6)
    
    S[:, 0] = np.concatenate((w1, v1))
    S[:, 1] = np.concatenate((w2, v2))
    S[:, 2] = np.concatenate((w3, v3))
    S[:, 3] = np.concatenate((w4, v4))
    S[:, 4] = np.concatenate((w5, v5))
    S[:, 5] = np.concatenate((w6, v6))


	# ==============================================================#
    return M, S


def screw_matrix(screw_axis):
    omega = screw_axis[:3]  # Angular velocity
    v = screw_axis[3:]      # Linear velocity
    omega_hat = np.array([[0, -omega[2], omega[1]],
                          [omega[2], 0, -omega[0]],
                          [-omega[1], omega[0], 0]])
    s = np.zeros((4, 4))
    s[:3, :3] = omega_hat
    s[:3, 3] = v
    return s

"""
Function that calculates encoder numbers for each motor
"""

def lab_fk(theta1, theta2, theta3, theta4, theta5, theta6):
    print("Forward kinematics calculated:\n")
    M,S = Get_MS()
    
    t=np.eye(4)
    theta = np.array([theta1 - PI/2, theta2, theta3, theta4 + PI/2, theta5, theta6])

    for i in range(len(theta)):
        ScrewM=screw_matrix(S[:, i])
        exp_theta=expm( theta[i] * ScrewM)
        t = np.dot(t, exp_theta) 

    T=np.dot(t,M)
    
    print(str(T)+"\n")
    return T

"""
Function that calculates an elbow up Inverse Kinematic solution for the UR3 """
def lab_invk(xWgrip, yWgrip, zWgrip, yaw_WgripDegree):
    # theta1 to theta6
	thetas = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
	
	l01 = 0.152
	l02 = 0.120
	l03 = 0.244
	l04 = 0.093
	l05 = 0.213
	l06 = 0.104
	l07 = 0.085
	l08 = 0.092
	l09 = 0
	l10 = 0.000

	# yawRad = yaw_WgripDegree * PI / 180
	xgrip = xWgrip
	ygrip = yWgrip
	zgrip = zWgrip
	xcen = xWgrip
	ycen = yWgrip
	zcen = zWgrip
		# theta1
	theta0 = np.arctan2(xcen,-ycen) - np.arcsin((l02-l04+l06)/np.sqrt(xcen ** 2 + ycen ** 2))       
	thetas[0] = theta0 + PI/2
	
	# theta6
	thetas[5] = yaw_WgripDegree
 
	x3end = xcen - np.sin(theta0) * l07 - np.cos(theta0) * (l02 - l04 + l06) 
	y3end = ycen + np.cos(theta0) * l07 - np.sin(theta0) * (l02 - l04 + l06) 
	z3end = zcen + l10 + l08
 
	thetas[1]= - np.arctan2(z3end - l01, np.sqrt(x3end ** 2 + y3end ** 2) ) - np.arccos(((x3end ** 2 + y3end ** 2 + (z3end - l01)**2) + l03**2 - l05**2)/(2*np.sqrt(x3end ** 2 + y3end ** 2 + (z3end - l01)**2)*l03)) 
	thetas[2]= PI - np.arccos((-(x3end ** 2 + y3end ** 2 + (z3end - l01)**2) + l03**2 + l05**2)/(2*l05*l03))
	thetas[3]= - PI/2 - thetas[1] - thetas[2]
	thetas[4]=-PI/2 
	
	print("theta1 to theta6: " + str(thetas) + "\n")
	# thetas = np.radians(thetas)
	return thetas