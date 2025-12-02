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
	M = np.eye(4)
	S = np.zeros((6,6))
	pos_1 = np.array([[0.0],[0.0],[0.152]])
	pos_2 = np.array([[0.12],[0.0],[0.152]])
	pos_3 = np.array([[0.12],[-0.244],[0.152]])
	pos_4 = np.array([[0.027],[-0.457],[0.152]])
	pos_5 = np.array([[0.131],[-0.457],[0.152]])
	pos_6 = np.array([[0.131],[-0.542],[0.152]])
	P = np.array([[0.293],[-0.542],[0.152]])
	R = np.array([[0,0,1],[0,1,0],[-1,0,0]])
	M = np.hstack([R, P])
	M = np.vstack([M,np.array([0.0,0.0,0.0,1.0])])
	#print(M)
	joint_positions = []
	joint_positions.append(pos_1)
	joint_positions.append(pos_2)
	joint_positions.append(pos_3)
	joint_positions.append(pos_4)
	joint_positions.append(pos_5)
	joint_positions.append(pos_6)

	q = []
	for i in range(0,len(joint_positions)):
		q.append(joint_positions[i])
	w1 = np.array([[0.0],[0.0],[1.0]])
	w2 = np.array([[1.0],[0.0],[0.0]])
	w3 = np.array([[1.0],[0.0],[0.0]])
	w4 = np.array([[1.0],[0.0],[0.0]])
	w5 = np.array([[0.0],[-1.0],[0.0]])
	w6 = np.array([[1.0],[0.0],[0.0]])

	w = []
	w.append(w1)
	w.append(w2)
	w.append(w3)
	w.append(w4)
	w.append(w5)
	w.append(w6)
	v = []
	for i in range(0,len(joint_positions)):
		v.append(-np.cross(w[i],q[i],axis=0))
	SA = []
	for i in range(0,len(joint_positions)):
		Si = np.vstack([w[i], v[i]])
		SA.append(Si)

	S = np.hstack([SA[0],SA[1],SA[2],SA[3],SA[4],SA[5]])




	
	# ==============================================================#
	return M, S


"""
Function that calculates encoder numbers for each motor
"""
def lab_fk(theta1, theta2, theta3, theta4, theta5, theta6):

	# =========== Implement joint angle to encoder expressions here ===========
	print("Forward kinematics calculated:\n")

	# =================== Your code starts here ====================#
	theta = np.array([theta1,theta2,theta3,theta4,theta5,theta6])
	T = np.eye(4)
	M, S = Get_MS()
	for i in range(6):
		
		omega = S[:3, i]
		v = S[3:, i]
		S_i = np.array([
            [0, -omega[2], omega[1], v[0]],
            [omega[2], 0, -omega[0], v[1]],
            [-omega[1], omega[0], 0, v[2]],
            [0, 0, 0, 0]
        ])
        
		exp_Si = expm(S_i * theta[i])
        
		T = np.dot(T, exp_Si)
	T = np.dot(T, M)

	








	# ==============================================================#
	
	print(str(T) + "\n")

	return T


"""
Function that calculates an elbow up Inverse Kinematic solution for the UR3
"""
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
	l10 = 0.07
    
	yaw = yaw_WgripDegree * PI / 180

	xgrip = -yWgrip 
	ygrip = xWgrip 
	zgrip = zWgrip 

	xcen = xgrip - l09*math.cos(yaw)
	ycen = ygrip - l09*math.sin(yaw)
	zcen = zgrip

	thetaA = np.arctan2(ycen,xcen)
	l = np.sqrt(xcen**2+ycen**2)
	thetaAAA = np.arcsin((l02-l04+l06)/l)
	thetas[0] = thetaA - thetaAAA 
	thetas[5] = - yaw + thetas[0]

	l67 = np.sqrt(l07**2+(l06+0.027)**2)
	theta67 = np.arctan2(l07,(l06+0.027))
 
	x3end = xcen - l67*np.sin(theta67-thetas[0])
	y3end = ycen - l67*np.cos(theta67-thetas[0])
	z3end = zcen + l10 + l08

	xy3end = np.sqrt(x3end**2 + y3end**2)
	z = z3end - l01
	xyz3end = np.sqrt(xy3end**2 + z**2)

	theta21 = np.arctan2(z,xy3end)
	theta41 = PI/2 - theta21
	theta22 = np.arccos((l03**2 + xyz3end**2 - l05**2) / (2*l03*xyz3end))
	theta42 = np.arccos((l05**2 + xyz3end**2 - l03**2) / (2*l05*xyz3end))
	thetas[1]= - (theta21 + theta22)     
	thetas[2]= theta22 + theta42            
	thetas[3]= - (theta41 + theta42 - PI/2)-PI / 2.0
	thetas[4]= - PI/2
	thetas[0]=thetas[0] +PI/2 
	print("theta1 to theta6: " + str(thetas) + "\n")

	return thetas


