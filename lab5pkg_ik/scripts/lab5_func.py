#!/usr/bin/env python3
import numpy as np
from scipy.linalg import expm
import math
from lab5_header import *

"""
Use 'expm' for matrix exponential.
Angles are in radian, distance are in meters.
"""
PI = np.pi

def Get_MS():
	# =================== Your code starts here ====================#
	# Fill in the correct values for a1~6 and q1~6, as well as the M matrix
	PI= np.pi

	M = np.identity(4)
	S = np.zeros((6,6))
	J = []
	Q = []
	W = []
	v = []
	s = []

	p_1 = np.array([  [0.0],   [0.0], [0.152]])
	p_2 = np.array([ [0.120],  [0.0], [0.152]])
	p_3 = np.array([ [0.120],[-0.244], [0.152]])
	p_4 = np.array([ [0.027],[-0.457], [0.152]])
	p_5 = np.array([ [0.131],[-0.457], [0.152]])
	p_6 = np.array([ [0.131],[-0.542],  [0.152]])

	J.append(p_1)
	J.append(p_2)
	J.append(p_3)
	J.append(p_4)
	J.append(p_5)
	J.append(p_6)


	P= np.array([[0.293], [-0.542], [0.152]])

	R = np.array([[0, 0, 1],\
			      [0, 1, 0],\
				  [-1, 0,0]])

	M = np.vstack([np.hstack([R, P]) ,np.array([0.0, 0.0, 0.0, 1.0])])


	W1 = np.array([[0.0],[0.0],[1.0]])
	W2 = np.array([[1.0],[0.0],[0.0]])
	W3 = np.array([[1.0],[0.0],[0.0]])
	W4 = np.array([[1.0],[0.0],[0.0]]) 
	W5 = np.array([[0.0],[-1.0],[0.0]])
	W6 = np.array([[1.0],[0.0],[0.0]])

	
	W.append(W1)
	W.append(W2)
	W.append(W3)
	W.append(W4)
	W.append(W5)
	W.append(W6)

	for i in range(0,len(J)):
		Q.append(J[i])
		v.append(-np.cross(W[i],Q[i],axis=0))
		S_i = np.vstack([W[i], v[i]])
		s.append(S_i)

	S = np.hstack([s[0],s[1],s[2],s[3],s[4],s[5]])


	# ==============================================================#
	return M, S


"""
Function that calculates encoder numbers for each motor
"""
def lab_fk(theta1, theta2, theta3, theta4, theta5, theta6):

	# Initialize the return_value
	return_value = [None, None, None, None, None, None]

	print("Foward kinematics calculated:\n")

	# =================== Your code starts here ====================#
	theta = np.array([theta1,theta2,theta3,theta4,theta5,theta6])
	T = np.eye(4)

	M, S = Get_MS()
	S1 = ScrewToBracket(S[0])
	S2 = ScrewToBracket(S[1])
	S3 = ScrewToBracket(S[2])
	S4 = ScrewToBracket(S[3])
	S5 = ScrewToBracket(S[4])
	S6 = ScrewToBracket(S[5])
	T = expm(S1*theta1)@expm(S2*theta2)@expm(S3*theta3)@expm(S4*theta4)@expm(S5*theta5)@expm(S6*theta6)@M

	print(str(T) + "\n")

	return_value[0] = theta1 + PI
	return_value[1] = theta2
	return_value[2] = theta3
	return_value[3] = theta4 - (0.5*PI)
	return_value[4] = theta5
	return_value[5] = theta6

	return return_value


"""
Function that calculates an elbow up Inverse Kinematic solution for the UR3
"""
def lab_invk(xWgrip, yWgrip, zWgrip, yaw_WgripDegree):
	# =================== Your code starts here ====================#

	L1 = 152/1000
	L2 = 120/1000
	L3 = 244/1000
	L4 = 93/1000
	L5 = 213/1000
	L6 = 104/1000
	L7 = 85/1000
	L8 = 92/1000
	L = 70/1000    							# extra length , check
	yaw = yaw_WgripDegree * (PI/180)
	x = -yWgrip
	y = xWgrip 			
	z = zWgrip 			

	x_cen = x #- L * np.cos(yaw)
	y_cen = y #- L * np.sin(yaw)
	z_cen = z  

	alpha = np.arctan2([y_cen],[x_cen])
	
	line = np.sqrt(x_cen**2 + y_cen**2)
	alpha_prime = np.arcsin((L2-L4+L6)/line)
 	
	#line_prime = line*np.cos(alpha_prime) - L7
  
	# theta 1
	theta1 = alpha - alpha_prime + PI/2		# Y
	# theta 5 = pi/2
	theta5 = -np.array([PI/2])				# Fixed
	# theta 6
	theta6 = np.pi/2 + theta1 - yaw		# Y 	

	x_3end = x_cen - L7*np.cos(theta1) + (0.027 + L6)*np.sin(theta1)
	y_3end = y_cen - L7*np.sin(theta1) - (0.027 + L6)*np.cos(theta1)
	z_3end = z_cen + L + L8

	LL = ((z_3end - L1)**2 + x_3end**2 + y_3end**2)**0.5


	theta3 = PI - np.arccos((L3**2 + L5**2 - LL**2)/(2*L3*L5))    ## Y 

	# theta 2 
	theta2 = -np.arcsin(L5*np.sin(theta3)/LL) - np.arcsin((z_3end-L1)/LL)

	# theta 4
	theta4 = -theta2 - theta3   ## Y 
	

	theta11 = theta1 * 180/PI
	theta22 = theta2 * 180/PI
	theta33 = theta3 * 180/PI
	theta44 = theta4 * 180/PI
	theta55 = theta5 * 180/PI
	theta66 = theta6 * 180/PI

	print("Theta1:", theta11)
	print("Theta2:", theta22)
	print("Theta3:", theta33)
	print("Theta4:", theta44)
	print("Theta5:", theta55)
	print("Theta6:", theta66)

 


	# theta1 = 
	# theta2 = 
	# theta3 = 
	# theta4 = 
	# theta5 = -pi/2
	# theta6 = 
	# ==============================================================#
	return lab_fk(theta1, theta2, theta3, theta4, theta5, theta6)


def ScrewToBracket(S):
	W = S[:3]
	V = S[3:]
	W_b = np.array([[0.0, -W[2], W[1]],[W[2], 0.0, -W[0]],[-W[1], W[0], 0.0]])
	S_theta = np.zeros((4,4))
	S_theta[:3, :3] = W_b
	S_theta[:3, 3] = V
	return S_theta