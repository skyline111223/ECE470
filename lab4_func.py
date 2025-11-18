#!/usr/bin/env python

'''

lab4pkg_fk/lab4_func.py

@brief: functions for computing forward kinematics using Product of Exponential (PoE) method
@author: Songjie Xiao
@date: Monday 2023/3/20

'''

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
	M = np.array([
		[0, 0, 1, 0.120 - 0.093 + 0.104 + 0.092 + 0.07],
		[0, 1, 0, -0.542],
		[-1, 0, 0, 0.152],
		[0, 0, 0, 1],
	])

	S = np.zeros((6, 6))

	w1 = np.array([0, 0, 1])
	w2 = np.array([1, 0, 0])
	w3 = np.array([1, 0, 0])
	w4 = np.array([1, 0, 0])
	w5 = np.array([0, -1, 0])
	w6 = np.array([1, 0, 0])

	q1 = np.array([0.120,                0,             0.152])
	q2 = np.array([0.120,                0,             0.152])
	q3 = np.array([0.120,            -0.244,           0.152])
	q4 = np.array([0.120 - 0.093,    -0.244 - 0.213,   0.152])
	q5 = np.array([0.120 - 0.093 + 0.104, -0.244 - 0.213, 0.152])
	q6 = np.array([0.120 - 0.093 + 0.104,     -0.542,     0.152])

	S[0] = np.hstack((w1, -np.cross(w1, q1)))
	S[1] = np.hstack((w2, -np.cross(w2, q2)))
	S[2] = np.hstack((w3, -np.cross(w3, q3)))
	S[3] = np.hstack((w4, -np.cross(w4, q4)))
	S[4] = np.hstack((w5, -np.cross(w5, q5)))
	S[5] = np.hstack((w6, -np.cross(w6, q6)))

	# ==============================================================#
	return M, S



"""
Function that calculates encoder numbers for each motor
"""
def lab_fk(theta1, theta2, theta3, theta4, theta5, theta6):

	# =========== Implement joint angle to encoder expressions here ===========
	print("Forward kinematics calculated:\n")

	# =================== Your code starts here ====================#
	
	theta1 -= np.pi/ 2
	theta4 += np.pi / 2
	
	M, S = Get_MS()
	T = compute_transformation(S, [theta1, theta2, theta3, theta4, theta5, theta6]) @ M
	# ==============================================================#
	
	print(str(T) + "\n")
	return T

def compute_transformation(S, theta):
	T = np.eye(4)

	for i in range(6):
		# Extract the screw axis for joint i
		screw = S[i].T
		omega = screw[:3]     # rotational part (ω)
		v = screw[3:]         # translational part (v)

		# Build the 4×4 twist hat matrix [S_i]
		S_hat = np.array([
			[0,        -omega[2],  omega[1],   v[0]],
			[omega[2],  0,        -omega[0],   v[1]],
			[-omega[1], omega[0],  0,          v[2]],
			[0,         0,         0,          0   ],
		])

		# Multiply into the total transform
		T = T @ expm(S_hat * theta[i])
	return T
