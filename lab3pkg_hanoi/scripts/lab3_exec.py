#!/usr/bin/env python

'''

lab3pkg_hanoi/lab3_exec.py

@brief: Hanoi implementation in ROS.
@author: Songjie Xiao
@date: Monday 2023/3/20

'''

import sys
import time
import rospy
import numpy as np

from lab3_ur3e import UR3e

def main():

	# Initialize ROS node
	rospy.init_node('lab3_node')

	# Definition of our tower

	# block contact position
	# | Q[0][0] Q[0][1] Q[0][2] |   Contact point of top block
	# | Q[1][0] Q[1][1] Q[1][2] |   Contact point of middle block
	# | Q[2][0] Q[2][1] Q[2][2] |   Contact point of bottom block
	# | Tower1  Tower2  Tower3  |

	# First index - From "top" to "bottom"
	# Second index - From "left" to "right"

	# How the arm will move (Suggestions)
	# 1. Go to the "home" position
	# 2. Drop to the "contact (start) block" position
	# 3. Rise back to the "home" position
	# 4. Drop to the corresponding "contact (end) block" position
	# 5. Rise back to the "home" position

	"""
	TODO: define position of our tower in Q array and home position of the arm
	"""
	############## Your Code Start Here ##############

	home = np.radians([69.53,-80.59 ,86.32 ,-95.81 ,-89.62 ,16.34 ])
	
	Q = np.zeros((3,3,6))
	Q[0][0] = np.radians([57.52,-62.22 ,104.62 , -132.49,-89.76 ,127.22 ])
	Q[1][0] = np.radians([57.51,-58.55 ,106.09 ,-137.63 ,-89.79 ,127.28 ])
	Q[2][0] = np.radians([57.51, -54.54,107.06 ,-142.61 ,-89.82 ,127.29 ])
	Q[0][1] = np.radians([69.73,-63.86 ,107.27,-133.50 ,-89.77 ,139.49 ])
	Q[1][1] = np.radians([69.73,-59.87 ,108.84 ,-139.05 ,-89.80 ,139.50 ])
	Q[2][1] = np.radians([69.73,-56.20 ,109.72 ,-143.61 ,-89.82 ,139.51 ])
	Q[0][2] = np.radians([83.42,-61.71 ,103.83 ,-132.20 ,-89.76 ,153.17 ])
	Q[1][2] = np.radians([83.41,-58.05 ,105.30 ,-137.33 ,-89.79 ,153.18 ])
	Q[2][2] = np.radians([83.41, -54.28,106.22 ,-142.03 ,-89.81 ,153.19 ])

	############### Your Code End Here ###############

	# This program will require two user inputs to specify the start location and end location of the block
	# TODO: modify the code below so that program can get two user inputs
	############## Your Code Start Here ##############

	# example code for getting user input is provided below
	input_done = 0
	#loop_count = 0
	start = 0
	mid = 1
	des = 2

	while not input_done:
		input_string1 = input("Enter number of loops <Either 1 2 3 or 0 to quit> ")
		print("You entered " + input_string1 + "\n")

		if int(input_string1) == 1:
			input_done = 1
			#loop_count = 1
			start = 0
		elif int(input_string1) == 2:
			input_done = 1
			#loop_count = 2
			start = 1
		elif int(input_string1) == 3:
			input_done = 1
			#loop_count = 3
			start = 2
		elif int(input_string1) == 0:
			print("Quitting... ")
			 #sys.exit()
		else:
			print("Please just enter the character 1 2 3 or 0 to quit \n\n")

		input_string2 = input("Enter number of loops <Either 1 2 3 or 0 to quit> ")
		print("You entered " + input_string2 + "\n")

		if int(input_string2) == 1:
			input_done = 1
			#loop_count = 1
			des = 0
		elif int(input_string2) == 2:
			input_done = 1
			#loop_count = 2
			des = 1
		elif int(input_string2) == 3:
			input_done = 1
			#loop_count = 3
			des = 2
		elif int(input_string2) == 0:
			print("Quitting... ")
			sys.exit()
		else:
			print("Please just enter the character 1 2 3 or 0 to quit \n\n")
	
	if (start == 0 and des == 1) or (start == 1 and des == 0):
		mid = 2
	elif (start == 1 and des == 2) or (start == 2 and des == 1):
		mid = 0
	else:
		mid = 1
			
	############### Your Code End Here ###############

	rospy.loginfo("Sending Goals ...")

	# Main manipulation code defined here
	# move_arm function is used to move the arm to a desired position
	# move_block function is used to move a block from start to end location
	# which includes moving the arm to the start location, gripping the block, moving the arm to the end location, and releasing the block
	# TODO: here to define a series of move_block or move_arm function calls to solve the Hanoi Tower problem
	############## Your Code Start Here ##############

	# initialize ur3e class
	ur3e = UR3e()
	ur3e.init_array(home, Q)
	time.sleep(2)

	# Solve Hanoi Tower
	ur3e.move_block(start, 0, des, 2)
	ur3e.move_block(start, 1, mid, 2)
	ur3e.move_block(des, 2, mid, 1)
	ur3e.move_block(start, 2, des, 2)
	ur3e.move_block(mid, 1, start, 2)
	ur3e.move_block(mid, 2, des, 1)
	ur3e.move_block(start, 2, des, 0)
	# Move back to home position
	ur3e.move_arm(home)

	############### Your Code End Here ###############

if __name__ == '__main__':
	try:
		main()
	# When Ctrl+C is executed, it catches the exception
	except rospy.ROSInterruptException:
		pass
