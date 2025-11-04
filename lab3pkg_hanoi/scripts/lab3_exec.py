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

	# Definition of our tower#!/usr/bin/env python

'''

lab3pkg_hanoi/lab3_ur3e.py

@brief: UR3e class include functions for controlling the UR3e arm and the gripper

@author: Songjie Xiao
@date: Monday 2023/03/17

'''
import time
import rospy

# IMPORT! headers for ROS messages and include useful message types
from lab3_header import *

class UR3e():

    def __init__(self):

        # define variables here
        # store the current position of the arm
        self.current_position = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        # store the current state of the suction cup
        self.current_io_0 = False

        # Initialize publisher for ur3e_driver_ece470/setjoint with buffer size of 10
        self.pub_setjoint = rospy.Publisher('ur3e_driver_ece470/setjoint', JointTrajectory, queue_size=10)

        # TODO: define a ROS publisher for /ur3e_driver_ece470/setio message with buffer size of 10
        self.pub_setio = rospy.Publisher('ur3e_driver_ece470/setio', Digital, queue_size=10)

        # Initialize subscriber to /joint_states, each time /joint_states publishes a new message, the function position_callback is called
        self.sub_position = rospy.Subscriber('/joint_states', JointState, self.position_callback)

        # TODO: define a ROS subscriber for /ur_hardware_interface/io_states message and corresponding callback function
        self.sub_io = rospy.Subscriber('/ur_hardware_interface/io_states', IOStates, self.gripper_input_callback)
    
    def init_array(self, home, Q):

        # initialize the home position and Q array
        self.home = home
        self.Q = Q

    def gripper_input_callback(self, msg):
        """
        TODO: define a ROS topic callback function for getting the state of suction cup
        Whenever /ur_hardware_interface/io_states publishes info, this callback function is called.
        """
        ############## Your Code Start Here ##############

        self.current_io_0 = msg.flag_states
        
                


        ############## Your Code End Here ##############


    def position_callback(self, msg):
        """
        TODO: define a ROS topic callback function for getting the current position of the arm
        Whenever /joint_states publishes info, this callback function is called.
        """
        ############## Your Code Start Here ##############
        self.thetas = list(msg.position)
        self.current_position = self.thetas
        self.current_position_set = True
	


        ############### Your Code End Here ###############


    def gripper(self, io_0):
        """
        TODO: define a function for ROS Publisher to publish your message to the Topic "ur3e_driver_ece470/setio",
        so that we can control the state of suction cup.
        """
        ############## Your Code Start Here ##############
        msg = Digital()
        msg.pin = 0
        msg.state = io_0
        self.pub_setio.publish(msg)
        time.sleep(0.5)
        

        ############### Your Code End Here ###############


    def move_arm(self, dest):
        """
        Function for moving the arm to a desired location
        """
        # define msg
        msg = JointTrajectory()
        msg.joint_names = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]
        point = JointTrajectoryPoint()
        point.positions = dest
        point.time_from_start = rospy.Duration(2)
        msg.points.append(point)
        # publish msg
        self.pub_setjoint.publish(msg)
        time.sleep(2.5)


    def move_block(self, start_loc, start_height, end_loc, end_height):

        """
        TODO: define move_block function which is used to move a block from start to end location
        which includes moving the arm to the start location, gripping the block, moving the arm to the end location, and releasing the block

        Hint: Use the Q array to map out your towers by location and height.
        """
        ############## Your Code Start Here ##############
        self.gripper(False)
        time.sleep(0.5)

        self.move_arm(self.home)
        time.sleep(0.5)

        self.move_arm(self.Q[start_height][start_loc])
        time.sleep(0.5)

        self.gripper(True)
        time.sleep(0.5)

        self.move_arm(self.home)
        time.sleep(0.5)

        self.move_arm(self.Q[end_height][end_loc])
        time.sleep(0.5)

        self.gripper(False)
        time.sleep(0.5)

        ############### Your Code End Here ###############







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

	home = np.radians([64.31,-71.89,39.32,-58.71,-89.04,244.52])

    # Example contact joint angles for each tower/level
    # !!! Replace these with your calibrated joint angles !!!
	Q = [
        [  # Top layer
            np.radians([79.07,-61.77,105.74,-135.24,-88.26,246.63]),  # Tower 1 top
            np.radians([67.42, -61.92, 105.84, -134.72, -91.37, 237.90]),    # Tower 2 top
            np.radians([54.52, -59.69, 101.52, -131.97, -89.19, 242.19]),   # Tower 3 top
        ],
        [  # Middle layer
            np.radians([79.49, -57.91, 106.57, -139.92, -89.02, 246.13]),  # Tower 1 mid
            np.radians([66.58, -58.50,106.87,-138.25,-90.02,242.30,]),    # Tower 2 mid
            np.radians([55.40, -55.95, 102.25, -136.58, -91.33, 242.21]),   # Tower 3 mid
        ],
        [  # Bottom layer
            np.radians([79.94,-53.97,107.07,-144.37,-90.04,244.65]),  # Tower 1 bottom
            np.radians([66.88, -54.46, 108.20, -144.78, -90.23, 242.15]),    # Tower 2 bottom
            np.radians([54.69, -52.25, 101.88, -138.36, -89.44, 240.27]),   # Tower 3 bottom
        ],
    ]

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

    # Example contact joint angles for each tower/level
    # !!! Replace these with your calibrated joint angles !!!

	while not input_done:
		input_string1 = input("Enter number of loops <Either 1 2 3 or 0 to quit> ")
		print("You entered " + input_string1 + "\n")


		if int(input_string1) == 1:
			input_done = 1
			#loop_count = 1
			des = 0
		elif int(input_string1) == 2:
			input_done = 1
			#loop_count = 2
			des = 1
		elif int(input_string1) == 3:
			input_done = 1
			#loop_count = 3
			des = 2
		elif int(input_string1) == 0:
			print("Quitting... ")
			# sys.exit()
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
