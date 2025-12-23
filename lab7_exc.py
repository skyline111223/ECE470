#!/usr/bin/env python

'''

lab7pkg_pick_place/lab7_exec.py

@brief: main file to execute the pick and place task

@author: Songjie Xiao
@date: Monday 2023/03/23

'''

# ckcamera api
import sys
sys.path.append("../utils/script/")
#from ckcamera import *

from lab7_func import *
from lab7_ur3e import *
from lab7_img import *

def main():

	# Initialize ROS node
	rospy.init_node('lab7_node')

	# Initialize CKCamera
	# camera = CKCamera()
	# camera.init()

	# # 1. save first calibration image, enter 's' to save image with an input of image_name
	# camera.display()
	# # 2. Free move the robot to obtain the robot coordinate (x,y) of first block
	# # TODO: get input of robot coordinate (x,y) of first block
	center_robot1 = (-458.50e-3,-101.68e-3)
	# # 3. save second calibration image, enter 's' to save image with an input of image_name
	# camera.display()
	# # 4. Free move the robot to obtain the robot coordinate (x,y) of second block
	# # TODO: get input of robot coordinate (x,y) of second block
	center_robot2 = (-364.52e-3,-230.26e-3)
	# # 5. save snapshot image, randomly place at least 5 blocks
	# camera.display()

	# # uninit CKCamera
	# camera.uninit()

	# TODO: complete the coordinate transformation function in lab7_img.py
	# 6. do Image Processing and coordinate transformation to obtain the robot coordinate (x,y) of each block
	center_values, shape, theta = lab_imgproc(center_robot1, center_robot2)
	
	############## Your Code Start Here ############## 	
	# TODO: main execution of pick and place task

	# initialize UR3e functions
	ur3e = UR3e()
	print("Please Press 'Start' button on the Teach Pendant to continue ...")

	# wait for the 'Start' button on the Teach Pendant
	time.sleep(2)

	# call inverse kinematics to compute joint values according to robot coordinate
	# z values is assigned by users
	high_dest_position = [-311e-3,-182.9e-3, 177e-3]
	rect_dest_position = [236.8e-3,-199.95e-3,100e-3]
	oval_dest_position = [325.45e-3, -129.12e-3, 100e-3]


	dest_between = lab_invk(high_dest_position[0], high_dest_position[1], high_dest_position[2], 0)
	

	before_fetch_height = 100e-3
	fetch_height = 68.35e-3
	
	# dest_pos = lab_invk(x, y, z, theta)

	# do the move_arm and gripper operation to pick and p# if not working, move yaw from herelace the block

	# # first stack
	r_counter = 0
	o_counter = 0

	# move to somewhere in between
	ur3e.move_arm(dest_between)


	time.sleep(2)

	# '''
	for i in range(len(shape)):
		# first get the block from original place
		orig_position = center_values[i]
		block_shape = shape[i]
		block_yaw = theta[i]
		print("Center value:", orig_position)
		# print("block_yaw:", block_yaw)
		# print("Center value:", orig_position)
		# print("Center value:", orig_position)
		# go to a higher place
		dest_pos1 = lab_invk(orig_position[0], orig_position[1], before_fetch_height, 0)
		ur3e.move_arm(dest_pos1)
		# grip
		# dest_pos2 = lab_invk(orig_position[0], orig_position[1], fetch_height, 0)					# if not working, move yaw from here
		dest_pos2 = lab_invk(orig_position[0], orig_position[1], fetch_height,0)					# if not working, move yaw from here

		ur3e.move_arm(dest_pos2)
		ur3e.gripper(True)

		dest_pos3 = lab_invk(orig_position[0], orig_position[1], before_fetch_height, -block_yaw)					# if not working, move yaw from here
		ur3e.move_arm(dest_pos3)

		# move to somewhere in between
		# dest_between = lab_invk(high_dest_position[0], high_dest_position[1], high_dest_position[2], 0)
		# ur3e.move_arm(dest_between)

		#release
		if(block_shape == 0):
			if (r_counter == 0):
				theta0_orig_r = dest_pos3[0]
			release_dest = lab_invk(rect_dest_position[0], rect_dest_position[1], rect_dest_position[2], -block_yaw)  # if not working, move yaw to here
			# release_dest = lab_invk(rect_dest_position[0], rect_dest_position[1], rect_dest_position[2], block_yaw)  # if not working, move yaw to here
			# release_dest = lab_invk(rect_dest_position[0], rect_dest_position[1], rect_dest_position[2], 0)  # if not working, move yaw to here
			release_dest[5] -= dest_pos3[0] - theta0_orig_r
			ur3e.move_arm(release_dest)
			ur3e.gripper(False)
			r_counter += 1
		else:
			if (o_counter == 0):
				theta0_orig_o = dest_pos3[0]
			# release_dest = lab_invk(oval_dest_position[0], oval_dest_position[1], oval_dest_position[2], block_yaw)  # if not working, move yaw to here
			release_dest = lab_invk(oval_dest_position[0], oval_dest_position[1], oval_dest_position[2], -block_yaw)  # if not working, move yaw to here
			release_dest[5] -= dest_pos3[0] - theta0_orig_o
			ur3e.move_arm(release_dest)
			time.sleep(2)
			ur3e.gripper(False)
			o_counter += 1

		# move to somewhere in between
		ur3e.move_arm(dest_between)
	# '''

	# pass

	############### Your Code End Here ###############

	rospy.loginfo("Finish Pick and place task!")

if __name__ == '__main__':
	
	try:
		main()
    # When Ctrl+C is executed, it catches the exception
	except rospy.ROSInterruptException:
		pass
