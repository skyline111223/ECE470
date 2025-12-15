#!/usr/bin/env python

'''

lab7pkg_pick_place/lab7_exec.py

@brief: main file to execute the pick and place task

@author: Songjie Xiao
@date: Monday 2023/03/23

'''

'''
# ckcamera api
import sys
sys.path.append("../utils/script/")
from ckcamera import *
'''


from lab7_func import *
from lab7_ur3e import *
from lab7_img import *

def main():

	# Initialize ROS node
	rospy.init_node('lab7_node')

	'''
	# Initialize CKCamera
	camera = CKCamera()
	camera.init()

	# 1. save first calibration image
	camera.display()

	# 2. Free move the robot to obtain the robot coordinate (x,y) of first block
	# ====================== ⚠️ YOU MUST DEFINE ======================
	# center_robot1:
	#   robot base frame (x, y) of calibration point 1
	#   obtained by free-moving the robot and reading TCP position
	# center_robot1 = (x1_robot, y1_robot)
	# ================================================================
	# center_robot1 = int(input("Please input the robot coordinate (x,y) of this block: "))

	# 3. save second calibration image
	camera.display()

	# 4. Free move the robot to obtain the robot coordinate (x,y) of second block
	# ====================== ⚠️ YOU MUST DEFINE ======================
	# center_robot2:
	#   robot base frame (x, y) of calibration point 2
	#   obtained by free-moving the robot and reading TCP position
	# center_robot2 = (x2_robot, y2_robot)
	# ================================================================
	# center_robot2 = int(input("Please input the robot coordinate (x,y) of this block: "))

	# 5. save snapshot image
	camera.display()

	camera.uninit()
	'''

	# ====================== ⚠️ YOU MUST DEFINE ======================
	# center_robot1, center_robot2 MUST be defined before this call
	# They are used for pixel → robot coordinate calibration
	# Example:
	# center_robot1 = (0.12, 0.18)
	# center_robot2 = (0.18, 0.10)
	# ================================================================
	center_values, shape, theta = lab_imgproc(center_robot1, center_robot2)
	
	############## Your Code Start Here ############## 	
	# TODO: main execution of pick and place task

	# initialize UR3e functions
	ur3e = UR3e()
	print("Please Press 'Start' button on the Teach Pendant to continue ...")

	time.sleep(2)

	# ====================== ⚠️ YOU MUST DEFINE ======================
	# z_above : safe height above the table (avoid collision)
	# z_pick  : actual picking height (depends on block thickness)
	# z_place : placing height
	# THESE VALUES MUST MATCH YOUR REAL SETUP
	# ================================================================
	z_above = 0.10
	z_pick  = 0.02
	z_place = 0.02

	# ====================== ⚠️ YOU MUST DEFINE ======================
	# destination layout in robot base frame
	# base_x, base_y : starting position of sorting area
	# dx, dy         : spacing between blocks
	# These are DESIGN CHOICES, not given by TA
	# ================================================================
	base_x = 0.20
	base_y = 0.10
	dx = 0.04
	dy = 0.04

	count_rect = 0
	count_elip = 0

	for i in range(len(center_values)):
		x = center_values[i][0]   # already robot-frame (from lab7_img.py)
		y = center_values[i][1]
		ang = theta[i]

		# ====================== ⚠️ YOU MUST DEFINE ======================
		# classification rule:
		# shape[i] == 0 → rectangle
		# shape[i] == 1 → ellipse
		# placement strategy is YOUR DESIGN
		# ================================================================
		if shape[i] == 0:
			px = base_x + dx * (count_rect % 3)
			py = base_y + dy * (count_rect // 3)
			count_rect += 1
		else:
			px = base_x + 0.15 + dx * (count_elip % 3)
			py = base_y + dy * (count_elip // 3)
			count_elip += 1

		# lab_invk MUST already be implemented in lab7_func.py (Lab 4/5)
		dest_pos = lab_invk(x, y, z_above, ang)
		ur3e.move(dest_pos)

		dest_pos = lab_invk(x, y, z_pick, ang)
		ur3e.move(dest_pos)

		ur3e.gripper(True)
		time.sleep(0.5)

		dest_pos = lab_invk(x, y, z_above, ang)
		ur3e.move(dest_pos)

		dest_pos = lab_invk(px, py, z_above, ang)
		ur3e.move(dest_pos)

		dest_pos = lab_invk(px, py, z_place, ang)
		ur3e.move(dest_pos)

		ur3e.gripper(False)
		time.sleep(0.5)

		dest_pos = lab_invk(px, py, z_above, ang)
		ur3e.move(dest_pos)

	############### Your Code End Here ###############

	rospy.loginfo("Finish Pick and place task!")

if __name__ == '__main__':
	
	try:
		main()
	except rospy.ROSInterruptException:
		pass
