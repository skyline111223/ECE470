#!/usr/bin/env python
# -*- coding: utf-8 -*-

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

    # ==============================
    # Define tower configuration
    # ==============================
    #
    # Q[row][col]: joint angles for contact positions
    #   row = 0/1/2 → top/middle/bottom block
    #   col = 0/1/2 → tower 1/2/3
    #
    # Each element in Q should be a NumPy array of 6 joint angles (in radians)
    # corresponding to the position where the suction cup just touches
    # the block at that tower and height.
    #
    # Steps (suggested):
    # 1) Move to home position
    # 2) Move down to the contact position of the start block
    # 3) Pick the block (enable suction)
    # 4) Move back to home
    # 5) Move to the contact position of the destination block
    # 6) Place the block (disable suction)
    # 7) Return to home

    # ---------- Define home and tower positions ----------

    # Example home position (replace with your own safe home joint angles)
    home = np.radians([0.0, -90.0, 90.0, -90.0, -90.0, 0.0])

    # Example contact joint angles for each tower/level
    # !!! Replace these with your calibrated joint angles !!!
    Q = [
        [  # Top layer
            np.radians([-15, -100, 90, -80, -90, 0]),  # Tower 1 top
            np.radians([0, -100, 90, -80, -90, 0]),    # Tower 2 top
            np.radians([15, -100, 90, -80, -90, 0]),   # Tower 3 top
        ],
        [  # Middle layer
            np.radians([-15, -110, 95, -75, -90, 0]),  # Tower 1 mid
            np.radians([0, -110, 95, -75, -90, 0]),    # Tower 2 mid
            np.radians([15, -110, 95, -75, -90, 0]),   # Tower 3 mid
        ],
        [  # Bottom layer
            np.radians([-15, -120, 100, -70, -90, 0]),  # Tower 1 bottom
            np.radians([0, -120, 100, -70, -90, 0]),    # Tower 2 bottom
            np.radians([15, -120, 100, -70, -90, 0]),   # Tower 3 bottom
        ],
    ]

    # ---------- Get user input for start and destination towers ----------

    def ask_peg(name):
        while True:
            s = input(f"Select {name} tower (enter 1 / 2 / 3, or 0 to quit): ").strip()
            if s == "0":
                print("Quitting...")
                sys.exit(0)
            if s in ("1", "2", "3"):
                return int(s) - 1
            print("Invalid input. Please enter 1, 2, 3, or 0.")

    start = ask_peg("start")
    des = ask_peg("destination")
    while des == start:
        print("Start and destination towers cannot be the same.")
        des = ask_peg("destination")

    mid = 3 - start - des  # The remaining tower

    rospy.loginfo(f"Start = Tower {start+1}, Destination = Tower {des+1}, Aux = Tower {mid+1}")
    rospy.loginfo("Sending Goals ...")

    # ---------- Initialize the robot ----------

    ur3e = UR3e()
    ur3e.init_array(home, Q)
    time.sleep(1.0)
    ur3e.move_arm(home)
    time.sleep(0.5)

    # ---------- Tower of Hanoi logic ----------

    # Keep track of each tower's blocks (2=bottom, 1=middle, 0=top)
    stacks = {0: [], 1: [], 2: []}
    stacks[start] = [2, 1, 0]
    stacks[mid] = []
    stacks[des] = []

    def move_one_disk(src_col, dst_col):
        """Move one disk from src_col to dst_col."""
        if not stacks[src_col]:
            raise RuntimeError(f"No disk to move from tower {src_col+1}")

        disk = stacks[src_col].pop()
        src_row = len(stacks[src_col])     # after popping, the row matches remaining count
        dst_row = len(stacks[dst_col])     # destination row before placing

        rospy.loginfo(f"Move disk {disk}: Tower {src_col+1} (row {src_row}) -> Tower {dst_col+1} (row {dst_row})")
        ur3e.move_block(src_col, src_row, dst_col, dst_row)
        time.sleep(0.2)

        stacks[dst_col].append(disk)

    def hanoi(n, src, aux, dst):
        """Recursive Hanoi algorithm."""
        if n == 1:
            move_one_disk(src, dst)
        else:
            hanoi(n-1, src, dst, aux)
            move_one_disk(src, dst)
            hanoi(n-1, aux, src, dst)

    # Solve 3-disk Tower of Hanoi
    hanoi(3, start, mid, des)

    # Return to home
    ur3e.move_arm(home)
    rospy.loginfo("Hanoi complete.")


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
