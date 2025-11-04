#!/usr/bin/env python
# -*- coding: utf-8 -*-

'''
lab3pkg_hanoi/lab3_ur3e.py

@brief: UR3e class including functions for controlling the UR3e arm and the gripper.
'''

import time
import rospy

# Import message headers and types
from lab3_header import *

class UR3e():

    def __init__(self):

        # Store current arm and gripper state
        self.current_position = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.current_io_0 = False  # suction cup state (True = ON, False = OFF)

        # Publisher for joint motion control
        self.pub_setjoint = rospy.Publisher('ur3e_driver_ece470/setjoint',
                                            JointTrajectory, queue_size=10)

        # TODO: define a publisher for gripper I/O control
        self.pub_setio = rospy.Publisher('ur3e_driver_ece470/setio',
                                         SetIO, queue_size=10)

        # Subscriber for joint state feedback
        self.sub_position = rospy.Subscriber('/joint_states',
                                             JointState, self.position_callback)

        # TODO: define subscriber for gripper state feedback
        self.sub_io = rospy.Subscriber('/ur_hardware_interface/io_states',
                                       IOStates, self.gripper_input_callback)

        # These will be initialized later
        self.home = None
        self.Q = None
    
    def init_array(self, home, Q):
        """Initialize home position and tower position array Q"""
        self.home = home
        self.Q = Q

    def gripper_input_callback(self, msg):
        """
        Callback for gripper I/O state.
        Updates the suction state (self.current_io_0) whenever
        /ur_hardware_interface/io_states publishes data.
        """
        # ----------- Your Code -----------
        state = None
        try:
            if hasattr(msg, 'digital_out_states') and len(msg.digital_out_states) > 0:
                state = bool(msg.digital_out_states[0].state)
        except Exception:
            pass

        if state is None:
            try:
                if hasattr(msg, 'standard_digital_output') and len(msg.standard_digital_output) > 0:
                    state = bool(msg.standard_digital_output[0])
            except Exception:
                pass

        if state is None:
            try:
                if hasattr(msg, 'tool_output'):
                    state = bool(msg.tool_output)
            except Exception:
                pass

        if state is not None:
            self.current_io_0 = state
        # ----------- End Code -----------

    def position_callback(self, msg):
        """
        Callback for current arm position.
        Updates self.current_position from /joint_states messages.
        """
        desired_order = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
                         "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]
        try:
            name_to_pos = {n: p for n, p in zip(msg.name, msg.position)}
            self.current_position = [name_to_pos.get(jn, 0.0) for jn in desired_order]
        except Exception:
            pass

    def gripper(self, io_0):
        """
        Publish to 'ur3e_driver_ece470/setio' to control the gripper suction cup.
        """
        msg = SetIO()
        # Handle different possible field names
        if hasattr(msg, 'io_0'):
            msg.io_0 = bool(io_0)
        elif hasattr(msg, 'state'):
            msg.state = bool(io_0)
        elif hasattr(msg, 'output'):
            msg.output = bool(io_0)
        else:
            try:
                setattr(msg, 'io_0', bool(io_0))
            except Exception:
                pass

        self.pub_setio.publish(msg)
        time.sleep(0.2)
        self.current_io_0 = bool(io_0)

    def move_arm(self, dest):
        """
        Move the UR3e arm to a desired joint configuration.
        """
        msg = JointTrajectory()
        msg.joint_names = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
                           "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]
        point = JointTrajectoryPoint()
        point.positions = dest
        point.time_from_start = rospy.Duration(2.0)
        msg.points.append(point)
        self.pub_setjoint.publish(msg)
        time.sleep(2.5)  # allow motion to finish

    def move_block(self, start_loc, start_height, end_loc, end_height):
        """
        Move a block from (start_loc, start_height) to (end_loc, end_height).

        Steps:
        1. Move to home (safe position)
        2. Move down to pick-up position Q[start_height][start_loc]
        3. Turn gripper ON
        4. Lift up to home
        5. Move to drop-off position Q[end_height][end_loc]
        6. Turn gripper OFF
        7. Return to home
        """
        assert self.Q is not None and self.home is not None, "Call init_array(home, Q) first."

        pick_pose = self.Q[start_height][start_loc]
        place_pose = self.Q[end_height][end_loc]

        # 1. Move to home (safe height)
        self.move_arm(self.home)

        # 2. Move down to pick-up position
        self.move_arm(pick_pose)

        # 3. Turn gripper ON (suction)
        self.gripper(True)
        time.sleep(0.3)

        # 4. Lift up to home
        self.move_arm(self.home)

        # 5. Move down to place position
        self.move_arm(place_pose)

        # 6. Turn gripper OFF (release)
        self.gripper(False)
        time.sleep(0.3)

        # 7. Lift up to home
        self.move_arm(self.home)
