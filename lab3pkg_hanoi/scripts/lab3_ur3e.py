#!/usr/bin/env python
# -*- coding: utf-8 -*-

import time
import rospy

from ur_msgs.msg import IOStates
from ur_msgs.srv import SetIO, SetIORequest, SetIOResponse
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class UR3e:
    def __init__(self, gripper_pin=0, setio_srv_name="/ur3e_driver_ece470/setio"):
        """
        :param gripper_pin: digital output pin index for the vacuum (default 0)
        :param setio_srv_name: service name for ur_msgs/SetIO
        """
        self.gripper_pin = int(gripper_pin)
        self.setio_srv_name = setio_srv_name

        # Cached states
        self.current_position = [0.0] * 6
        self.current_io_0 = False  # Whether the gripper pin is ON

        # Publishers
        self.pub_setjoint = rospy.Publisher(
            "ur3e_driver_ece470/setjoint", JointTrajectory, queue_size=10
        )

        # Subscribers
        self.sub_position = rospy.Subscriber(
            "/joint_states", JointState, self.position_callback
        )
        self.sub_io = rospy.Subscriber(
            "/ur_hardware_interface/io_states", IOStates, self.gripper_input_callback
        )

        # Service client for SetIO
        rospy.loginfo("Waiting for SetIO service: %s", self.setio_srv_name)
        rospy.wait_for_service(self.setio_srv_name)
        self.setio = rospy.ServiceProxy(self.setio_srv_name, SetIO)
        rospy.loginfo("SetIO service connected.")

        # To be filled by init_array
        self.home = None
        self.Q = None

    def init_array(self, home, Q):
        """Set the home joint configuration and the contact array Q[row][col]."""
        self.home = home
        self.Q = Q

    # ---------- Callbacks ----------

    def gripper_input_callback(self, msg: IOStates):
        """
        Update self.current_io_0 from /ur_hardware_interface/io_states.
        We look up digital_out_states and read our configured pin.
        """
        state = None
        try:
            for d in msg.digital_out_states:
                if d.pin == self.gripper_pin:
                    state = bool(d.state)
                    break
        except Exception:
            pass

        if state is not None:
            self.current_io_0 = state

    def position_callback(self, msg: JointState):
        """
        Update self.current_position using /joint_states in the controller's expected order.
        """
        desired = [
            "shoulder_pan_joint",
            "shoulder_lift_joint",
            "elbow_joint",
            "wrist_1_joint",
            "wrist_2_joint",
            "wrist_3_joint",
        ]
        try:
            name_to_pos = {n: p for n, p in zip(msg.name, msg.position)}
            self.current_position = [name_to_pos.get(j, 0.0) for j in desired]
        except Exception:
            pass

    # ---------- Gripper & Motion ----------

    def gripper(self, on: bool) -> bool:
        """
        Control the vacuum via ur_msgs/SetIO.
        :param on: True -> ON, False -> OFF
        :return: service call success
        """
        req = SetIORequest()
        req.fun = SetIORequest.FUN_SET_DIGITAL_OUT   # 1
        req.pin = self.gripper_pin
        req.state = float(SetIORequest.STATE_ON if on else SetIORequest.STATE_OFF)
        try:
            resp: SetIOResponse = self.setio(req)
            if not resp.success:
                rospy.logerr("SetIO failed: pin=%d on=%s", self.gripper_pin, on)
            else:
                self.current_io_0 = bool(on)
            # small delay to allow valve to settle
            time.sleep(0.2)
            return bool(resp.success)
        except rospy.ServiceException as e:
            rospy.logerr("SetIO service call failed: %s", e)
            return False

    def move_arm(self, dest):
        """Send a 2s joint trajectory to the desired joint positions (rad)."""
        msg = JointTrajectory()
        msg.joint_names = [
            "shoulder_pan_joint",
            "shoulder_lift_joint",
            "elbow_joint",
            "wrist_1_joint",
            "wrist_2_joint",
            "wrist_3_joint",
        ]
        pt = JointTrajectoryPoint()
        pt.positions = dest
        pt.time_from_start = rospy.Duration(2.0)
        msg.points.append(pt)
        self.pub_setjoint.publish(msg)
        time.sleep(2.5)

    def move_block(self, start_loc, start_height, end_loc, end_height):
        """
        Pick at Q[start_height][start_loc], place at Q[end_height][end_loc], with home in between.
        """
        assert self.Q is not None and self.home is not None, "Call init_array(home, Q) first."

        pick_pose = self.Q[start_height][start_loc]
        place_pose = self.Q[end_height][end_loc]

        # Safe raise
        self.move_arm(self.home)

        # Pick
        self.move_arm(pick_pose)
        if not self.gripper(True):
            raise RuntimeError("Gripper ON failed via SetIO.")
        # Optional: check IO feedback; if your setup reports vacuum success on another pin, read it here.

        self.move_arm(self.home)

        # Place
        self.move_arm(place_pose)
        if not self.gripper(False):
            raise RuntimeError("Gripper OFF failed via SetIO.")

        self.move_arm(self.home)
