#!/usr/bin/env python
import sys
# from os import sys, path
# sys.path.append(path.dirname(path.dirname(path.abspath(__file__))))
import numpy as np
from StateMachines import StateMachine
from Controller import ControllerNode
from Model import Exoskeleton, Human
import rospy
from ambf_client import Client
from Controller import DynController

Kp = np.zeros((7, 7))
Kd = np.zeros((7, 7))

Kp_hip = 1000
Kd_hip = 25

Kp_knee = 2000
Kd_knee = 30

Kp_ankle = 2000
Kd_ankle = 25

Kp[0, 0] = 0
Kd[0, 0] = 0
Kp[1, 1] = 0
Kd[1, 1] = 0
Kp[2, 2] = 0
Kd[2, 2] = 0

Kp[3, 3] = Kp_hip
Kd[3, 3] = Kd_hip
Kp[4, 4] = Kp_knee
Kd[4, 4] = Kd_knee
Kp[5, 5] = Kp_ankle
Kd[5, 5] = Kd_ankle


_client = Client()
_client.connect()
rate = rospy.Rate(1000)

body_joints = ['HumanLeftHip', 'HumanLeftKnee', 'HumanLeftAnkle',
               'HumanRightHip', 'HumanRightKnee', 'HumanRightAnkle']

robot_joints = ['ExoLeftHip', 'ExoLeftKnee', 'ExoLeftAnkle', 'ExoRightHip', 'ExoRightKnee', 'ExoRightAnkle', 'ExoHipCrutches']
LARRY = Human.Human(_client, "human", body_joints, 0, 0)
LARRE = Exoskeleton.Exoskeleton(_client, "exo", robot_joints, 56, 1.56)
# LARRE.handle.set_rpy(0.25, 0, 0)
# LARRE.handle.set_pos(0, 0, 1.0)
Dyn = DynController.DynController(LARRE, Kp, Kd)

#mpc = MPController.MPController(LARRE, LARRE.get_runner())


# lqr = LQRController.LQRController(LARRE, LARRE.get_runner())
# controllers = {'Dyn': Dyn,
#                "LQR":lqr}

# lqr = LQRController.LQRController(LARRE, LARRE.get_runner())
controllers = {'Dyn': Dyn}

cnrl = ControllerNode.ControllerNode(LARRE, controllers)
#


# while True:
#     fk = LARRE.fk()
#     print(fk["right_hip"])
# while True:
#     LARRE.calculate_torque()
LARRE.handle.set_rpy(0, 0, 0)
LARRE.handle.set_pos(0.0, 0, 1.0)
machine = StateMachine.ExoStateMachine(LARRE)
