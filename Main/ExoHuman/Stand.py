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

Kp_hip = 250.0
Kd_hip = 0.5

Kp_knee = 50.0
Kd_knee = 0.5

Kp_ankle = 250.0
Kd_ankle = 0.8

Kp[0, 0] = Kp_hip
Kd[0, 0] = Kd_hip
Kp[1, 1] = Kp_knee
Kd[1, 1] = Kd_knee
Kp[2, 2] = Kp_ankle
Kd[2, 2] = Kd_ankle

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
               'HumanRightHip', 'HumanRightKnee', 'HumanRightAnkle',]

robot_joints = ['ExoLeftHip', 'ExoLeftKnee', 'ExoLeftAnkle',
                'ExoRightHip', 'ExoRightKnee', 'ExoRightAnkle',  'ExoHipCrutches']
# LARRY = Human.Human(_client, "human", body_joints, 0, 0)
LARRE = Exoskeleton.Exoskeleton(_client, "exo", robot_joints, 56, 1.56)
# LARRE.handle.set_rpy(0.25, 0, 0)
# LARRE.handle.set_pos(0, 0, 1.0)
Dyn = DynController.DynController(LARRE, Kp, Kd)


controllers = {'Dyn': Dyn}

cnrl = ControllerNode.ControllerNode(LARRE, controllers)
#

LARRE.handle.set_rpy(0.25, 0, 0)
LARRE.handle.set_pos(5, 5, -0.1)
#machine = StateMachine.ExoStateMachine(LARRE)
