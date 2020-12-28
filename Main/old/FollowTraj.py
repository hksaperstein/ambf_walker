#!/usr/bin/env python
import sys
# from os import sys, path
# sys.path.append(path.dirname(path.dirname(path.abspath(__file__))))
import numpy as np
from StateMachines import StateMachine
from Controller import ControllerNode
from Model import Exoskeleton
import rospy
from ambf_client import Client
from Controller import DynController
Kp = np.zeros((7, 7))
Kd = np.zeros((7, 7))

Kp_hip = 50.0
Kd_hip = 0.5

Kp_knee = 125.0
Kd_knee = 1.0

Kp_ankle = 100.0
Kd_ankle = 0.4

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
LARRE = Exoskeleton.Exoskeleton(_client, 56, 1.56)
controller = DynController.DynController(LARRE, Kp, Kd)
cnrl = ControllerNode.ControllerNode(LARRE, controller)
#machine = StateMachine.ExoStateMachineFollowing(LARRE)
machine = StateMachine.ExoStateMachineGoTo(LARRE)

