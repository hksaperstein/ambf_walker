#!/usr/bin/env python
import sys
import os
# from os import sys, path
# sys.path.append(path.dirname(path.dirname(path.abspath(__file__))))
import numpy as np
dynamic_path = os.path.abspath(__file__+"/../../")
print(dynamic_path)
sys.path.append(dynamic_path)
from StateMachines import StateMachine
from Controller import ControllerNode
from Model import Exoskeleton
import rospy
from ambf_client import Client
from Controller import DynController, MPCController, FeedForwardController ,LQRController


####### Dynamcis Controller #######
Kp_Dyn = np.zeros((7, 7))
Kd_Dyn = np.zeros((7, 7))
#
Kp_hip = 100.0
Kd_hip = 0.5

Kp_knee = 100.0
Kd_knee = 1.0

Kp_ankle = 100.0
Kd_ankle = 0.4

Kp_Dyn[0, 0] = Kp_hip
Kd_Dyn[0, 0] = Kd_hip
Kp_Dyn[1, 1] = Kp_knee
Kd_Dyn[1, 1] = Kd_knee
Kp_Dyn[2, 2] = Kp_ankle
Kd_Dyn[2, 2] = Kd_ankle

Kp_Dyn[3, 3] = Kp_hip
Kd_Dyn[3, 3] = Kd_hip
Kp_Dyn[4, 4] = Kp_knee
Kd_Dyn[4, 4] = Kd_knee
Kp_Dyn[5, 5] = Kp_ankle
Kd_Dyn[5, 5] = Kd_ankle

####### FF Controller #######
Kp_FF = np.zeros((7, 7))
Kd_FF = np.zeros((7, 7))
#
Kp_hip = 150.0
Kd_hip = 0.85

Kp_knee = 50.0
Kd_knee = 1.0

Kp_ankle = 50.0
Kd_ankle = 0.2

Kp_FF[0, 0] = Kp_hip
Kd_FF[0, 0] = Kd_hip
Kp_FF[1, 1] = Kp_knee
Kd_FF[1, 1] = Kd_knee
Kp_FF[2, 2] = Kp_ankle
Kd_FF[2, 2] = Kd_ankle

Kp_FF[3, 3] = Kp_hip
Kd_FF[3, 3] = Kd_hip
Kp_FF[4, 4] = Kp_knee
Kd_FF[4, 4] = Kd_knee
Kp_FF[5, 5] = Kp_ankle
Kd_FF[5, 5] = Kd_ankle

_client = Client()
_client.connect()
rate = rospy.Rate(1000)

joints = ['Hip-RobLeftThigh', 'RobLeftThigh-RobLeftShank', 'RobLeftShank-RobLeftFoot',
          'Hip-RobRightThigh', 'RobRightThigh-RobRightShank', 'RobRightShank-RobRightFoot',  'Hip-Crutches']


#joints = ['Hip-Leftthigh', 'Leftthigh-Leftshank', 'Leftshank-Leftfoot', 'Hip-Rightthigh', 'Rightthigh-Rightshank', 'Rightshank-Rightfoot', 'Hip-Cylinder']


LARRE = Exoskeleton.Exoskeleton(_client, joints, 56, 1.56)
Dyn = DynController.DynController(LARRE, Kp_Dyn, Kd_Dyn)


FF = FeedForwardController.FeedForwardController(LARRE, Kp_FF, Kd_FF)
controllers = {'Dyn': Dyn,
               "FF": FF}

cnrl = ControllerNode.ControllerNode(LARRE, controllers)

machine = StateMachine.ExoStateMachine(LARRE)
