"""
This should be moved to a seperate repo later
"""

import abc
import numpy as np
import rbdl
import Model
import time
import message_filters
from GaitCore.Core import Point
from GaitCore.Core import utilities
from std_msgs.msg import Float32MultiArray
from threading import Thread
from . import Model
from GaitCore.Bio import Leg, Joint
import rospy
from ambf_msgs.msg import RigidBodyState
from GaitAnaylsisToolkit.LearningTools.Runner import TPGMMRunner

class Human(Model.Model):

    def __init__(self, client, model_name, joint_names, mass, height):
        # inits dynamic model and joints for leg
        super(Human, self).__init__(client, model_name=model_name, joint_names=joint_names)
        self._mass = mass
        self._height = height

        self.handle = self._client.get_obj_handle('Hip')
        self.rbdl_model = self.dynamic_model()
        # num_of_segments should be initialized with the dynamical model, which is created in the constructor
        self.num_joints = len(self.handle.get_joint_names())
        self.q = self.num_joints * [0.0]
        self.qd = self.num_joints * [0.0]

        time.sleep(2)
        self._state = (self._q, self._qd)
        self._updater.start()  # start update thread

        self.ambf_order_crutch_left = {'crutch': 0, 'hip': 1, 'ankle': 2, 'knee': 3, 'elbow': 4, 'shoulder': 5, 'wrist': 6, 'neck': 7}
        self.ambf_order_crutch_right = {'crutch': 8, 'hip': 9, 'ankle': 10, 'knee': 11, 'elbow': 12, 'shoulder': 13, 'wrist': 14, 'neck': 7}

    @property
    def state(self):
        return self._state

    @state.setter
    def state(self, value):
        self._state = np.concatenate(value)

    # @q.setter
    # def q(self, value):
    #     # TODO: transform AMBF to RBDL
    #     self._q = np.asarray(value)
    #
    # @qd.setter
    # def qd(self, value):
    #     # TODO: transform RBDL to AMBF
    #     value[2] *= -1
    #     value[5] *= -1
    #     self._qd = np.asarray(value)

    def dynamic_model(self):
        model = rbdl.Model()
        bodies = {}
        mass = {}
        com = {}
        inertia = {}
        bodies["right"] = {}
        bodies["left"] = {}
        segments = ["thigh", "shank", "foot"]

        mass["hip"] = 2.37
        mass["right_thigh"] = 2.11
        mass["left_thigh"] = 2.11
        mass["right_shank"] = 1.28
        mass["left_shank"] = 1.28
        mass["right_foot"] = 0.86
        mass["left_foot"] = 0.86

        return model

    def fk(self):
        fk = {}

    def update_state(self, q, qd):
        pass

    def calculate_torque(self, model):
        left_leg, right_leg = model.get_leg_sensors()
        print(left_leg[0].x, right_leg[0])
        G = np.array([], (3, 3))
        jacobian = rbdl.CalcPointJacobian(self.rbdl_model, self.q, 0, 10, G)
        # jacobiang = G
        # print(jacobian, jacobiang)