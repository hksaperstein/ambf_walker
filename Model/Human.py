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

        mass["hip"] = 5
        mass["right_thigh"] = 2.11
        mass["left_thigh"] = 2.11
        mass["right_shank"] = 1.28
        mass["left_shank"] = 1.28
        mass["right_foot"] = 0.866
        mass["left_foot"] = 0.866

        parent_dist = {}
        parent_dist["hip"] = np.array([0.0, 0.0, 0.0])

        parent_dist["left_thigh"] = np.array([0.0061, -0.0263, -.6277])
        parent_dist["left_shank"] = np.array([0.1445, 0.0231, -0.4364])
        parent_dist["left_foot"] = np.array([0.0, 0.0268, -0.3881])

        parent_dist["right_thigh"] = np.array([-0.0061, -0.0263, -0.6277])
        parent_dist["right_shank"] = np.array([-0.1445, 0.0231, -0.4364])
        parent_dist["right_foot"] = np.array([-0.0, 0.0268, -0.3881])

        inertia["hip"] = np.diag([ 0.0,0.0,0.0])

        inertia["left_thigh"] = np.diag([0.0, 0.0, 0.07])
        inertia["left_shank"] = np.diag([0.0, 0.0, 0.05])
        inertia["left_foot"] = np.diag([0.0, 0.0, 0.0])

        inertia["right_thigh"] = np.diag([0.0, 0.00, 0.07])
        inertia["right_shank"] = np.diag([0.0, 0.0, 0.05])
        inertia["right_foot"] = np.diag([0.0, 0.0, 0.0])

        com["hip"] = np.array([0.00, -0.02, 0.18])
        com["left_thigh"] = np.array([0.02, 0.01,  -0.09])
        com["left_shank"] = np.array([-0.02, -0.007, 0.06])
        com["left_foot"] = np.array([0.08, -0.06, 0.04])

        com["right_thigh"] = np.array([-0.02, 0.01, -0.09])
        com["right_shank"] = np.array([0.02, -0.007, 0.06])
        com["right_foot"] = np.array([0.08, -0.06, 0.04])

        com["left_thigh"] = np.array([.1009, 0.0088, -0.1974])  # needs to be fixed in blender
        com["left_shank"] = np.array([-0.0305, 0.0174, -0.2118])
        com["left_foot"] = np.array([-0.0052, -0.0017, -0.0235])

        com["right_thigh"] = np.array([-.1009, 0.0088, -0.1974])  # needs to be fixed in blender
        com["right_shank"] = np.array([0.0305, 0.0174, -0.2118])
        com["right_foot"] = np.array([0.0052, -0.0017, -0.0235])

        hip_body = rbdl.Body.fromMassComInertia(mass["hip"], com["hip"], inertia["hip"])
        for segs in segments:
            bodies["right_" + segs] = rbdl.Body.fromMassComInertia(mass["right_" + segs], com["right_" + segs], inertia["right_" + segs])
            bodies["left_" + segs] = rbdl.Body.fromMassComInertia(mass["left_" + segs], com["left_" + segs], inertia["left_" + segs])

        xtrans = rbdl.SpatialTransform()
        xtrans.r = np.array([0.0, 0.0, 0.0])
        xtrans.E = np.eye(3)

        self.hip = model.AddBody(0, xtrans, rbdl.Joint.fromJointType("JointTypeFixed"), hip_body, "hip")
        joint_rot_z = rbdl.Joint.fromJointType("JointTypeRevoluteX")

        xtrans.r = parent_dist["left_thigh"]
        self.left_thigh = model.AddBody(self.hip, xtrans, joint_rot_z, bodies["left_thigh"], "left_thigh")
        xtrans.E = np.eye(3)
        xtrans.r = parent_dist["left_shank"]
        self.left_shank = model.AddBody(self.left_thigh, xtrans, joint_rot_z, bodies["left_shank"], "left_shank")
        xtrans.r = parent_dist["left_foot"]
        self.left_foot = model.AddBody(self.left_shank, xtrans, joint_rot_z, bodies["left_foot"], "left_foot")

        xtrans.r = parent_dist["right_thigh"]
        self.right_thigh = model.AddBody(self.hip, xtrans, joint_rot_z, bodies["right_thigh"], "right_thigh")
        xtrans.E = np.eye(3)
        xtrans.r = parent_dist["right_shank"]
        self.right_shank = model.AddBody(self.right_thigh, xtrans, joint_rot_z, bodies["right_shank"], "right_shank")
        xtrans.r = parent_dist["right_foot"]
        self.right_foot = model.AddBody(self.right_shank, xtrans, joint_rot_z, bodies["right_foot"], "right_foot")

        model.gravity = np.array([0, 0, -9.81])

        return model

    def fk(self):
        fk = {}

    def update_state(self, q, qd):
        pass

    def calculate_torque(self):
        pass
        # left_leg_force, right_leg_force = model.get_leg_sensors()
        #
        # front_left_thigh_tab_force = [left_leg_force[0].x, left_leg_force[0].y, left_leg_force[0].z]
        # front_left_shank_tab_force = [left_leg_force[1].x, left_leg_force[1].y, left_leg_force[1].z]
        # front_right_thigh_tab_force = [right_leg_force[0].x, right_leg_force[0].y, right_leg_force[0].z]
        # front_right_shank_tab_force = [right_leg_force[1].x, right_leg_force[1].y, right_leg_force[1].z]
        # forces = [front_left_thigh_tab_force, front_left_shank_tab_force, front_right_thigh_tab_force, front_right_shank_tab_force]
        #
        # # Sensor point locations
        # # add back sensor logic
        # front_left_thigh_tab_location = np.array([0.1229, -0.2401, -0.1364])
        # front_right_thigh_tab_location = np.array([-0.1229, -0.2401, -0.1364])
        # front_left_shank_tab_location = np.array([0.1229, -0.184, -0.5801])
        # front_right_shank_tab_location = np.array([-0.1236, -0.184, -0.5801])
        # locations = [front_left_thigh_tab_location, front_left_shank_tab_location, front_right_thigh_tab_location, front_right_shank_tab_location]
        #
        # J = np.zeros((3, 6))
        #
        # # need proper body_ids
        # rbdl.CalcPointJacobian(self.rbdl_model, self.q, 1, front_left_thigh_tab_location, J)
        # J_t = np.transpose(J)
        # print(J_t)
        # print(J_t*forces[0])