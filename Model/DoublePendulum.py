
import abc
import numpy as np
import rbdl
import time
import rospy
from threading import Thread
from GaitCore.Bio import Joint, Leg
from GaitCore.Core import Point
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import JointState
from Utlities import PendPlotter
from . import Model

class DoublePendulum(Model.Model):

    def __init__(self, client, model_name, joint_names):
        super(DoublePendulum, self).__init__(client, model_name=model_name, joint_names=joint_names)
        self._handle = self._client.get_obj_handle('head')

        self.rbdl_model = self.dynamic_model()
        self._state = (self._q, self._qd)


        self._updater.start()


    def torque_cb(self, tau):
        self.update_torque(list(tau.effort))

    def update_torque(self, tau):
        """

        :type tau: list
        """
        self.tau = tau
        self._enable_control = True

    @property
    def enable_control(self):
        return self._enable_control

    @enable_control.setter
    def enable_control(self, value):
        self._enable_control = value

    @property
    def handle(self):
        return self._handle

    @handle.setter
    def handle(self, value):
        self._handle = value

    @property
    def q(self):
        return self._q

    @q.setter
    def q(self, value):
        self._q = np.asarray(value)

    @property
    def qd(self):
        return self._qd

    @qd.setter
    def qd(self, value):

        self._qd = np.asarray(value)

    @property
    def state(self):
        return self._state

    @state.setter
    def state(self, value):
        self._state = np.concatenate(value)

    @abc.abstractmethod
    def dynamic_model(self):
        # m1 = 1
        # m2 = 1
        # l1 = 0.5
        # l2 = 0.5
        # r1 = 0
        # r2 = -0.6
        # I1 = np.diag([0.06, 0.06, 0.002])
        # I2 = np.diag([0.06, 0.06, 0.002])
        # joint_rot_y = rbdl.Joint.fromJointType("JointTypeRevoluteY")
        #
        # xtrans_1 = rbdl.SpatialTransform()
        # xtrans_1.r = np.array([0.0, 0.0, 0.0])
        #
        # xtrans_2 = rbdl.SpatialTransform()
        # xtrans_2.r = np.array([0.0, 0.0, r2])

        model = rbdl.Model()
        parent_dist = {}
        mass = {}
        com = {}
        inertia = {}
        mass["head"] = 0
        mass["top"] = 1
        mass["bottom"] = 1
        parent_dist["head"] = np.array([0.0, 0.0, 0.0])
        parent_dist["top"] = np.array([1.6763, 2.293, 3.1565])
        parent_dist["bottom"] = np.array([-0.0687, 0.0044, -0.6083])
        inertia["head"] = np.diag([0.0, 0.0, 0.0])
        inertia["top"] = np.diag([0.6656, 0.6416, 0.0272])
        inertia["bottom"] = np.diag([0.6656, 0.6416, 0.0272])
        com["head"] = np.array([0.0, 0.0, 0.0])
        com["top"] = np.array([0.0, 0.0, 0.0])
        com["bottom"] = np.array([0.0, 0.0, 0.0])

        head = rbdl.Body.fromMassComInertia(mass["head"], com["head"], inertia["head"])
        top = rbdl.Body.fromMassComInertia(mass["top"], com["top"], inertia["top"])
        bottom = rbdl.Body.fromMassComInertia(mass["bottom"], com["bottom"], inertia["bottom"])

        xtrans = rbdl.SpatialTransform()
        xtrans.r = np.array([0.0, 0.0, 0.0])
        xtrans.E = np.eye(3)
        self.head = model.AddBody(0, xtrans, rbdl.Joint.fromJointType("JointTypeFixed"), head, "head")
        joint_rot_z = rbdl.Joint.fromJointType("JointTypeRevoluteX")

        xtrans.r = parent_dist["top"]
        self.top = model.AddBody(self.head, xtrans, joint_rot_z, top, "top")
        xtrans.E = np.eye(3)
        xtrans.r = parent_dist["bottom"]
        self.bottom = model.AddBody(self.top, xtrans, joint_rot_z, bottom, "bottom")

        model.gravity = np.array([0, 0, -9.81])
        return model

    def update(self):
        """

        :return:
        """
        rate = rospy.Rate(1000)  # 1000hz
        q_msg = Float32MultiArray()
        while 1:
            self.q = self.handle.get_all_joint_pos()
            self.qd = self.handle.get_all_joint_vel()
            self._joint_num = self.q.size
            q_msg.data = self.q
            self.q_pub.publish(q_msg)
            if self._enable_control:
                self.handle.set_all_joint_effort(self.tau)

            rate.sleep()
            #self.plt.update()

    @abc.abstractmethod
    def  ambf_to_dyn(self, q):
        pass

    @abc.abstractmethod
    def fk(self):
        """

        :param model: model
        :type model: rbdl.model
        :return: 2D array of the joint locations
        """

        x = []
        y = []

        point_local = np.array([0.0, 0.0, 0.0])
        data = rbdl.CalcBodyToBaseCoordinates(self._model, self.q, self.b0, point_local)
        x.append(data[0])
        y.append(data[2])
        data = rbdl.CalcBodyToBaseCoordinates(self._model, self.q, self.b1, point_local)
        x.append(data[0])
        y.append(data[2])
        data = rbdl.CalcBodyToBaseCoordinates(self._model, self.q, self.b2, point_local)
        x.append(data[0])
        y.append(data[2])
        return (x, y)

    @abc.abstractmethod
    def update_state(self, q, qd):
        self.state = q + qd

    def calculate_dynamics(self, qdd):
        tau = np.asarray([0.0] * self._joint_num)
        rbdl.InverseDynamics(self.rbdl_model, self.q[0:2], self.qd[0:2], qdd[0:2], tau)
        return tau

    def calculate_torque(self):
        force1 = np.array([2.0, 2.0, 2.0])
        force2 = np.array([3.0, 4.0, -2.0])
        force3 = np.array([-1.0, 3.0, 5.0])
        force4 = np.array([12.0, 22.0, 22.0])
        forces = [force1, force2, force3, force4]

        point1 = np.array([2.0, 2.0, 2.0])
        point2 = np.array([4.0, 4.0, 4.0])

        points = [point1, point2]
        bodies = [self.top, self.bottom]
        topJ = np.zeros((3, 2))
        bottomJ = np.zeros((3, 2))
        J = [topJ, bottomJ]
        print(self.q)
        for i, body in enumerate(bodies):
            for force in forces:

                rbdl.CalcPointJacobian(self.rbdl_model, np.array([0.0, 0.0]), body, points[i], J[i])
                J_t = np.transpose(J[i])
                print(np.dot(J_t, np.transpose(force)))
