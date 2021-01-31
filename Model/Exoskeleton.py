"""
This should be moved to a separate repo later
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
from ambf_msgs.msg import RigidBodyState, SensorState
from GaitAnaylsisToolkit.LearningTools.Runner import TPGMMRunner
from geometry_msgs.msg import Point32
from sensor_msgs.msg import PointCloud
from os.path import dirname, join



class Exoskeleton(Model.Model):

    def __init__(self, client, model_name, joints, mass, height):
        super(Exoskeleton, self).__init__(client, model_name=model_name, joint_names=joints)
        self._handle = self._client.get_obj_handle('ExoHip')
        self.left_foot = self._client.get_obj_handle('ExoLeftFoot')
        self.right_foot = self._client.get_obj_handle('ExoRightFoot')
        # self.left_foot.set_pos(0.24, -0.55, -0.24)
        # self.left_foot.set_rpy(0.0, 0.0, 0.0)
        # self.right_foot.set_pos(-0.24, -0.55, -0.24)
        # self.right_foot.set_rpy(0.0, 0.0, 0.0)

        # Update to current
        self.prox = {}
        self.prox["LeftSideProx"] = rospy.Publisher('left_leg', PointCloud, queue_size=10)
        self.prox["RightSideProx"] = rospy.Publisher('right_leg', PointCloud, queue_size=10)
        time.sleep(4)
        self._mass = mass
        self._height = height

        self.rbdl_model = self.dynamic_model()
        left_joints = {}
        right_joints = {}

        for joint in (left_joints, right_joints):
            for output in ["Hip", "Knee", "Ankle"]:
                angle = Point.Point(0, 0, 0)
                force = Point.Point(0, 0, 0)
                moment = Point.Point(0, 0, 0)
                power = Point.Point(0, 0, 0)
                joint[output] = Joint.Joint(angle, moment, power, force)

        self._left_leg = Leg.Leg(left_joints["Hip"], left_joints["Knee"], left_joints["Ankle"])
        self._right_leg = Leg.Leg(right_joints["Hip"], right_joints["Knee"], right_joints["Ankle"])

        self._state = (self._q, self._qd)

        # START ATTEMPT
        # self._left_thigh_sensorF = Point.Point(0, 0, 0)
        # self._left_thigh_sensorB = Point.Point(0, 0, 0)
        # self._left_shank_sensorF = Point.Point(0, 0, 0)
        # self._left_shank_sensorB = Point.Point(0, 0, 0)
        # self._right_thigh_sensorF = Point.Point(0, 0, 0)
        # self._right_thigh_sensorB = Point.Point(0, 0, 0)
        # self._right_shank_sensorF = Point.Point(0, 0, 0)
        # self._right_shank_sensorB = Point.Point(0, 0, 0)
        rospy.Subscriber("/ambf/env/LeftSideProx/State", SensorState, self.prox_callback)
        rospy.Subscriber("/ambf/env/RightSideProx/State", SensorState, self.prox_callback)

        rospy.Subscriber("/ambf/env/LeftFootProx/State", SensorState, self.left_foot_prox_callback)
        rospy.Subscriber("/ambf/env/RightFootProx/State", SensorState, self.right_foot_prox_callback)

        self._left_thigh_sensorF_sub = message_filters.Subscriber("/ambf/env/FrontSensorLeftThigh/State", RigidBodyState)
        self._left_thigh_sensorB_sub = message_filters.Subscriber("/ambf/env/BackSensorLeftThigh/State", RigidBodyState)
        self._left_shank_sensorF_sub = message_filters.Subscriber("/ambf/env/FrontSensorLeftShank/State", RigidBodyState)
        self._left_shank_sensorB_sub = message_filters.Subscriber("/ambf/env/BackSensorLeftShank/State", RigidBodyState)
        self._right_thigh_sensorF_sub = message_filters.Subscriber("/ambf/env/FrontSensorRightThigh/State", RigidBodyState)
        self._right_thigh_sensorB_sub = message_filters.Subscriber("/ambf/env/BackSensorRightThigh/State", RigidBodyState)
        self._right_shank_sensorF_sub = message_filters.Subscriber("/ambf/env/FrontSensorRightShank/State", RigidBodyState)
        self._right_shank_sensorB_sub = message_filters.Subscriber("/ambf/env/BackSensorRightShank/State", RigidBodyState)
        self._leg_sensor_ls = [self._left_thigh_sensorF_sub, self._left_thigh_sensorB_sub,
                               self._left_shank_sensorF_sub, self._left_shank_sensorB_sub,
                               self._right_thigh_sensorF_sub, self._right_thigh_sensorB_sub,
                               self._right_shank_sensorF_sub, self._right_shank_sensorB_sub]
        self._leg_sensor_cb = message_filters.TimeSynchronizer(self._leg_sensor_ls, 1)
        self._leg_sensor_cb.registerCallback(self.leg_sensor_callback)

        self._left_foot_force_sensor = []
        self._right_foot_force_sensor = []

        self._left_foot_force_sensor.append(Point.Point(0, 0, 0))
        self._left_foot_force_sensor.append(Point.Point(0, 0, 0))
        self._left_foot_force_sensor.append(Point.Point(0, 0, 0))

        self._right_foot_force_sensor.append(Point.Point(0, 0, 0))
        self._right_foot_force_sensor.append(Point.Point(0, 0, 0))
        self._right_foot_force_sensor.append(Point.Point(0, 0, 0))

        self._left_foot_sensor1_sub = message_filters.Subscriber("/ambf/env/SensorLeftFoot1Tab/State", RigidBodyState)
        self._left_foot_sensor2_sub = message_filters.Subscriber("/ambf/env/SensorLeftFoot2Tab/State", RigidBodyState)
        self._left_foot_sensor3_sub = message_filters.Subscriber("/ambf/env/SensorLeftFoot3Tab/State", RigidBodyState)
        self._right_foot_sensor1_sub = message_filters.Subscriber("/ambf/env/SensorRightFoot1Tab/State", RigidBodyState)
        self._right_foot_sensor2_sub = message_filters.Subscriber("/ambf/env/SensorRightFoot2Tab/State", RigidBodyState)
        self._right_foot_sensor3_sub = message_filters.Subscriber("/ambf/env/SensorRightFoot3Tab/State", RigidBodyState)
        self._foot_sensor_ls = [self._left_foot_sensor1_sub, self._left_foot_sensor2_sub, self._left_foot_sensor3_sub,
                                self._right_foot_sensor1_sub, self._right_foot_sensor2_sub, self._right_foot_sensor3_sub]
        self._foot_sensor_cb = message_filters.TimeSynchronizer(self._foot_sensor_ls, 1)
        self._foot_sensor_cb.registerCallback(self.foot_sensor_callback)
        self._right_foot_prox = SensorState()
        self._left_foot_prox = SensorState()
        self._updater.start()

    def left_foot_prox_callback(self, msg):
        self._left_foot_prox = msg

    def right_foot_prox_callback(self, msg):
        self._right_foot_prox = msg

    def check_left_foot_collision(self):
        return any(self._left_foot_prox.triggered)

    def check_foot_collision(self):
        return any(self._left_foot_prox.triggered)

    def get_right_foot_collision_distance(self):
        dist = self._left_foot_prox.measurement
        raduis = self._left_foot_prox.range[0]
        return dist[0] - raduis

    def get_left_foot_collision_distance(self):
        dist = self._right_foot_prox.measurement
        raduis = self._right_foot_prox.range[0]
        return dist[0] - raduis

    def prox_callback(self, msg):

        pos = msg.pose
        name = msg.name.data
        ranges = msg.measurement
        raduis = msg.range[0]
        parent_name = msg.parent_name
        sensed_objects = msg.sensed_objects
        theta = np.linspace(-np.pi, -0.70 * np.pi, 75)
        cloud = PointCloud()
        for rad, angle, obj in zip(ranges, theta, sensed_objects):
            point = Point32()
            if "Rob" in obj.data:
                r = 0
            else:
                r = rad
            point.y = (raduis - r) * np.cos(angle) + pos.position.y
            point.z = (raduis - r) * np.sin(angle) + pos.position.z
            point.x = pos.position.x
            cloud.points.append(point)

        cloud.header.stamp = rospy.Time.now()
        cloud.header.frame_id = "/exoskeleton"
        self.prox[name].publish(cloud)

    def leg_sensor_callback(self, flt, blt, fls, bls, frt, brt, frs, brs):
        force_flt = Point.Point(flt.wrench.force.x, flt.wrench.force.y, flt.wrench.force.z)
        force_blt = Point.Point(blt.wrench.force.x, blt.wrench.force.y, blt.wrench.force.z)
        force_fls = Point.Point(fls.wrench.force.x, fls.wrench.force.y, fls.wrench.force.z)
        force_bls = Point.Point(bls.wrench.force.x, bls.wrench.force.y, bls.wrench.force.z)
        force_frt = Point.Point(frt.wrench.force.x, frt.wrench.force.y, frt.wrench.force.z)
        force_brt = Point.Point(brt.wrench.force.x, brt.wrench.force.y, brt.wrench.force.z)
        force_frs = Point.Point(frs.wrench.force.x, frs.wrench.force.y, frs.wrench.force.z)
        force_brs = Point.Point(brs.wrench.force.x, brs.wrench.force.y, brs.wrench.force.z)

        self._left_leg.hip.force = force_flt
        self._left_leg.knee.force = force_fls
        self._right_leg.hip.force = force_frt
        self._right_leg.knee.force = force_frs

        # self._left_thigh_sensorF = force_flt
        # self._left_thigh_sensorB = force_blt
        # self._left_shank_sensorF = force_fls
        # self._left_shank_sensorB = force_bls
        # self._right_thigh_sensorF = force_frt
        # self._right_thigh_sensorB = force_brt
        # self._right_shank_sensorF = force_frs
        # self._right_shank_sensorB = force_brs

    def foot_sensor_callback(self, lf1, lf2, lf3, rf1, rf2, rf3):
        force_lf1 = Point.Point(lf1.wrench.force.x, lf1.wrench.force.y, lf1.wrench.force.z)
        force_lf2 = Point.Point(lf2.wrench.force.x, lf2.wrench.force.y, lf2.wrench.force.z)
        force_lf3 = Point.Point(lf3.wrench.force.x, lf3.wrench.force.y, lf3.wrench.force.z)
        force_rf1 = Point.Point(rf1.wrench.force.x, rf1.wrench.force.y, rf1.wrench.force.z)
        force_rf2 = Point.Point(rf2.wrench.force.x, rf2.wrench.force.y, rf2.wrench.force.z)
        force_rf3 = Point.Point(rf3.wrench.force.x, rf3.wrench.force.y, rf3.wrench.force.z)

        self.left_foot_force_sensor = [force_lf1, force_lf2, force_lf3]
        self.right_foot_force_sensor = [force_rf1, force_rf2, force_rf3]

    def calculate_dynamics(self, qdd):
        tau = np.asarray([0.0] * self._joint_num)
        rbdl.InverseDynamics(self.rbdl_model, self.q[0:6], self.qd[0:6], qdd[0:6], tau)
        return tau

    def grav(self, q ):
        tau = np.asarray([0.0] * self._joint_num)
        qd = qdd = np.asarray([0.0] * self._joint_num)
        rbdl.InverseDynamics(self.rbdl_model, q, qd, qdd, tau)
        return tau

    def dynamic_model(self):
        # add in mass and height params
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
        parent_dist = {}

        parent_dist["hip"] = np.array([0.0, 0.0, 0.0])

        parent_dist["left_thigh"] = np.array([0.237, -0.124, -0.144])
        parent_dist["left_shank"] = np.array([0.033, -0.03, -0.436])
        parent_dist["left_foot"] = np.array([0.02, -0.027, -0.39])

        parent_dist["right_thigh"] = np.array([-0.237, -0.124,  -0.144])
        parent_dist["right_shank"] = np.array([0.033, -0.03,  -0.436])
        parent_dist["right_foot"] = np.array([0.02, -0.027,  -0.39])



        inertia["hip"] = np.diag([ 0.0,0.0,0.0])

        inertia["left_thigh"] = np.diag([0.0, 0.0, 0.07])
        inertia["left_shank"] = np.diag([0.18, 0.18, 0.0])
        inertia["left_foot"] = np.diag([0.07, 0.07, 0.0])

        inertia["right_thigh"] = np.diag([0.0, 0.00, 0.07])
        inertia["right_shank"] = np.diag([0.18, 0.18, 0.0])
        inertia["right_foot"] = np.diag([0.07, 0.07, 0.0])

        com["hip"] = np.array([0.00, -0.02, 0.18])
        com["left_thigh"] = np.array([0.02, 0.01,  -0.09])
        com["left_shank"] = np.array([-0.02, -0.007, 0.06])
        com["left_foot"] = np.array([0.08, -0.06, 0.04])

        com["right_thigh"] = np.array([-0.02, 0.01, -0.09])
        com["right_shank"] = np.array([0.02, -0.007, 0.06])
        com["right_foot"] = np.array([0.08, -0.06, 0.04])

        hip_body = rbdl.Body.fromMassComInertia(mass["hip"], com["hip"], inertia["hip"])
        for segs in segments:
            bodies["right_" + segs] = rbdl.Body.fromMassComInertia(mass["right_" + segs], com["right_" + segs], inertia["right_" + segs])
            bodies["left_" + segs] = rbdl.Body.fromMassComInertia(mass["left_" + segs], com["left_" + segs], inertia["left_" + segs])

        xtrans = rbdl.SpatialTransform()
        xtrans.r = np.array([0.0, 0.0, 0.0])
        xtrans.E = np.eye(3)

        self.hip = model.AddBody(0, xtrans, rbdl.Joint.fromJointType("JointTypeFixed"), hip_body,"hip")
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

        # constraint_set_right = rbdl.ConstraintSet()
        # constraint_set_left = rbdl.ConstraintSet()
        # constraint_set_both = rbdl.ConstraintSet()
        #
        # constraint_set_right.AddContactConstraint(id_r, heel_point, np.array([1., 0., 0.]), "right_heel_x")
        # constraint_set_right.AddContactConstraint(id_r, heel_point, np.array([0., 1., 0.]), "right_heel_y")
        #
        # constraint_set_left.AddContactConstraint(id_l, heel_point, np.array([1., 0., 0.]), "left_heel_x")
        # constraint_set_left.AddContactConstraint(id_l, heel_point, np.array([0., 1., 0.]), "left_heel_y")
        #
        # constraint_set_both.AddContactConstraint(id_r, heel_point, np.array([1., 0., 0.]), "right_heel_x")
        # constraint_set_both.AddContactConstraint(id_r, heel_point, np.array([0., 1., 0.]), "right_heel_y")
        # constraint_set_both.AddContactConstraint(id_r, heel_point, np.array([0., 0., 1.]), "right_heel_z")
        #
        # constraint_set_both.AddContactConstraint(id_l, heel_point, np.array([1., 0., 0.]), "left_heel_x")
        # constraint_set_both.AddContactConstraint(id_l, heel_point, np.array([0., 1., 0.]), "left_heel_y")
        # constraint_set_both.AddContactConstraint(id_l, heel_point, np.array([0., 0., 1.]), "left_heel_z")
        #
        # constraint_set_right.Bind(model)
        # constraint_set_left.Bind(model)
        # constraint_set_both.Bind(model)

        x = []
        y = []
        return model

    def fk(self):
        fk = {}

        point_local = np.array([0.0, 0.0, 0.0])

        data = rbdl.CalcBodyToBaseCoordinates(self.rbdl_model, self.q, self.left_thigh, point_local)
        fk["left_hip"] = Point.Point(data[0], data[1], data[2])
        data = rbdl.CalcBodyToBaseCoordinates(self.rbdl_model, self.q, self.left_shank, point_local)
        fk["left_knee"] = Point.Point(data[0], data[1], data[2])
        data = rbdl.CalcBodyToBaseCoordinates(self.rbdl_model, self.q, self.left_foot, point_local)
        fk["left_ankle"] = Point.Point(data[0], data[1], data[2])

        data = rbdl.CalcBodyToBaseCoordinates(self.rbdl_model, self.q, self.right_thigh, point_local)
        fk["right_hip"] = Point.Point(data[0], data[1], data[2])
        data = rbdl.CalcBodyToBaseCoordinates(self.rbdl_model, self.q, self.right_shank, point_local)
        fk["right_knee"] = Point.Point(data[0], data[1], data[2])
        data = rbdl.CalcBodyToBaseCoordinates(self.rbdl_model, self.q, self.right_foot, point_local)
        fk["right_ankle"] = Point.Point(data[0], data[1], data[2])

        q_left = self.get_left_leg().ankle.angle.z
        q_right = self.get_right_leg().ankle.angle.z
        fk["left_toe"] = Point.Point(0, 0, 0)
        fk["left_toe"].x = fk["left_ankle"].x - 0.8 * (8.0 / 100.0) * self._height * np.cos(q_left)
        fk["left_toe"].y = fk["left_ankle"].y - 0.8 * (8.0 / 100.0) * self._height * np.cos(q_left)
        fk["left_toe"].z = fk["left_ankle"].z - 0.05 + 0.8 * (8.0 / 100.0) * self._height * np.sin(q_left)

        fk["left_heel"] = Point.Point(0, 0, 0)
        fk["left_heel"].x = fk["left_ankle"].x + 0.2 * (8.0 / 100.0) * self._height * np.cos(q_left)
        fk["left_heel"].y = fk["left_ankle"].y + 0.2 * (8.0 / 100.0) * self._height * np.cos(q_left)
        fk["left_heel"].z = fk["left_ankle"].z - 0.05 + 0.2 * (8.0 / 100.0) * self._height * np.sin(q_left)

        fk["right_toe"] = Point.Point(0, 0, 0)
        fk["right_toe"].x = fk["right_ankle"].x - 0.8 * (8.0 / 100.0) * 1.57 * np.cos(q_right)
        fk["right_toe"].y = fk["right_ankle"].y - 0.8 * (8.0 / 100.0) * 1.57 * np.cos(q_right)
        fk["right_toe"].z = fk["right_ankle"].z - 0.05 + 0.8 * (8.0 / 100.0) * self._height * np.sin(q_right)

        fk["right_heel"] = Point.Point(0, 0, 0)
        fk["right_heel"].x = fk["right_ankle"].x + 0.2 * (8.0 / 100.0) * self._height * np.cos(q_right)
        fk["right_heel"].y = fk["right_ankle"].y + 0.2 * (8.0 / 100.0) * self._height * np.cos(q_right)
        fk["right_heel"].z = fk["right_ankle"].z - 0.05 + 0.2 * (8.0 / 100.0) * self._height * np.sin(q_right)

        return fk

    def stance_trajectory(self, tf=2, dt=0.01):
        hip = Model.get_traj(0.0, -0.5, 0.0, 0.0, tf, dt)
        knee = Model.get_traj(0.0, 0.50, 0.0, 0., tf, dt)
        ankle = Model.get_traj(-0.349, -0.2, 0.0, 0.0, tf, dt)
        return hip, knee, ankle

    def walk_init_trajectory(self, tf=2, dt=0.01):
        hip = Model.get_traj(0.0, 0.3234, 0.0, 0.0, tf, dt)
        knee = Model.get_traj(0.0, 0.815, 0.0, 0., tf, dt)
        ankle = Model.get_traj(-0.349, 0.07, 0.0, 0.0, tf, dt)
        return hip, knee, ankle

    def standing_to_sitting_trajectory(self, ip, tf=2, dt=0.01):
        # exo_hip = self.handle.get_pos()
        # exo_hip_x = Model.get_traj(exo_hip.x, exo_hip.x, 0.0, 0.0, tf, dt)
        # exo_hip_y = Model.get_traj(exo_hip.y, exo_hip.y + .5, 0.0, 0.0, tf, dt)
        # exo_hip_z = Model.get_traj(exo_hip.z, exo_hip.z - .25, 0.0, 0.0, tf, dt)
        hip = Model.get_traj(-0.5, -1.35, 0.0, 0.0, tf, dt)
        knee = Model.get_traj(0.5, 1.54, 0.0, 0., tf, dt)
        ankle = Model.get_traj(-0.2, -0.04, 0.0, 0.0, tf, dt)
        return  hip, knee, ankle #, exo_hip_x, exo_hip_y, exo_hip_z,

    def sitting_to_standing_trajectory(self, ip, tf=2, dt=0.01):
        # -0.66
        #.77
        #-0.32
        hip = Model.get_traj(-1.35, -0.5, 0.0, 0.0, tf, dt)
        knee = Model.get_traj(1.54, 0.5, 0.0, 0.0, tf, dt)
        ankle = Model.get_traj(-0.04, -0.2, 0.0, 0.0, tf, dt)
        return hip, knee, ankle

    def get_runner(self):
        project_root = dirname(dirname(__file__))
        config_path = join(project_root, 'config/gotozero.pickle')
        return TPGMMRunner.TPGMMRunner(config_path)

    def get_walker(self):
        project_root = dirname(dirname(__file__))
        config_path = join(project_root, 'config/walk2.pickle')
        return TPGMMRunner.TPGMMRunner(config_path)

    def linearize(self):
        pass

    def state(self, q, qd ):
        self.get_left_leg.hip.angle.z = q[0]
        self.get_left_leg.knee.angle.z = q[1]
        self.get_left_leg.ankle.angle.z = q[2]

        self.get_right_leg.hip.angle.z = q[3]
        self.get_right_leg.knee.angle.z = q[4]
        self.get_right_leg.ankle.angle.z = q[5]

    def get_right_leg(self):
        """
        :return:
        """
        return self._right_leg

    def get_left_leg(self):
        """
        :return:
        """
        return self._left_leg


    # def get_leg_sensors(self):
    #     left_leg_sensors = [self._left_leg.hip.force, self._left_leg.knee.force]
    #     right_leg_sensors = [self._right_leg.hip.force, self._right_leg.knee.force]
    #     return left_leg_sensors, right_leg_sensors

    @property
    def left_foot_force_sensor(self):
        return self._left_foot_force_sensor

    @property
    def right_foot_force_sensor(self):
        return self._right_foot_force_sensor

    @left_foot_force_sensor.setter
    def left_foot_force_sensor(self, value):
        self._left_foot_force_sensor = value

    @right_foot_force_sensor.setter
    def right_foot_force_sensor(self, value):
        self._right_foot_force_sensor = value

    # def get_foot_sensors(self):
    #     left_foot_sensors = [self._left_foot_sensor1, self._left_foot_sensor2, self._left_foot_sensor3]
    #     right_foot_sensors = [self._right_foot_sensor1, self._right_foot_sensor2, self._right_foot_sensor3]
    #     return left_foot_sensors, right_foot_sensors


    def leg_inverse_kinimatics(self, toe, hip_location):

        l1 = 436.0
        l2 = 390.0
        l3 = 98.0
        l4 = 217.0

        x = toe[0] - hip_location[0] - abs(l4)
        y = toe[1] - hip_location[1] + abs(l3)

        num = x*x + y*y - l1**2 - l2**2
        dem = 2*l1*l2

        q2 = np.arctan2(-np.sqrt(1 - (num / dem)**2), (num / dem))
        q2 = np.nan_to_num(q2)
        q1 = -(np.nan_to_num(np.arctan2(y, x) - np.arctan2(l2*np.sin(q2), l1 + l2*np.cos(q2))) + 0.5*np.pi)
        q3 = -(np.nan_to_num(2*np.pi - q1 - q2) - 2*np.pi) + 0.75*np.pi

        return [q1, -q2, q3]

