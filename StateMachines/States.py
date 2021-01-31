#!/usr/bin/env python
import smach
import rospy
import numpy as np
from sensor_msgs.msg import JointState
from ambf_walker.msg import DesiredJoints
from GaitAnaylsisToolkit.LearningTools.Runner import TPGMMRunner
from std_msgs.msg import Float32MultiArray
from Model import Model
from std_msgs.msg import Empty,String
import matplotlib.pyplot as plt
from ambf_walker.srv import DesiredJointsCmdRequest, DesiredJointsCmd
#from ilqr.controller import RecedingHorizonController
#from ilqr.cost import PathQsRCost
#from ilqr import iLQR
#from ilqr.dynamics import FiniteDiffDynamics
from GaitAnaylsisToolkit.LearningTools.Runner import GMMRunner
import numpy.polynomial.polynomial as poly
from os.path import dirname, join

class Initialize(smach.State):

    def __init__(self, model, outcomes=['Initializing', 'Initialized']):

        smach.State.__init__(self, outcomes=outcomes)
        rospy.wait_for_service('joint_cmd')

        self.send = rospy.ServiceProxy('joint_cmd', DesiredJointsCmd)
        self._model = model
        self.rate = rospy.Rate(1000)
        tf = 6.0
        dt = 0.01
        self.hip, self.knee, self.ankle = self._model.stance_trajectory(tf=tf, dt=dt)
        self.msg = DesiredJoints()
        self.pub = rospy.Publisher(self._model.model_name + "_set_points", DesiredJoints, queue_size=1)

        self.total = tf / dt
        self.count = 0

    def execute(self, userdata):

        self._model.handle.set_rpy(0.25, 0, 0)
        self._model.handle.set_pos(0.0, -0.5, 1.0)

        if self.count <= self.total - 1:

            # q = np.array([self.hip["q"][self.count].item(), self.knee["q"][self.count].item(),
            #               self.ankle["q"][self.count].item(),
            #               self.hip["q"][self.count].item(), self.knee["q"][self.count].item(),
            #               self.ankle["q"][self.count].item(), 0.0])
            #
            # qd = np.array([self.hip["qd"][self.count].item(), self.knee["qd"][self.count].item(),
            #                self.ankle["qd"][self.count].item(),
            #                self.hip["qd"][self.count].item(), self.knee["qd"][self.count].item(),
            #                self.ankle["qd"][self.count].item(), 0.0])
            #
            # qdd = np.array([self.hip["qdd"][self.count].item(), self.knee["qdd"][self.count].item(),
            #                 self.ankle["qdd"][self.count].item(),
            #                 self.hip["qdd"][self.count].item(), self.knee["qdd"][self.count].item(),
            #                 self.ankle["qdd"][self.count].item(), 0.0])

            self.count += 1
            # self.msg.q = q
            # self.msg.qd = qd
            # self.msg.qdd = qdd
            # self.msg.controller = "Dyn"
            # self.pub.publish(self.msg)
            # self.send(q, qd, qdd, "Dyn", [])
            self.rate.sleep()

            return 'Initializing'
        else:
            return "Initialized"

class WalkInit(smach.State):

    def __init__(self, model, outcomes=['WalkInitializing', 'WalkInitialized']):

        smach.State.__init__(self, outcomes=outcomes)
        rospy.wait_for_service('joint_cmd')

        self.send = rospy.ServiceProxy('joint_cmd', DesiredJointsCmd)
        self._model = model
        self.rate = rospy.Rate(100)
        tf = 2.0
        dt = 0.01
        self.hip, self.knee, self.ankle = self._model.walk_init_trajectory()
        self.msg = DesiredJoints()
        self.pub = rospy.Publisher(self._model.model_name + "_set_points", DesiredJoints, queue_size=1)

        self.total = tf / dt
        self.count = 0

    def execute(self, userdata):

        self._model.handle.set_rpy(0.25, 0, 0)
        self._model.handle.set_pos(0.0, 0, 1.0)

        if self.count <= self.total - 1:

            q = np.array([self.hip["q"][self.count].item(), self.knee["q"][self.count].item(),
                          self.ankle["q"][self.count].item(),
                          self.hip["q"][self.count].item(), self.knee["q"][self.count].item(),
                          self.ankle["q"][self.count].item(), 0.0])

            qd = np.array([self.hip["qd"][self.count].item(), self.knee["qd"][self.count].item(),
                           self.ankle["qd"][self.count].item(),
                           self.hip["qd"][self.count].item(), self.knee["qd"][self.count].item(),
                           self.ankle["qd"][self.count].item(), 0.0])

            qdd = np.array([self.hip["qdd"][self.count].item(), self.knee["qdd"][self.count].item(),
                            self.ankle["qdd"][self.count].item(),
                            self.hip["qdd"][self.count].item(), self.knee["qdd"][self.count].item(),
                            self.ankle["qdd"][self.count].item(), 0.0])

            self.count += 1
            self.msg.q = q
            self.msg.qd = qd
            self.msg.qdd = qdd
            self.msg.controller = "Dyn"
            self.pub.publish(self.msg)
            #self.send(q, qd, qdd, "Dyn", [])
            self.rate.sleep()

            return 'WalkInitializing'
        else:
            return "WalkInitialized"

class Main(smach.State):

    def __init__(self, model,outcomes=["Poly"]):
        smach.State.__init__(self, outcomes=outcomes)
        rospy.Subscriber("Mode", String, callback=self.mode_cb)
        rospy.wait_for_service('joint_cmd')

        self.send = rospy.ServiceProxy('joint_cmd', DesiredJointsCmd)
        self._model = model
        self.have_msg = False
        self.msg = String
        self.Rate = rospy.Rate(100)

    def mode_cb(self, msg):

        if not self.have_msg:
            self.msg = msg
            self.have_msg = True

    def execute(self, userdata):

        rate = rospy.Rate(1000)
        self.have_msg = False
        while not self.have_msg:
            rate.sleep()

        return self.msg.data

class DMP(smach.State):

    def __init__(self, model,outcomes=["stepping", "stepped"]):
        smach.State.__init__(self, outcomes=outcomes)
        rospy.wait_for_service('joint_cmd')
        self.send = rospy.ServiceProxy('joint_cmd', DesiredJointsCmd)
        self._model = model
        self.runner = self._model.get_runner()
        self.rate = rospy.Rate(100)
        self.msg = DesiredJoints()
        self.pub = rospy.Publisher("set_points", DesiredJoints, queue_size=1)
        self.count = 0

    def execute(self, userdata):

        count = self.count

        if count == 0:
            start = []
            for q in self._model.q[0:6]:
                start.append(np.array([q]))
            print(start)
            print(self.runner.x)
            self.runner.update_start(start)

        if count < self.runner.get_length():

            self.runner.step()
            x = self.runner.x
            dx = self.runner.dx
            print(dx)
            ddx = self.runner.ddx
            q = np.append(x, [0.0])
            qd = np.append(dx, [0.0])
            qdd = np.append(ddx, [0.0])
            self.msg.q = q
            self.msg.qd = qd
            self.msg.qdd = qdd
            self.msg.controller = "Dyn"
            self.pub.publish(self.msg)
            #self.send(q, qd, qdd,"Dyn",[])
            self.count += 1
            self.rate.sleep()
            return "stepping"
        else:
            self.count = 0
            return "stepped"

class Walk(smach.State):

    def __init__(self, model,outcomes=["walking", "walked"]):
        smach.State.__init__(self, outcomes=outcomes)
        rospy.wait_for_service('joint_cmd')
        self.send = rospy.ServiceProxy('joint_cmd', DesiredJointsCmd)
        self._model = model
        self.runner = self._model.get_walker()
        self.rate = rospy.Rate(1000)
        self.set_msg = DesiredJoints()
        self.error_msg = DesiredJoints()
        self.set_pub = rospy.Publisher(self._model.model_name + "_set_points", DesiredJoints, queue_size=1)
        self.error_pub = rospy.Publisher(self._model.model_name + "_error", DesiredJoints, queue_size=1)
        self.state_pub = rospy.Publisher("state", String, queue_size=1)
        self.restart_sub = rospy.Subscriber("restart", String, self.restart_cb)
        self.start = ""
        self.count = 0
        self.num_cycles = 0
        self.execute_start = True
        self.last_x = []

    def restart_cb(self, msg):
        self.num_cycles = 0

    def execute(self, userdata):
        count = self.count

        if self.execute_start and count == 0:
            self.execute_start = False
            start = []
            for q in self._model.q[0:6]:
                start.append(np.array([q]))
            self.runner.update_start(start)
        elif count == 0:
            start = []
            for q in self.last_x:
                start.append(np.array([q[0]]))
            self.runner.update_start(start)


        # print(self.runner.get_length())
        if count < self.runner.get_length():
            if self.num_cycles == 3:
                self.state_pub.publish("walking")
            self.runner.step()
            x = self.runner.x
            dx = self.runner.dx
            ddx = self.runner.ddx
            q = np.append(x, [0.0])
            qd = np.append(dx, [0.0])
            qdd = np.append(ddx, [0.0])
            # self._model.handle.set_multiple_joint_pos(q, [3, 1, 2, 6, 4, 5, 0])
            self.set_msg.q = q
            self.set_msg.qd = qd
            self.set_msg.qdd = qdd
            self.set_msg.controller = "Dyn"
            self.set_pub.publish(self.set_msg)
            self.error_msg.q = q - self._model.q
            self.error_msg.qd = qd
            self.error_msg.qdd = qdd
            self.error_msg.controller = "Dyn"
            self.error_pub.publish(self.error_msg)
            self.send(q, qd, qdd, "Dyn", [])
            self.count += 1
            # print(count)
            self.rate.sleep()
            self.last_x = x
            # self.state_pub.publish("walking")
            return "walking"
        else:
            self.count = 0
            if self.num_cycles == 3:
                self.state_pub.publish("walked")
                self.num_cycles = 0
            else:
                self.num_cycles += 1
            self.runner.reset()
            # can torques be released?
            # if self.num_cycles == 5:
            #     self.state_pub.publish("walked")
            return "walking"

class Listening(smach.State):

    def __init__(self, model, outcomes=["Sending", "Waiting"]):
        smach.State.__init__(self, outcomes=outcomes, output_keys=['q'])


        rospy.Subscriber("Traj", DesiredJoints, callback=self.traj_cb)
        self._model = model
        self.have_msg = False
        self.Rate = rospy.Rate(100)
        self.q = []

    def traj_cb(self, msg):
        self.q = []
        if not self.have_msg:
            current_joints = self._model.q
            for q, q_d in zip(tuple(current_joints), msg.q):
                self.q.append(Model.get_traj(q, q_d, 0.0, 0.0, 1.0, 0.01))
            self.have_msg = True

    def execute(self, userdata):
        # Your state execution goes here
        userdata.count = 0

        self.Rate.sleep()
        if self.have_msg:
            userdata.q = self.q
            self.have_msg = False
            return "Sending"
        else:
            return "Waiting"

class Follow(smach.State):

    def __init__(self, model, outcomes=['Following', 'Followed']):

        smach.State.__init__(self, outcomes=outcomes,
                              input_keys=['q'],
                              output_keys=['q'])

        rospy.wait_for_service('joint_cmd')
        self.send = rospy.ServiceProxy('joint_cmd', DesiredJointsCmd)
        self._model = model
        self.rate = rospy.Rate(100)

        self.pub = rospy.Publisher("set_points", DesiredJoints, queue_size=1)
        self.count = 0

    def execute(self, userdata):

        q = userdata.q
        msg = DesiredJoints()
        count = self.count
        if count <= len(q[0]["q"]) - 1:

            q_d = np.array([q[0]["q"][count].item(), q[1]["q"][count].item(),
                          q[2]["q"][count].item(), q[3]["q"][count].item(),
                          q[4]["q"][count].item(), q[5]["q"][count].item(), 0.0])

            qd_d = np.array([q[0]["qd"][count].item(), q[1]["qd"][count].item(),
                          q[2]["qd"][count].item(), q[3]["qd"][count].item(),
                          q[4]["qd"][count].item(), q[5]["qd"][count].item(), 0.0])

            qdd_d = np.array([q[0]["qdd"][count].item(), q[1]["qdd"][count].item(),
                          q[2]["qdd"][count].item(), q[3]["qdd"][count].item(),
                          q[4]["qdd"][count].item(), q[5]["qdd"][count].item(), 0.0])

            msg.q = q_d
            msg.qd = qd_d
            msg.qdd = qdd_d
            msg.controller = "Dyn"
            #self.send(q_d, qd_d, qdd_d,"Dyn", [])
            self.pub.publish(msg)
            self.count += 1
            self.rate.sleep()
            return "Following"
        else:
            self.count = 0
            return "Followed"


class LowerBody(smach.State):

    def __init__(self, model, outcomes=['Lowering', 'Lowered']):

        smach.State.__init__(self, outcomes=outcomes)
        self._model = model
        self.rate = rospy.Rate(1)
        self.step = 0.00000000001
        self.final_height = -0.75

    def execute(self, userdata):

        self.rate.sleep()
        current = self._model.handle.get_pos().z

        if current > self.final_height:
            self._model.handle.set_pos(0, 0.0, current-self.step)
            self._model.handle.set_rpy(0.25, 0, 0)
            return 'Lowering'
        else:
            #self._model.handle.set_rpy(0.25, 0, 0)
            self._model.handle.set_force(0.0, 0.0, 0.0)
            return "Lowered"


class LQR(smach.State):

    def __init__(self, model, outcomes=["LQRing", "LQRed"]):
        smach.State.__init__(self, outcomes=outcomes)
        rospy.wait_for_service('joint_cmd')
        self.send = rospy.ServiceProxy('joint_cmd', DesiredJointsCmd)
        self._model = model
        self.rate = rospy.Rate(100)
        project_root = dirname(dirname(__file__))
        self.runner = self._model.get_walker()
        file = join(project_root, 'config/tau.npy')
        with open(file, 'rb') as f:
            self.us2 = np.load(f)
        self.pub = rospy.Publisher("set_points2", DesiredJoints, queue_size=1)
        self.my_joints = rospy.Publisher("my_joints", DesiredJoints, queue_size=1)
        self.count = 0

    def execute(self, userdata):

        if self.count < self.runner.get_length():
            self.runner.step()
            x = self.runner.x
            dx = self.runner.dx
            ddx = self.us2[self.count]
            q = np.append(x, [0.0])
            qd = np.append(dx, [0.0])
            qdd = np.append(ddx, [0.0])
            msg = DesiredJoints()
            msg.q = q.tolist()
            msg.qd = qd.tolist()
            msg.qdd = qdd.tolist()
            msg.other = q.tolist()
            msg.controller = "FF"
            self.send(q, qd, qdd, "FF", [self.count])
            msg = DesiredJoints()
            joints = DesiredJoints()
            msg.q = q.tolist()
            joints.q = self._model.q.tolist()
            self.pub.publish(msg)
            self.my_joints.publish(joints)
            self.rate.sleep()
            self.count += 1
            return "LQRing"
        else:
            return "LQRed"

class StairDMP(smach.State):

    def __init__(self, model,outcomes=["stairing", "staired"]):
        smach.State.__init__(self, outcomes=outcomes)
        rospy.wait_for_service('joint_cmd')
        self.send = rospy.ServiceProxy('joint_cmd', DesiredJointsCmd)
        self._model = model
        project_root = dirname(dirname(__file__))
        fileZ = join(project_root, 'config/toeZ_all.pickle')
        fileY = join(project_root, 'config/toeY_all.pickle')
        self.runnerZ = GMMRunner.GMMRunner(fileZ)  # make_toeZ([file1, file2], hills3, nb_states, "toe_IK")
        self.runnerY = GMMRunner.GMMRunner(fileY)  # make_toeY([file1, file2], hills3, nb_states, "toe_IK")
        self.rate = rospy.Rate(10)
        self.msg = DesiredJoints()
        self.pub = rospy.Publisher("set_points", DesiredJoints, queue_size=1)
        self.count = 0
        self.init_joint_angles = []

    def smooth_curve(self, data, t, order):

        coefs = poly.polyfit(t, data, order)
        ffit = poly.Polynomial(coefs)  # instead of np.poly1d
        return ffit(t)

    def execute(self, userdata):

        if self.count == 0:
            self.init_joint_angles = self._model.q

            self.runnerZ.update_start(0)
            self.runnerZ.update_goal(192)

            self.runnerY.update_start(-225.0)
            self.runnerY.update_goal(258.0)


            self.hip_angles = []
            self.knee_angles = []
            self.ankle_angles = []

            pathZ = self.runnerZ.run()
            pathY = self.runnerY.run()
            self.hip_angles.append(0.0)
            self.knee_angles.append(0.0)
            self.ankle_angles.append(-0.2)
            for y, x in zip(pathZ, pathY):#-483.4
                joint_angle = self._model.leg_inverse_kinimatics([y, x], hip_location=[-483.0, 960.67])
                self.hip_angles.append(joint_angle[0][0])
                self.knee_angles.append(joint_angle[1][0])
                self.ankle_angles.append(-0.2)

            t = np.linspace(0, 100, len(self.knee_angles))

            self.hip_angles = self.smooth_curve(self.hip_angles, t, 6)
            self.knee_angles = self.smooth_curve(self.knee_angles, t, 6)
            self.ankle_angles = self.smooth_curve(self.ankle_angles, t, 6)

            self.hip_vel = []
            self.knee_vel = []
            self.ankle_vel = []

            tf = len(self.hip_angles)
            V_hip = 0.001
            V_knee = 0.001
            V_ankle = 0.001
            alpha = 5.0

            for t in range(tf):

                if  0 <= t and  t <= int(tf/alpha):
                    self.hip_vel.append((alpha*V_hip*t)/tf)
                    self.knee_vel.append(-(alpha*V_knee*t)/tf)
                    self.ankle_vel.append((alpha*V_knee*t)/tf)

                if int(tf/alpha) < t and  t <= int((alpha*tf - tf )/alpha):
                    self.hip_vel.append(V_hip)
                    self.knee_vel.append(-V_knee)
                    self.ankle_vel.append(V_knee)

                if int((alpha*tf - tf )/alpha ) < t and t < tf:
                    self.hip_vel.append((-alpha * V_hip * t) / tf + alpha*V_hip)
                    self.knee_vel.append(-((-alpha * V_knee * t) / tf + alpha*V_knee))
                    self.ankle_vel.append((-alpha * V_knee * t) / tf + alpha*V_ankle)
            #
            self.hip_vel.append(0.0)
            self.knee_vel.append(0.0)
            self.ankle_vel.append(0.0)
            fig, ax =  plt.subplots(3)
            ax[0].plot(self.hip_angles)
            ax[1].plot(self.knee_angles)
            ax[2].plot(self.ankle_angles)
            plt.show()

        if self.count < self.runnerY.get_length()-2:

            # self.runnerZ.step()
            # self.runnerY.step()
            # x = self.runnerZ.x[0].item()
            # dx = self.runnerZ.dx
            # ddx = self.runnerZ.ddx
            #
            # y = self.runnerY.x[0].item()
            # dt = self.runnerY.dx
            # ddy = self.runnerY.ddx
            # x = self.pathZ[self.count][0]
            # y = self.pathY[self.count][0]
            # joint_angle = self._model.leg_inverse_kinimatics([y, x], hip_location=[-483.4, 960.67])

            q = np.array([self.init_joint_angles[0],
                          self.init_joint_angles[1],
                          -0.2,
                          self.hip_angles[self.count],
                          self.knee_angles[self.count],
                          self.ankle_angles[self.count],
                          0.0])

            qd = np.array([0.0,
                           0.0,
                           0.0,
                           self.hip_vel[self.count],
                           self.knee_vel[self.count],
                           self.ankle_vel[self.count],
                           0.0])

            # qdd = np.array([0.0,
            #                 0.0,
            #                 0.0,
            #                 self.hip_acl[self.count],
            #                 self.knee_acl[self.count],
            #                 self.ankle_acl[self.count],
            #                 0.0])

            #qd = np.array([0.0]*7)
            qdd = np.array([0.0]*7)
            # q[3] = 0# 1.50971 - 0.5*3.14
            # q[4] =  -q[4] # 0# 0.523599
            # q[5] = 0.0
            #self._model.handle.set_multiple_joint_pos(q, [0,1,2,3,4,5,6])
            self.count += 1
            self.msg.q = q
            self.msg.qd = qd
            self.msg.qdd = qdd
            self.msg.controller = "Dyn"
            #self.pub.publish(self.msg)
            self.send(q, qd, qdd, "Dyn", [])
            self.count += 1
            self.rate.sleep()
            return "stairing"
        else:
            self.count = 0

            #plt.plot(self.Zpoints)
            plt.show()
            return "staired"


class Stand2Sit(smach.State):

    def __init__(self, model, outcomes=["sitting", "sat"]):
        smach.State.__init__(self, outcomes=outcomes)
        rospy.wait_for_service('joint_cmd')
        self.send = rospy.ServiceProxy('joint_cmd', DesiredJointsCmd)
        self._model = model

        self.rate = rospy.Rate(10)
        self.tf = 2.0
        self.dt = 0.01
        # ip = self._model.q[3:6]
        # self.hip, self.knee, self.ankle = self._model.standing_to_sitting_trajectory(ip)
        # print(self.hip)
        # print(self.knee)
        # print(self.ankle)

        self.total = self.tf / self.dt

        self.set_msg = DesiredJoints()
        self.error_msg = DesiredJoints()
        self.set_pub = rospy.Publisher(self._model.model_name + "_set_points", DesiredJoints, queue_size=1)
        self.error_pub = rospy.Publisher(self._model.model_name + "_error", DesiredJoints, queue_size=1)
        self.state_pub = rospy.Publisher("state", String, queue_size=1)
        # self.restart_sub = rospy.Subscriber("restart", String, self.restart_cb)
        self.start = ""
        self.count = 0
        self.num_cycles = 0
        self.execute_start = True
        self.last_x = []

    # def restart_cb(self, msg):
    #     self.num_cycles = 0

    def execute(self, userdata):
        # self._model.handle._pose_cmd_set = False
        # self.left_foot.set_pos(0.24, -0.55, -0.24)
        # self.right_foot.set_pos(-0.24, -0.55, -0.24)
        # self._model.handle.set_rpy(1.5, 0, 0)

        if self.count == 0:
            self._model.handle.set_rpy(1.75, 0, 0)

            self._model.handle.set_force(0.0, 0.0, 0.0)
            rospy.sleep(10)
            ip = self._model.q[3:6]
            self.hip, self.knee, self.ankle = self._model.standing_to_sitting_trajectory(ip, tf=self.tf, dt=self.dt)

        # print(self.runner.get_length())
        if self.count < self.total:
            # self._model.handle.set_pos(self.ehx["q"][self.count].item(), self.ehy["q"][self.count].item(), self.ehz["q"][self.count].item())


            q = np.array([self.hip["q"][self.count].item(), self.knee["q"][self.count].item(),
                          self.ankle["q"][self.count].item(),
                          self.hip["q"][self.count].item(), self.knee["q"][self.count].item(),
                          self.ankle["q"][self.count].item(), 0.0])

            qd = np.array([self.hip["qd"][self.count].item(), self.knee["qd"][self.count].item(),
                           self.ankle["qd"][self.count].item(),
                           self.hip["qd"][self.count].item(), self.knee["qd"][self.count].item(),
                           self.ankle["qd"][self.count].item(), 0.0])

            qdd = np.array([self.hip["qdd"][self.count].item(), self.knee["qdd"][self.count].item(),
                            self.ankle["qdd"][self.count].item(),
                            self.hip["qdd"][self.count].item(), self.knee["qdd"][self.count].item(),
                            self.ankle["qdd"][self.count].item(), 0.0])

            # self._model.handle.set_multiple_joint_pos(q, [3, 1, 2, 6, 4, 5, 0])

            self.set_msg.q = q
            self.set_msg.qd = qd
            self.set_msg.qdd = qdd
            self.set_msg.controller = "Dyn"
            self.set_pub.publish(self.set_msg)
            self.error_msg.q = q - self._model.q
            self.error_msg.qd = qd - self._model.qd
            self.error_msg.qdd = qdd
            self.error_msg.controller = "Dyn"
            self.error_pub.publish(self.error_msg)
            self.send(q, qd, qdd, "Dyn", [])
            self.count += 1
            # print(count)
            self.rate.sleep()
            self.last_x = q
            self.state_pub.publish("sitting")
            return "sitting"
        else:
            self.count = 0

            # if self.num_cycles == 3:
            #     self.state_pub.publish("sat")
            #     self.num_cycles = 0
            # else:
            #     self.num_cycles += 1
            # self.runner.reset()
            # can torques be released?
            # if self.num_cycles == 5:
            #     self.state_pub.publish("walked")
            return "sat"


class Sit2Stand(smach.State):

    def __init__(self, model, outcomes=["standing", "stood"]):
        smach.State.__init__(self, outcomes=outcomes)
        rospy.wait_for_service('joint_cmd')
        self.send = rospy.ServiceProxy('joint_cmd', DesiredJointsCmd)
        self._model = model
        self.rate = rospy.Rate(10)
        self.tf = 2.0
        self.dt = 0.01
        # ip = self._model.q[3:6]
        # self.hip, self.knee, self.ankle = self._model.standing_to_sitting_trajectory(ip)
        # print(self.hip)
        # print(self.knee)
        # print(self.ankle)

        self.total = self.tf / self.dt

        self.set_msg = DesiredJoints()
        self.error_msg = DesiredJoints()
        self.set_pub = rospy.Publisher(self._model.model_name + "_set_points", DesiredJoints, queue_size=1)
        self.error_pub = rospy.Publisher(self._model.model_name + "_error", DesiredJoints, queue_size=1)
        self.state_pub = rospy.Publisher("state", String, queue_size=1)
        # self.restart_sub = rospy.Subscriber("restart", String, self.restart_cb)
        self.start = ""
        self.count = 0
        self.num_cycles = 0
        self.execute_start = True
        self.last_x = []
        self.ip = []

    # def restart_cb(self, msg):
    #     self.num_cycles = 0

    def execute(self, userdata):
        # self._model.handle._pose_cmd_set = False
        # self.left_foot.set_pos(0.24, -0.55, -0.24)
        # self.right_foot.set_pos(-0.24, -0.55, -0.24)
        # self._model.handle.set_rpy(0, 0, 0)

        if self.count == 0:
            self._model.handle.set_rpy(1.75, 0, 0)
            # rospy.sleep(2)
            self._model.handle.set_force(0.0, 0.0, 0.0)
            rospy.sleep(10)
            # self._model.handle.set_rpy(1.75, 0, 0)
            # print(self._model.handle.get_pos_command())
            self.ip = self._model.q[3:6]
            self.hip, self.knee, self.ankle = self._model.sitting_to_standing_trajectory(self.ip, tf=self.tf)
            self._model.handle._pose_cmd_set = False

        if self.count < self.total:
            self.state_pub.publish("standing")
            # self._model.handle.set_pos(self.ehx["q"][self.count].item(), self.ehy["q"][self.count].item(), self.ehz["q"][self.count].item())
            # if self.num_cycles == 3:
            #     self.state_pub.publish("sitting")

            q = np.array([self.hip["q"][self.count].item(), self.knee["q"][self.count].item(),
                          self.ankle["q"][self.count].item(),
                          self.hip["q"][self.count].item(), self.knee["q"][self.count].item(),
                          self.ankle["q"][self.count].item(), 0.0])

            qd = np.array([self.hip["qd"][self.count].item(), self.knee["qd"][self.count].item(),
                           self.ankle["qd"][self.count].item(),
                           self.hip["qd"][self.count].item(), self.knee["qd"][self.count].item(),
                           self.ankle["qd"][self.count].item(), 0.0])

            qdd = np.array([self.hip["qdd"][self.count].item(), self.knee["qdd"][self.count].item(),
                            self.ankle["qdd"][self.count].item(),
                            self.hip["qdd"][self.count].item(), self.knee["qdd"][self.count].item(),
                            self.ankle["qdd"][self.count].item(), 0.0])

            # self._model.handle.set_multiple_joint_pos(q, [3, 1, 2, 6, 4, 5, 0])

            self.set_msg.q = q
            self.set_msg.qd = qd
            self.set_msg.qdd = qdd
            self.set_msg.controller = "Dyn"
            self.set_pub.publish(self.set_msg)
            self.error_msg.q = q - self._model.q
            self.error_msg.qd = qd - self._model.qd
            self.error_msg.qdd = qdd
            self.error_msg.controller = "Dyn"
            self.error_pub.publish(self.error_msg)
            # self.send(q, qd, qdd, "Dyn", [])
            self.count += 1
            # print(count)
            self.rate.sleep()
            self.last_x = q
            # self.state_pub.publish("standing")
            return "standing"
        else:
            self.count = 0

            # if self.num_cycles == 3:
            #     self.state_pub.publish("sat")
            #     self.num_cycles = 0
            # else:
            #     self.num_cycles += 1
            # self.runner.reset()
            # can torques be released?
            # if self.num_cycles == 5:
            self.state_pub.publish("stood")
            return "stood"
