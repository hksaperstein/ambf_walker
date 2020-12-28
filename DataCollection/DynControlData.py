#!/usr/bin/env python

import numpy as np
import time
from Model import Exoskeleton, Human
import rospy
from std_msgs.msg import Float32MultiArray, String
from ambf_walker.msg import DesiredJoints
from sensor_msgs.msg import JointState

import matplotlib.pyplot as plt
from matplotlib.lines import Line2D

class DynControlData():

    def __init__(self, ):
        self.q_sub = rospy.Subscriber("exo_q", Float32MultiArray, self.q_cb)
        self.set_points_sub = rospy.Subscriber("exo_set_points", DesiredJoints, self.set_points_cb)
        self.tau_sub = rospy.Subscriber("exo_joint_torque", JointState, self.tau_cb)
        self.state_sub = rospy.Subscriber("state", String, self.state_cb)
        self.restart_pub = rospy.Publisher("restart", String, queue_size=1)
        self.current_q = [0, 0, 0]
        self.current_set_point = None
        self.current_tau = [0, 0, 0]
        self.q = [[], [], []]
        self.set_points = [[], [], []]
        self.tau = [[], [], []]
        self.state = ""

        rospy.init_node("DataPlotter", anonymous=True)

    def q_cb(self, msg):
        if self.state == "walking":
            rh = round(np.rad2deg(msg.data[3]), 2)
            rk = round(np.rad2deg(msg.data[4]), 2)
            ra = round(np.rad2deg(msg.data[5]), 2)
            self.current_q = [rh, rk, ra]


    def set_points_cb(self, msg):
        if self.state == "walking":
            current_q = self.current_q
            current_tau = self.current_tau
            rh = round(np.rad2deg(msg.q[3]), 2)
            rk = round(np.rad2deg(msg.q[4]), 2)
            ra = round(np.rad2deg(msg.q[5]), 2)

            self.set_points[0].append(rh)
            self.set_points[1].append(rk)
            self.set_points[2].append(ra)

            self.q[0].append(current_q[0])
            self.q[1].append(current_q[1])
            self.q[2].append(current_q[2])

            self.tau[0].append(current_tau[0])
            self.tau[1].append(current_tau[1])
            self.tau[2].append(current_tau[2])

    def tau_cb(self, msg):
        if self.state == "walking":

            rh = round(np.rad2deg(msg.effort[3]), 2)
            rk = round(np.rad2deg(msg.effort[4]), 2)
            ra = round(np.rad2deg(msg.effort[5]), 2)
            self.current_tau = [rh, rk, ra]

    def state_cb(self, msg):
        self.state = msg.data

    def main(self):
        self.restart_pub.publish("restart")
        while self.state != "walked":
            pass
        self.q_sub.unregister()
        self.set_points_sub.unregister()
        self.tau_sub.unregister()
        print(len(self.q[0]), len(self.set_points[0]), len(self.tau[0]))
        print(len(self.q[0]) + 1)
        x = [round(100*(i/len(self.q[0])), 2) for i in range(1, len(self.q[0])+1)]

        joint_fig = plt.figure()
        joint_fig.suptitle('Walking Gait Trajectory Control', fontsize=20, verticalalignment='top')
        tau_fig = plt.figure()
        tau_fig.suptitle("Walking Gait Effort", fontsize=20, verticalalignment='top')

        ax1 = joint_fig.add_subplot(311)
        ax1.plot(x, self.q[0], label='Actual')
        ax1.plot(x, self.set_points[0], label='Trajectory')
        ax1.legend()
        ax1.set_title("Hip", fontsize=15, verticalalignment='top')
        ax1.set_ylabel("Joint Position \n (Degrees)", fontsize=12, verticalalignment="bottom")
        ax1.get_xaxis().set_visible(False)

        ax2 = joint_fig.add_subplot(312)
        ax2.plot(x, self.q[1], label='Actual')
        ax2.plot(x, self.set_points[1], label='Trajectory')
        ax2.legend()
        ax2.set_title("Knee", fontsize=15, verticalalignment='top')
        ax2.set_ylabel("Joint Position \n (Degrees)", fontsize=12, verticalalignment="bottom")
        ax2.get_xaxis().set_visible(False)

        ax3 = joint_fig.add_subplot(313)
        ax3.plot(x, self.q[2], label='Actual')
        ax3.plot(x, self.set_points[2], label='Trajectory')
        ax3.legend()
        ax3.set_title("Ankle", fontsize=15, verticalalignment='top')
        ax3.set_ylabel("Joint Position \n (Degrees)", fontsize=12, verticalalignment="bottom")
        ax3.set_xlabel("Walking Gait Cycle Percentage", fontsize=12, verticalalignment="top")

        ax4 = tau_fig.add_subplot(311)
        ax4.plot(x, self.tau[0])
        ax4.set_title("Hip", fontsize=15, verticalalignment='top')
        ax4.set_ylabel("Tau (Nm)",  fontsize=12, verticalalignment="bottom")
        ax4.get_xaxis().set_visible(False)

        ax5 = tau_fig.add_subplot(312)
        ax5.plot(x, self.tau[1])
        ax5.set_title("Knee", fontsize=15, verticalalignment='top')
        ax5.set_ylabel("Tau (Nm)",  fontsize=12, verticalalignment="bottom")
        ax5.get_xaxis().set_visible(False)

        ax6 = tau_fig.add_subplot(313)
        ax6.plot(x, self.tau[2])
        ax6.set_title("Ankle", fontsize=15, verticalalignment='top')
        ax6.set_ylabel("Tau (Nm)", fontsize=12, verticalalignment="bottom")
        ax6.set_xlabel("Walking Gait Cycle Percentage", fontsize=12, verticalalignment="top")

        plt.show()


if __name__ == '__main__':
    plotter = DynControlData()
    plotter.main()
