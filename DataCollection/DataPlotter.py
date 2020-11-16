#!/usr/bin/env python

import numpy as np
import time
from Model import Exoskeleton, Human
import rospy
from std_msgs.msg import Float32MultiArray

import matplotlib.pyplot as plt
from matplotlib.lines import Line2D

class DataPlotterNode():

    def __init__(self, ):
        self.ROBOT_Q_SUB = rospy.Subscriber("robot_q", Float32MultiArray, self.robot_callback)
        self.BODY_Q_SUB = rospy.Subscriber("body_q", Float32MultiArray, self.body_callback)

        self._robot_q = None
        self._body_q = None

        rospy.init_node("DataPlotter", anonymous=True)

    @property
    def robot_q(self):
        return self._robot_q

    @robot_q.setter
    def robot_q(self, msg):
        self._robot_q = msg

    @property
    def body_q(self):
        return self._body_q

    @body_q.setter
    def body_q(self, msg):
        self._body_q = msg

    def robot_callback(self, msg):
        self._robot_q = msg.data

    def body_callback(self, msg):
        self._body_q = msg.data

    def main(self):

        x_data = []
        plt.ion()
        fig = plt.figure()
        left_hip_plot = fig.add_subplot(3, 2, 1)
        left_knee_plot = fig.add_subplot(3, 2, 3)
        left_ankle_plot = fig.add_subplot(3, 2, 5)
        right_hip_plot = fig.add_subplot(3, 2, 2)
        right_knee_plot = fig.add_subplot(3, 2, 4)
        right_ankle_plot = fig.add_subplot(3, 2, 6)
        plot_names = ["Left Hip", "Left knee", "Left Ankle", "Right Hip", "Right Knee", "Right Ankle"]
        plots = [left_hip_plot, left_knee_plot, left_ankle_plot, right_hip_plot, right_knee_plot, right_ankle_plot]
        num_joints = 6
        robot_lines = [Line2D([], [], color='red') for _ in range(num_joints)]
        body_lines = [Line2D([], [], color='blue') for _ in range(num_joints)]

        lines = robot_lines + body_lines
        robot_data = [[], [], [], [], [], []]
        body_data = [[], [], [], [], [], []]

        for i, plot in enumerate(plots):
            plot.add_line(robot_lines[i])
            plot.add_line(body_lines[i])
            plot.set_ylim(-2, 2)
            plot.set_xlim(0, 10000)
            # plot.set_autoscalex_on(True)
            plot.set_title(plot_names[i])


        while True:
            index = len(x_data)
            x_data.append(index)
            robot_q = self.robot_q
            body_q = self.body_q
            for i, data in enumerate(robot_data):
                data.append(robot_q[i])
            for i, data in enumerate(body_data):
                data.append(body_q[i])

            for i, line in enumerate(robot_lines):
                line.set_data(x_data, robot_data[i])
            for i, line in enumerate(body_lines):
                line.set_data(x_data, body_data[i])

            fig.canvas.draw()

            fig.canvas.flush_events()

if __name__ == '__main__':
    plotter = DataPlotterNode()
    plotter.main()
