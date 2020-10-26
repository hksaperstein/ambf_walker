#!/usr/bin/env python

import numpy as np
from datetime import datetime
from Model import Exoskeleton, Human
import rospy
from std_msgs.msg import Float32MultiArray

import csv

class DataCSVNode():

    def __init__(self, ):
        self.ROBOT_Q_SUB = rospy.Subscriber("exo_q", Float32MultiArray, self.robot_callback)
        self.BODY_Q_SUB = rospy.Subscriber("human_q", Float32MultiArray, self.body_callback)

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
        now = datetime.now()
        date_time = now.strftime("%m-%d-%Y_%H-%M-%S")
        # while True:p
        with open("data_" + date_time + ".csv", 'w') as file:
            writer = csv.writer(file, delimiter=',')
            writer.writerow(["Human Left Hip", "Robot Left Hip",
                             "Human Left knee", "Robot Left knee",
                             "Human Left Ankle", "Robot Left Ankle",
                             "Human Right Hip", "Robot Right Hip",
                             "Human Right Knee", "Robot Right Knee",
                             "Human Right Ankle", "Robot Right Ankle"])
            while True:
                writer.writerow([self.body_q[0], self.robot_q[0],
                                 self.body_q[1], self.robot_q[1],
                                 self.body_q[2], self.robot_q[2],
                                 self.body_q[3], self.robot_q[3],
                                 self.body_q[4], self.robot_q[4],
                                 self.body_q[5], self.robot_q[5]])
                # rospy.sleep(.25)

if __name__ == '__main__':
    csv_node = DataCSVNode()
    csv_node.main()
    rospy.spin()