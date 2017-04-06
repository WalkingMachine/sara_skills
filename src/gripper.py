#! /usr/bin/env python
# -*-coding:Utf-8 -*

# launch dependencies : sara_launch TcpNode.launch

import roslib; roslib.load_manifest('robotiq_c_model_control')
import rospy
from robotiq_c_model_control.msg import _CModel_robot_output  as outputMsg
from time import sleep

class Gripper:
    """
    Gripper Interface
        Go to : 0-255 (0 => open)
        Speed : 0-255
        Force : 0-255
    """

    def __init__(self):
        self.gripperPub = rospy.Publisher('CModelRobotOutput', outputMsg.CModel_robot_output, queue_size=1)
        self.command = outputMsg.CModel_robot_output();

    def reset(self):
        self.command = outputMsg.CModel_robot_output();
        self.command.rACT = 0
        self.gripperPub.publish(self.command)
        rospy.sleep(0.1)

    def activate(self):
        self.command = outputMsg.CModel_robot_output();
        self.command.rACT = 1
        self.command.rGTO = 1
        self.command.rSP  = 255
        self.command.rFR  = 150
        self.gripperPub.publish(self.command)
        rospy.sleep(0.1)

    def close(self):
        self.command.rPR = 255
        self.gripperPub.publish(self.command)
        rospy.sleep(0.1)

    def open(self):
        self.command.rPR = 0
        self.gripperPub.publish(self.command)
        rospy.sleep(0.1)

    def goTo(self, position):
        try:
            self.command.rPR = int(position)
            if self.command.rPR > 255:
                self.command.rPR = 255
            if self.command.rPR < 0:
                self.command.rPR = 0

            self.gripperPub.publish(self.command)
            rospy.sleep(0.1)
        except ValueError:
            pass

    def setSpeed(self, speed):
        try:
            self.command.rSP = int(speed)
            if self.command.rSP > 255:
                self.command.rSP = 255
            if self.command.rSP < 0:
                self.command.rSP = 0

            self.gripperPub.publish(self.command)
            rospy.sleep(0.1)
        except ValueError:
            pass

    def setForce(self, force):
        try:
            self.command.rFR = int(force)
            if self.command.rFR > 255:
                self.command.rFR = 255
            if self.command.rFR < 0:
                self.command.rFR = 0
            self.gripperPub.publish(self.command)
            rospy.sleep(0.1)
        except ValueError:
            pass
