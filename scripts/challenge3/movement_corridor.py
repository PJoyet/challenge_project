#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""auteur: Pierre Joyet (3407684) - Marine CORNET (3531423) - Projet ROS- M1 SAR 2020"""

import rospy
import numpy as np
from math import atan2,pi
import sys, termios, tty
from geometry_msgs.msg import Twist
from challenge_project.msg import cardinal_direction

pubTopicName = "/cmd_vel"
subTopicName = "/camera/image_raw"


def set_speed(linear,angular):
    pub = rospy.Publisher(pubTopicName,Twist,queue_size=3)
    twist = Twist()
    twist.linear.x = linear
    twist.angular.z = angular
    pub.publish(twist)

def corridor(lds):
    l_s = rospy.get_param("/linear_scale")
    maxDist = max(lds.N,lds.E,lds.W,lds.NW,lds.NE)

    if lds.N <3.5:
        if maxDist==lds.N:
            set_speed(l_s,0)   

        if maxDist==lds.E:
            set_speed(l_s,-(np.pi/2))

        if maxDist==lds.W:
            set_speed(l_s,(np.pi/2))

        if maxDist==lds.NW:
            set_speed(l_s,(np.pi/2))

        if maxDist==lds.NE:
            set_speed(l_s,-(np.pi/2))

    else:
        set_speed(l_s,-(np.pi/4))
        rospy.sleep(1)
        set_speed(l_s,(np.pi/4))
        rospy.sleep(0.5)
        set_speed(0,0)
        rospy.set_param("task2_part1_running",False)
        rospy.signal_shutdown("task2 part1 end")         


def Task(data):
    if rospy.get_param("/task1_running") == False:
        corridor(data)
    else:
        pass
   


if __name__ == '__main__':

    rospy.init_node('movement_corridor')
    rospy.Subscriber("lds_distance", cardinal_direction, Task)
    rospy.spin()
