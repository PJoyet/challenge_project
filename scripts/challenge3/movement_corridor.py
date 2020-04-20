#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""auteur: Pierre Joyet (3407684) - Marine CORNET (3531423) - Projet ROS- M1 SAR 2020"""

import rospy
import numpy as np
from math import atan2,pi
import sys, termios, tty
from geometry_msgs.msg import Twist
from challenge_project.msg import cardinal_direction

# Parameter initialization
pubTopicName = "/cmd_vel" # Motor command topic
subTopicName = "/lds_distance" # Lidar message topic
max_linear_speed = 0.22 # Maximum linear speed of the robot
max_angular_speed = 2.84 # Maximum angular speed of the robot

def set_speed(linear,angular):
    # Publish the speed data
    pub = rospy.Publisher(pubTopicName,Twist,queue_size=3)
    twist = Twist()

    if linear >= max_linear_speed:
        linear = max_linear_speed
    if angular >= max_angular_speed:
        linear = max_angular_speed

    twist.linear.x = linear
    twist.angular.z = angular
    pub.publish(twist)

def corridor(lds):
    l_s = rospy.get_param("/linear_scale")

    # get the maximum distance
    maxDist = max(lds.N,lds.E,lds.W,lds.NW,lds.NE)

    if lds.N <3.5: # if Front distance less than 3.5
        if maxDist==lds.N:# if maximum distance is Front
            set_speed(l_s,0)   

        if maxDist==lds.E:# if maximum distance is Right
            set_speed(l_s,-(np.pi/2))

        if maxDist==lds.W:# if maximum distance is Left
            set_speed(l_s,(np.pi/2))

        if maxDist==lds.NW:# if maximum distance is Front-Left
            set_speed(l_s,(np.pi/2))

        if maxDist==lds.NE:# if maximum distance is Front-East
            set_speed(l_s,-(np.pi/2))

    else: # if Front distance more than 3.5
        set_speed(l_s,-(np.pi/4)) # place the robot in front of line
        rospy.sleep(1)
        set_speed(l_s,(np.pi/4))
        rospy.sleep(0.5)
        set_speed(l_s,0)
        rospy.sleep(3)
        set_speed(0,(np.pi/4))
        rospy.sleep(0.5)
        set_speed(0,0)
        rospy.set_param("task2_part1_running",False)# set task2_part1_running parameter to False
        rospy.signal_shutdown("task2 part1 end") # stop the node    


def Task(data):
    if rospy.get_param("/task1_running") == False: # if task1 is running don't start task2
        corridor(data)
    else:
        pass
   


if __name__ == '__main__':

    rospy.init_node('movement_corridor')
    rospy.Subscriber(subTopicName, cardinal_direction, Task)
    rospy.spin()
