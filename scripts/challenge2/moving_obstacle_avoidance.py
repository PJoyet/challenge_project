#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""author: Pierre Joyet (3407684) - Marine CORNET (3531423) - Project ROS- M1 SAR 2020"""

import rospy
import numpy as np
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

def distance_servoing(lds):
    l_s = rospy.get_param("/linear_scale")
    a_s = rospy.get_param("/angular_scale")
    d = rospy.get_param("/constant_distance")

    # get the minimum distance
    minDist = min(lds.N,lds.S,lds.E,lds.W,lds.NW,lds.NE,lds.SE,lds.SW)

    if minDist<1.2: # if minimum distance less than 1.2

        if minDist==lds.N: # if minimum distance is Front
            delta = (lds.N-d)*0.8

            if lds.N <= d:
                set_speed(-l_s+delta,0.0)
            else :
                set_speed(l_s+delta,0.0)

        elif minDist==lds.S: # if minimum distance is Back
            set_speed(0.0,np.pi*a_s)

        elif minDist==lds.E:  # if minimum distance is Right
            set_speed(0.0,-(np.pi/2)*a_s)

        elif minDist==lds.W:  # if minimum distance is Left
            set_speed(0.0,(np.pi/2)*a_s)

        elif minDist==lds.NW: # if minimum distance is Front-Left
            set_speed(0.0,(np.pi/4)*a_s)

        elif minDist==lds.NE: # if minimum distance is Front-Right
            set_speed(0.0,-(np.pi/4)*a_s)

        elif minDist==lds.SW: # if minimum distance is Back-Left
            set_speed(0.0,(2*np.pi/3)*a_s)

        elif minDist==lds.SE: # if minimum distance is Back-Right
            set_speed(0.0,-(2*np.pi/3)*a_s)

    else:
        set_speed(0,0)        

if __name__ == '__main__':

    rospy.init_node('lds_moving-obstacle-avoidance')
    rospy.Subscriber(subTopicName, cardinal_direction, distance_servoing)
    rospy.spin()