#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""auteur: Pierre Joyet (3407684) - Marine CORNET (3531423) - Projet ROS- M1 SAR 2020"""

import rospy
import numpy as np
import sys, termios, tty
from geometry_msgs.msg import Twist
from challenge_project.msg import FBLR_distance as Dist


def front_dist(data):
    pub = rospy.Publisher('/cmd_vel',Twist,queue_size=3)
    l_s = rospy.get_param("/linear_scale")
    twist = Twist()

    if data.Left < 1:
	if data.Back < data.Front :
            twist.linear.x =  l_s * np.cos (- np.pi/4)
	    twist.linear.y =  l_s * np.sin(- np.pi/4)
            twist.angular.z =  - np.pi/4
            pub.publish(twist)

	elif data.Back > data.Front :
	    twist.linear.x =  l_s * np.cos (np.pi/4)
	    twist.linear.y =  l_s * np.sin (np.pi/4)
            twist.angular.z =   np.pi/4
            pub.publish(twist)
   
    else :
	twist.linear.x = l_s 
	twist.linear.y = l_s 
        twist.angular.z = 0.0
        pub.publish(twist) 

    
   


if __name__ == '__main__':

    rospy.init_node('teleop_movement_corridor')

    rospy.Subscriber("front_lidar", Dist, front_dist)

    rospy.spin()
