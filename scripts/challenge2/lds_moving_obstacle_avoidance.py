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
    d = rospy.get_param("/distance_constante")
    twist = Twist()

    #mur à l'Ouest
    if data.Left < d + 0.21:
        twist.linear.x = - l_s 
	twist.linear.y = - l_s 
        twist.angular.z = 0.0
        pub.publish(twist)

    #mur au Sud-Ouest
    elif (data.Left < d + 0.21) and (data.Back < d + 0.21) :
	twist.linear.x = - l_s * np.cos (np.pi/8)
	twist.linear.y = - l_s * np.sin (np.pi/8)
	twist.angular.z = np.pi/8
        pub.publish(twist)

    #mur au Sud
    elif data.Back < d + 0.21:
        twist.linear.x = - l_s * np.cos (np.pi/4)
	twist.linear.y = - l_s * np.sin(np.pi/4)
        twist.angular.z = np.pi/4
        pub.publish(twist)

    #mur au Sud 
    elif (data.Back > d + 0.21) and (data.Back < 10):
        twist.linear.x = l_s * np.cos (np.pi/4)
	twist.linear.y = l_s * np.sin(np.pi/4)
	twist.angular.z = np.pi/4
        pub.publish(twist)

    #mur au Nord
    elif data.Front < d + 0.21:
        twist.linear.x = - l_s * np.cos (-np.pi/4)
	twist.linear.y = - l_s * np.sin(-np.pi/4)
        twist.angular.z = - np.pi/4
        pub.publish(twist)

    #mur au Nord-Ouest
    elif (data.Left < d + 0.21) and (data.Front < d + 0.21) :
	twist.linear.x = - l_s * np.cos(- np.pi/8)
	twist.linear.y = - l_s * np.sin (- np.pi/8)
        twist.angular.z = - np.pi/8
        pub.publish(twist)
    
    #mur au Nord-Est
    elif (data.Right < d + 0.21) and (data.Front < d + 0.21) :
	twist.linear.x = - l_s * np.cos (-3*np.pi/8)
	twist.linear.y = - l_s * np.sin (-3*np.pi/8)
        twist.angular.z = -3*np.pi/8
        pub.publish(twist)

    #mur au nord
    elif (data.Front > d + 0.21) and (data.Front < 10):
        twist.linear.x = l_s * np.cos(- np.pi/4)
	twist.linear.y = l_s * np.sin (- np.pi/4)
	twist.angular.z = - np.pi/4
	pub.publish(twist)
  
    #mur à l'Est
    elif data.Right < d + 0.21:
        twist.linear.x = - l_s * np.cos (np.pi/2)
	twist.linear.y = - l_s * np.sin (np.pi/2)
        twist.angular.z = np.pi/2
        pub.publish(twist)

    #mur à l'Est
    elif (data.Right > d + 0.21) and (data.Right < 10):
        twist.linear.x = l_s * np.cos (np.pi/2)
	twist.linear.y = l_s * np.sin (np.pi/2)
	twist.angular.z = np.pi/2
	pub.publish(twist)

    #mur au Sud-Est
    elif (data.Right < d + 0.21) and (data.Back < d + 0.21) :
	twist.linear.x = - l_s * np.cos (3*np.pi/8)
	twist.linear.y = - l_s * np.sin (3*np.pi/8)
        twist.angular.z = 3*np.pi/8
        pub.publish(twist)
   
    #mur à l'Ouest
    else :
	twist.linear.x = l_s 
	twist.linear.y = l_s 
        twist.angular.z = 0.0
        pub.publish(twist) 

    
   


if __name__ == '__main__':

    rospy.init_node('teleop_lidar-emergency')

    rospy.Subscriber("front_lidar", Dist, front_dist)

    rospy.spin()
