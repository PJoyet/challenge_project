#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""auteur: Pierre Joyet (3407684) - Marine CORNET (3531423) - Projet ROS- M1 SAR 2020"""

import rospy, cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from math import atan2,pi
import sys, termios, tty
from geometry_msgs.msg import Twist
from challenge_project.msg import cardinal_direction

from challenge_project.CamColorDetection import LineDetection,PointRallyingSpeed

pubTopicName = "/cmd_vel"
subTopicName = "/camera/image_raw"
max_linear_speed = 0.22
max_angular_speed = 2.84
gains = [1,5,5]
camINFO = { 'height' : 0.135,
            'pitch'  : 0.610865238,
            'fov'    : 1.3962634
          }

nbYellow = 0
line_color = None

def set_speed(linear,angular):
    pub = rospy.Publisher(pubTopicName,Twist,queue_size=3)
    twist = Twist()
    twist.linear.x = linear
    twist.angular.z = angular
    pub.publish(twist)
     
def line_detection(img):
    global nbYellow,line_color
    bridge = CvBridge()
    image = bridge.imgmsg_to_cv2(img, "bgr8")
    hsv=cv2.cvtColor(image,cv2.COLOR_RGB2HSV)
    image = cv2.GaussianBlur(hsv,(5,5),0)

    yellow_points = LineDetection(image,'yellow',colorformat="hsv")
    red_points = LineDetection(image,'red',colorformat="hsv")
    max_points = max([yellow_points,red_points],key=len)

    if len(max_points)>=3:
        if max_points == yellow_points:
            linear,angular = PointRallyingSpeed(yellow_points[0],yellow_points[2],image.shape,gains,camINFO)
            set_speed(2*linear,2*angular)
            if line_color == 'red':
                nbYellow += 1
            line_color = 'yellow'
            print 'yellow'

        if max_points == red_points:
            linear,angular = PointRallyingSpeed(red_points[0],red_points[2],image.shape,gains,camINFO)
            set_speed(linear,angular)
            line_color = 'red'
            print 'red'

    else:
        pass

    print nbYellow
    if nbYellow==2:
        set_speed(0.2,0)
        rospy.sleep(1.5)
        set_speed(0,0)
        rospy.set_param("task2_part2_running",False)
        rospy.signal_shutdown("task2 end") 



def Task(data):
    if rospy.get_param("/task2_part1_running") == False:
        line_detection(data)
    else:
        pass
   


if __name__ == '__main__':

    rospy.init_node('teleop_movement_corridor_2')

    rospy.Subscriber(subTopicName,Image,Task)

    rospy.spin()
