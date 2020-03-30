#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys, rospy, cv2
import numpy as np
from math import tan,pi,sqrt,atan2,atan
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist

yellow_upper  = np.array([0,255,255])
yellow_lower  = np.array([0,200,200])

red_upper  = np.array([0,0,255])
red_lower  = np.array([0,0,200])


def get_points(img,lower,upper,dim=10):
    height,weight = rospy.get_param("/cam_image_size")
    points = []
    for i in range(dim,height-dim,dim):
        row = img[height-i-dim:height-i+dim]
        mask = cv2.inRange(row,lower,upper)
        M = cv2.moments(mask)
        if M['m00'] > 0:
            cx = int (M[ 'm10' ] /M[ 'm00' ] )
            cy = int (M[ 'm01' ] /M[ 'm00' ] )+height-i-dim
            points.append((cx,cy))
    return points

def get_angle(pt1,pt2):
    X = pt2[0]-pt1[0]
    Y = pt2[1]-pt1[1]
    return (atan2(Y,X)-pi/2)

def get_pos(pt):
    height,width = rospy.get_param("/cam_image_size")
    cam_position = rospy.get_param("/cam_position")
    cam_angle = rospy.get_param("/cam_orientation")
    cam_fov = rospy.get_param("/cam_fov")
    
    
    delta_X = -(pt[0]-width//2)
    delta_Y = height-pt[1]
    
    dist_bas =  cam_position[2]*tan((pi-cam_fov)/2-cam_angle[1])
    dist_haut = cam_position[2]*tan(pi/2-cam_angle[1])
    
    Y = delta_Y*(dist_haut-dist_bas)/(height/2)+dist_bas+cam_position[0]

    dist_x = Y*tan(cam_fov/2)
    X = delta_X*(dist_x/width/2)
    
    return X,Y


def detection(img):
    global yellow_lower
    global yellow_upper
    global red_lower
    global red_upper
    bridge = CvBridge()
    image = bridge.imgmsg_to_cv2(img, "bgr8")
   # img_not = cv2.bitwise_not(image)
   # hsv_image = cv2.cvtColor(image,cv2.COLOR_RGB2HSV)
    blur = cv2.GaussianBlur(image,(5,5),0)
    yellow_points = get_points(blur,yellow_lower,yellow_upper,dim=10)
    red_points = get_points(blur,red_lower,red_upper,dim=10)
    print len(yellow_points), len(red_points)
    if yellow_points>red_points:
       # print 'yellow'
        if len(yellow_points)>11:
            pt1 = get_pos(yellow_points[5])
            pt2 = get_pos(yellow_points[10])
            angle = get_angle(pt1,pt2)
            rho = sqrt(pt1[0]**2 + pt1[1]**2)
            beta = get_angle((0,0),pt1)
            alpha = beta
            krho = 1
            kalpha = -2
            kangle = -2
            twist = Twist()
            twist.linear.x = (rho*krho)*2
            twist.angular.z = (alpha*kalpha+kangle*angle)*2
            pub = rospy.Publisher('/cmd_vel',Twist,queue_size=3)
            pub.publish(twist)
    else:
       # print 'red'
        if len(red_points)>11:
            pt1 = get_pos(red_points[5])
            pt2 = get_pos(red_points[10])
            angle = get_angle(pt1,pt2)
            rho = sqrt(pt1[0]**2 + pt1[1]**2)
            beta = get_angle((0,0),pt1)
            alpha = beta
            krho = 1
            kalpha = -2
            kangle = -2
            twist = Twist()
            twist.linear.x = (rho*krho)*0.5
            twist.angular.z = (alpha*kalpha+kangle*angle)*0.5
            pub = rospy.Publisher('/cmd_vel',Twist,queue_size=3)
            pub.publish(twist)

    pub = rospy.Publisher("image_mask",Image, queue_size=3)
    pub.publish(bridge.cv2_to_imgmsg(image,"bgr8"))  

if __name__ == '__main__':
    rospy.init_node('line_detection')
    topicName = rospy.get_param("imageTopicName")
    sub = rospy.Subscriber(topicName,Image,detection)
    rospy.spin()

