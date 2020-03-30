#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys, rospy, cv2
import numpy as np
from math import tan,pi,sqrt,atan2,atan
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist

pubTopicName = rospy.get_param("/motorsCommandTopic")
subTopicName = rospy.get_param("/imageTopicName")

cam_pos = rospy.get_param("/cam_position")
cam_ori = rospy.get_param("/cam_orientation")
height,width = rospy.get_param("/cam_image_size")
cam_rate = rospy.get_param("/cam_rate")
cam_fov = rospy.get_param("/cam_fov")

yellow = np.uint8(rospy.get_param("/yellow_range"))
red = np.uint8(rospy.get_param("/red_range"))
green = np.uint8(rospy.get_param("/green_range"))

klinear = rospy.get_param("/klinear")
kangular1 = rospy.get_param("/kangular1")
kangular2 = rospy.get_param("/kangular2")

def get_angle(pt1,pt2):
    X = pt2[0]-pt1[0]
    Y = pt2[1]-pt1[1]
    return (atan2(Y,X)-pi/2)

def get_pos(pt):
    global height,width,cam_pos,cam_ori,cam_fov

    delta_X = -(pt[0]-width//2)
    delta_Y = height-pt[1]
    
    dist_bas =  cam_pos[2]*tan((pi-cam_fov)/2-cam_ori[1])
    dist_haut = cam_pos[2]*tan(pi/2-cam_ori[1])
    
    Y = delta_Y*(dist_haut-dist_bas)/(height/2)+dist_bas+cam_pos[0]

    dist_x = Y*tan(cam_fov/2)
    X = delta_X*(dist_x/width/2)
    
    return X,Y

def get_center(img,color):
    cx = None
    cy = None
    mask = cv2.inRange(img,color[0],color[1])
    M = cv2.moments(mask)
    if M['m00'] > 0:
        cx = int (M[ 'm10' ] /M[ 'm00' ] )
        cy = int (M[ 'm01' ] /M[ 'm00' ] )
    return cx,cy

def get_points(img,color,nbPoints=20):
    points = [(0,0)]
    height = img.shape[0]
    width  = img.shape[1]
    
    for i in range(height//nbPoints,height,height//nbPoints):
        row = img[i-height//nbPoints:i]
        cx,cy = get_center(row,color)
        if cx!=None and cy !=None:
            points.append((cx,cy+i-height//nbPoints))
        else:
            pass
    #points = list(filter(None, points))
    #print(len(points))
    return points[::-1]#.reverse()
    
def get_speed(pub,pt1,pt2,angular2,globalspeed):
    linear = sqrt(pt1[0]**2 + pt1[1]**2)
    angular1 = get_angle((0,0),pt1)
    
    global klinear,kangular1,kangular2
    
    linear = (linear*klinear)*globalspeed
    angular = (- (kangular1*angular1) - (kangular2*angular2))*globalspeed
    return linear,angular
    
def line_detection(img):
    global yellow,red,green
    global height,width
    global pubTopicName
    twist = Twist()
    pub = rospy.Publisher(pubTopicName,Twist,queue_size=3)    
    
    bridge = CvBridge()
    image = bridge.imgmsg_to_cv2(img, "bgr8")
    blur = cv2.GaussianBlur(image,(5,5),0)
    
    yellow_points = get_points(blur,yellow)
    red_points = get_points(blur,red)
    green_points = get_points(blur,green)
    #print(len(yellow_points),len(red_points),len(green_points))
    max_points = max([yellow_points,red_points,green_points],key=len)
    if len(max_points)>=5:
        if max_points == yellow_points:
            pt1 = get_pos(yellow_points[0])
            pt2 = get_pos(yellow_points[4])
            angle = get_angle(pt1,pt2)
            linear,angular = get_speed(pub,pt1,pt2,angle,2)
            twist.linear.x = linear
            twist.angular.z = angular
            pub.publish(twist)
            cv2.arrowedLine(image, yellow_points[0], yellow_points[4],(0, 100, 100), 5, cv2.LINE_AA, 0, 0.08)
            
            text_linear =  "linear speed  : %f"%(linear)
            text_angular = "angular speed : %f"%(angular)
            font = cv2.FONT_HERSHEY_SIMPLEX
            textsize_a = cv2.getTextSize(text_angular, font, 0.8, 2)[0]
            textX_l = 5
            textY_l = height-textsize_a[1]-25
            textX_a = 5
            textY_a = height-20
            cv2.putText(image, text_linear, (textX_l,textY_l), font , 0.8, (0, 100, 100), 2, cv2.LINE_AA)
            cv2.putText(image, text_angular, (textX_a,textY_a), font , 0.8, (0, 100, 100), 2, cv2.LINE_AA)
            
        if max_points == red_points:
            pt1 = get_pos(red_points[0])
            pt2 = get_pos(red_points[4])
            angle = get_angle(pt1,pt2)
            linear,angular = get_speed(pub,pt1,pt2,angle,1)
            twist.linear.x = linear
            twist.angular.z = angular
            pub.publish(twist)
            cv2.arrowedLine(image, red_points[0], red_points[4],(0, 0, 100), 5, cv2.LINE_AA, 0, 0.08)
            
            text_linear =  "linear speed  : %f"%(linear)
            text_angular = "angular speed : %f"%(angular)
            font = cv2.FONT_HERSHEY_SIMPLEX
            textsize_a = cv2.getTextSize(text_angular, font, 0.8, 2)[0]
            textX_l = 5
            textY_l = height-textsize_a[1]-25
            textX_a = 5
            textY_a = height-20
            cv2.putText(image, text_linear, (textX_l,textY_l), font , 0.8, (0, 0, 100), 2, cv2.LINE_AA)
            cv2.putText(image, text_angular, (textX_a,textY_a), font , 0.8, (0, 0, 100), 2, cv2.LINE_AA)
            
        if max_points == green_points:
            twist.linear.x = 0
            twist.angular.z = 0
            pub.publish(twist)
            
            text = "Task completed"
            font = cv2.FONT_HERSHEY_SIMPLEX
            textsize = cv2.getTextSize(text, font, 2, 5)[0]
            textX = (width - textsize[0]) // 2
            textY = green_points[len(green_points)//2][1]
            image = cv2.putText(image, text, (textX,textY), font , 2, (0, 100, 0), 5, cv2.LINE_AA) 


    
    else:
        twist.linear.x = 0
        twist.angular.z = -0.01
        pub.publish(twist)
        
    global subTopicName
    pub = rospy.Publisher("image_mask",Image, queue_size=3)
    pub.publish(bridge.cv2_to_imgmsg(image,"bgr8")) 
        

if __name__ == '__main__':
    rospy.init_node('line_following')
    topicName = subTopicName
    sub = rospy.Subscriber(topicName,Image,line_detection)
    rospy.spin()




