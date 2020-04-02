#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy, cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist

from challenge_project.CamColorDetection import LineDetection,PointRallyingSpeed

pubTopicName = rospy.get_param("/motorsCommandTopic")
subTopicName = rospy.get_param("/imageTopicName")

def set_speed(linear,angular):
    pub = rospy.Publisher(pubTopicName,Twist,queue_size=3)
    twist = Twist()
    twist.linear.x = linear
    twist.angular.z = angular
    pub.publish(twist)

def line_detection(img):

    bridge = CvBridge()
    image = bridge.imgmsg_to_cv2(img, "bgr8")
    hsv=cv2.cvtColor(image,cv2.COLOR_RGB2HSV)
    image = cv2.GaussianBlur(hsv,(5,5),0)

    yellow_points = LineDetection(image,'yellow',colorformat="hsv")
    red_points = LineDetection(image,'red',colorformat="hsv")
    green_points = LineDetection(image,'green',colorformat="hsv")
    max_points = max([yellow_points,red_points,green_points],key=len)

    if len(max_points)>=5:
        if max_points == yellow_points:
            linear,angular = PointRallyingSpeed(yellow_points[0],yellow_points[4],2,image.shape)
            set_speed(linear,angular)
        if max_points == red_points:
            linear,angular = PointRallyingSpeed(red_points[0],red_points[4],0.8,image.shape)
            set_speed(linear,angular)
        if max_points == green_points:
            set_speed(0,0)
    else:
        set_speed(0,0)


if __name__ == '__main__':
    rospy.init_node('line_following')
    sub = rospy.Subscriber(subTopicName,Image,line_detection)
    rospy.spin()
