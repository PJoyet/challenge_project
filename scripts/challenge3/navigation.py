#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy, cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from challenge_project.msg import cardinal_direction
from challenge_project.CamColorDetection import LineDetection,PointRallyingSpeed

pubTopicName = "/cmd_vel"
subTopicName = "/camera/image_raw"
max_linear_speed = 0.22
max_angular_speed = 2.84
gains = [1,10,10]
camINFO = { 'height' : 0.135,
            'pitch'  : 0.610865238,
            'fov'    : 1.3962634
          }
orange_flag = False
nb = 0


def set_speed(linear,angular):
    pub = rospy.Publisher(pubTopicName,Twist,queue_size=3)
    twist = Twist()
    twist.linear.x = linear
    twist.angular.z = angular
    pub.publish(twist)

def line_detection(img):
    global orange_flag
    global nb
    bridge = CvBridge()
    image = bridge.imgmsg_to_cv2(img, "bgr8")
    #image=cv2.cvtColor(image,cv2.COLOR_RGB2HSV)
    image = cv2.GaussianBlur(image,(5,5),0)

    yellow_points = LineDetection(image,'yellow')#,colorformat="hsv")
    orange_points = LineDetection(image,'orange')#,colorformat="hsv")
    green_points = LineDetection(image,'green')#,colorformat="hsv")

    max_points = max([yellow_points,orange_points,green_points],key=len)


    if len(max_points)>=4:

        if max_points == yellow_points:
            if orange_flag == False:
                linear,angular = PointRallyingSpeed(yellow_points[0],yellow_points[3],image.shape,gains,camINFO)
            else:
                if len(max_points)>=10:
                    linear,angular = PointRallyingSpeed(yellow_points[0],yellow_points[9],image.shape,gains,camINFO)
                    nb+=1
                else:
                    linear,angular = PointRallyingSpeed(yellow_points[0],yellow_points[3],image.shape,gains,camINFO)
 
            set_speed(2*linear,2*angular)
            print 'yellow'

        if max_points == orange_points:
            set_speed(-0.1,2)
            orange_flag = True
            print 'orange'
            orange_flag = True

        if max_points == green_points:
            set_speed(0,0)
            print 'green'

    else:
        pass
       # set_speed(0,1)

    print orange_flag,nb
    if nb==800:
        orange_flag=False
        nb = 0


def Task(data):
    if rospy.get_param("task2_part2_running") == False:
        line_detection(data)
    else:
        pass



if __name__ == '__main__':

    rospy.init_node('line-follow_obst-avoidance')
    rospy.Subscriber(subTopicName,Image,Task)
    rospy.spin()