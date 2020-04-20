#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""author: Pierre Joyet (3407684) - Marine CORNET (3531423) - Project ROS- M1 SAR 2020"""

import rospy, cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from math import atan2,pi
import sys, termios, tty
from geometry_msgs.msg import Twist
from challenge_project.msg import cardinal_direction

from challenge_project.CamColorDetection import LineDetection,PointRallyingSpeed

# Parameter initialization
pubTopicName = "/cmd_vel" # Motor command topic
subTopicName = "/camera/image_raw" # Camera image topic
max_linear_speed = 0.22 # Maximum linear speed of the robot
max_angular_speed = 2.84 # Maximum angular speed of the robot
gains = [1,5,5] # Gains control speed
camINFO = { 'height' : 0.135, # Height of the camera
            'pitch'  : 0.610865238, # Pich rotation of the camera
            'fov'    : 1.3962634 # FOV of the camera
          }

# Global data initialisation
nbYellow = 0
line_color = None

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
     
def line_detection(img):
    global nbYellow,line_color
    bridge = CvBridge()
    # Conversion from image to open cv image
    image = bridge.imgmsg_to_cv2(img, "bgr8")
    #Conversion to hsv mode (hue saturation  value )
    hsv=cv2.cvtColor(image,cv2.COLOR_RGB2HSV)
    #Blur of the image
    image = cv2.GaussianBlur(hsv,(5,5),0)

    # Find yellow and red points on the image
    yellow_points = LineDetection(image,'yellow',colorformat="hsv")
    red_points = LineDetection(image,'red',colorformat="hsv")
    # Get the the maximum numbers from yellow_points, red_points
    max_points = max([yellow_points,red_points],key=len)

    if len(max_points)>=3: # If max_point more than 3
        if max_points == yellow_points:# If yellow_point have maximum point get the speed of the robot
            linear,angular = PointRallyingSpeed(yellow_points[0],yellow_points[2],image.shape,gains,camINFO)
            set_speed(2*linear,2*angular)
            if line_color == 'red': # if previous line_color is red
                nbYellow += 1 # add 1 to nbYellow
            line_color = 'yellow' # change line_color to yellow 

        if max_points == red_points: # If red_point have maximum point get the speed of the robot
            linear,angular = PointRallyingSpeed(red_points[0],red_points[2],image.shape,gains,camINFO)
            set_speed(linear,angular)
            line_color = 'red' # change line_color to red

    else:
        pass

    if nbYellow==2: # if nbYellow is 2 place the robot a little forward
        set_speed(0.2,0)
        rospy.sleep(1)
        set_speed(0,0)
        rospy.set_param("task2_part2_running",False)
        rospy.signal_shutdown("task2 end") 



def Task(data):
    if rospy.get_param("/task2_part1_running") == False:# if task2 part1 is running don't start task2 part 2
        line_detection(data)
    else:
        pass
   


if __name__ == '__main__':

    rospy.init_node('teleop_movement_corridor_2')
    rospy.Subscriber(subTopicName,Image,Task)
    rospy.spin()
