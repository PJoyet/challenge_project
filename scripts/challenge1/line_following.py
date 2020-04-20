#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""author: Pierre Joyet (3407684) - Marine CORNET (3531423) - ROS Project - M1 SAR 2020"""

import rospy, cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist

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

# Get speed gain
try:
    SpeedGain = rospy.get_param("/SpeedGain")
except rospy.ROSInterruptException:
    SpeedGain = 1


def set_speed(linear,angular):
    # Publish the speed data
    pub = rospy.Publisher(pubTopicName,Twist,queue_size=3)
    twist = Twist()
    linear *= SpeedGain
    angular *= SpeedGain

    if linear >= max_linear_speed:
        linear = max_linear_speed
    if angular >= max_angular_speed:
        linear = max_angular_speed

    twist.linear.x = linear
    twist.angular.z = angular
    pub.publish(twist)

def line_detection(img):

    bridge = CvBridge()
    # Conversion from image to open cv image
    image = bridge.imgmsg_to_cv2(img, "bgr8")
    #Conversion to hsv mode (hue saturation  value )
    hsv=cv2.cvtColor(image,cv2.COLOR_RGB2HSV)
    #Blue of the image
    image = cv2.GaussianBlur(hsv,(5,5),0)

    # Find yellow, red and green points on the image
    yellow_points = LineDetection(image,'yellow',colorformat="hsv")
    red_points = LineDetection(image,'red',colorformat="hsv")
    green_points = LineDetection(image,'green',colorformat="hsv")
    # Get the the maximum numbers from yellow_points, red_points and green_points lists
    max_points = max([yellow_points,red_points,green_points],key=len)

    if len(max_points)>=5: # If max_point more than 5

        if max_points == yellow_points: # If yellow_point have maximum point get the speed of the robot
            linear,angular = PointRallyingSpeed(yellow_points[0],yellow_points[4],image.shape,gains,camINFO)
            set_speed(linear,angular)

        if max_points == red_points: # If red_point have maximum point get the speed of the robot
            linear,angular = PointRallyingSpeed(red_points[0],red_points[4],image.shape,gains,camINFO)
            linear *= 0.5 # Reducing by half the speed for red line
            set_speed(linear,angular)

        if max_points == green_points: # If green_point have maximum point get the speed of the robot
            set_speed(0,0) # Stop the robot
            rospy.signal_shutdown("chg1 task1 end") # Stopping the node

    else:
        set_speed(0,0)


if __name__ == '__main__':
    rospy.init_node('line_following')
    sub = rospy.Subscriber(subTopicName,Image,line_detection)
    rospy.spin()
