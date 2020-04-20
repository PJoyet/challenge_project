#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy, cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
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
orange_flag = False
nb = 0


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
    global orange_flag
    global nb

    bridge = CvBridge()
    # Conversion from image to open cv image
    image = bridge.imgmsg_to_cv2(img, "bgr8")
    #Blur of the image
    image = cv2.GaussianBlur(image,(5,5),0)

    # Find yellow, orange and green points on the image
    yellow_points = LineDetection(image,'yellow')
    orange_points = LineDetection(image,'orange')
    green_points = LineDetection(image,'green')
    # Get the the maximum numbers from yellow_points, orange_points and green_point
    max_points = max([yellow_points,orange_points,green_points],key=len)

    if len(max_points)>=5: # If maximum more than 4

        if max_points == yellow_points: # If yellow_point has maximum point
            if orange_flag == False:# If orange_flag is false set the speed for point 0 and 3
                linear,angular = PointRallyingSpeed(yellow_points[0],yellow_points[4],image.shape,gains,camINFO)
            else:
                if len(max_points)>=10: # else set the speed for point 0 and 9
                    linear,angular = PointRallyingSpeed(yellow_points[0],yellow_points[9],image.shape,gains,camINFO)
                    nb+=1 # add 1 to flag nb
                else: # or 0 and 3
                    linear,angular = PointRallyingSpeed(yellow_points[0],yellow_points[3],image.shape,gains,camINFO)
 
            set_speed(2*linear,2*angular)

        if max_points == orange_points: # If yellow_point has maximum point
            set_speed(-0.1,2) # return the robot
            orange_flag = True # set orange_flag to True

        if max_points == green_points:# If yellow_point has maximum point
            set_speed(0,0) # stop the robot

    else:
        pass

    if nb==800: # if nb is 800 set orange_flag to False
        orange_flag=False
        nb = 0 # set the robot to 0


def Task(data):
    if rospy.get_param("task2_part2_running") == False:# if task2 part2 is running don't start task3
        line_detection(data)
    else:
        pass



if __name__ == '__main__':

    rospy.init_node('line-follow_obst-avoidance')
    rospy.Subscriber(subTopicName,Image,Task)
    rospy.spin()