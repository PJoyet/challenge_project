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
cam_data = False
speed_data = (0,0)
lds_data = False
blue_flag = False
soon_end = False

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
    global cam_data
    global speed_data
    global blue_flag

    bridge = CvBridge()
    # Conversion from image to open cv image
    image = bridge.imgmsg_to_cv2(img, "bgr8")
    #Conversion to hsv mode (hue saturation  value )
    hsv=cv2.cvtColor(image,cv2.COLOR_RGB2HSV)
    #Blur of the image
    image = cv2.GaussianBlur(hsv,(5,5),0)

    # Find yellow and blue points on the image
    yellow_points = LineDetection(image,'yellow',colorformat="hsv")
    blue_points = LineDetection(image,'blue',colorformat="hsv")

    if len(blue_points) >=8: # If blue_point more than 8 set blue flag to True
        blue_flag = True
    else:
        blue_flag = False

    if len(yellow_points)>=5: # If yellow_point more than 5 set cam_data flag to True
        cam_data = True       # and set the speed
        speed_data = PointRallyingSpeed(yellow_points[0],yellow_points[4],image.shape,gains,camINFO)
    else:
        cam_data = False
        speed_data = (0,0)



def lds_detection(lds):
    global lds_data
    if lds.N <=0.3: # if the front distance less than 0.3
        lds_data = True # set lds_data flag to True
    else:
        lds_data = False

    Task()


def Task():
    global soon_end
    if lds_data == False and cam_data == True: # if lidar data more than 0.3 and yellow line found
        set_speed(4*speed_data[0],2*speed_data[1]) # set speed
        if blue_flag: # if blue found in camera: open the blue gate
            pub = rospy.Publisher('Garage_Door_Opener',Bool,queue_size=3)
            pub.publish(1)
            soon_end = True # set soon_end flag to True

    elif cam_data == False and soon_end == True: # if nothing found in cam and soon_end True
        set_speed(0.22,-0.5) # place the robot in front of corridor
        rospy.sleep(1.5)
        set_speed(0,0)
        rospy.set_param("task1_running",False) # set task1_running parameter to False
        rospy.signal_shutdown("task1 end") # stop the node
    else:
        set_speed(0,0)



if __name__ == '__main__':

    rospy.init_node('line-follow_obst-avoidance')
    rospy.Subscriber(subTopicName,Image,line_detection)
    rospy.Subscriber("lds_distance", cardinal_direction, lds_detection)
    rospy.spin()