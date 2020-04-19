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
gains = [1,5,5]
camINFO = { 'height' : 0.135,
            'pitch'  : 0.610865238,
            'fov'    : 1.3962634
          }

cam_data = False
speed_data = (0,0)
lds_data = False
blue_flag = False
soon_end = False

def set_speed(linear,angular):
    pub = rospy.Publisher(pubTopicName,Twist,queue_size=3)
    twist = Twist()
    twist.linear.x = linear
    twist.angular.z = angular
    pub.publish(twist)

def line_detection(img):
    global cam_data
    global speed_data
    global blue_flag
    bridge = CvBridge()
    image = bridge.imgmsg_to_cv2(img, "bgr8")
    hsv=cv2.cvtColor(image,cv2.COLOR_RGB2HSV)
    image = cv2.GaussianBlur(hsv,(5,5),0)

    yellow_points = LineDetection(image,'yellow',colorformat="hsv")
    blue_points = LineDetection(image,'blue',colorformat="hsv")
    if len(blue_points) >=8:
        blue_flag = True
    else:
        blue_flag = False

    if len(yellow_points)>=5:
        cam_data = True
        speed_data = PointRallyingSpeed(yellow_points[0],yellow_points[4],image.shape,gains,camINFO)
    else:
        cam_data = False
        speed_data = (0,0)



def lds_detection(lds):
    global lds_data
    if lds.N <=0.3:
        lds_data = True
    else:
        lds_data = False

    Task()


def Task():
    global soon_end
    if lds_data == False and cam_data == True:
        set_speed(4*speed_data[0],2*speed_data[1])
        if blue_flag:
            pub = rospy.Publisher('Garage_Door_Opener',Bool,queue_size=3)
            pub.publish(1)
            soon_end = True

    elif cam_data == False and soon_end == True:
        set_speed(0,-0.5)
        rospy.sleep(1.5)
        set_speed(0,0)
        rospy.set_param("task1_running",False)
        rospy.signal_shutdown("task1 end")
    else:
        set_speed(0,0)



if __name__ == '__main__':

    rospy.init_node('line-follow_obst-avoidance')
    rospy.Subscriber(subTopicName,Image,line_detection)
    rospy.Subscriber("lds_distance", cardinal_direction, lds_detection)
    rospy.spin()