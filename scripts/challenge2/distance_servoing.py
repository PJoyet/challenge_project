#! /usr/bin/env python
# -*- coding: utf-8 -*-

"""author: Pierre Joyet (3407684) - Marine CORNET (3531423) - Project ROS- M1 SAR 2020"""

import rospy

from geometry_msgs.msg import Twist
from challenge_project.msg import cardinal_direction

# Parameter initialization
pubTopicName = "/cmd_vel" # Motor command topic
subTopicName = "/lds_distance" # Lidar message topic
max_linear_speed = 0.22 # Maximum linear speed of the robot
max_angular_speed = 2.84 # Maximum angular speed of the robot


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

def distance_servoing(lds):
    l_s = rospy.get_param("/linear_scale")
    d = rospy.get_param("/constant_distance")

    if lds.N <=d+0.3: # if the front distance less than 0.6
        delta = (lds.N-d)*0.6 # get distance from robot to desired position
        if lds.N <= d: # if the front distance less than desired distance
        	set_speed(-l_s+delta,0.0) # set robot move backward
        else :
        	set_speed(l_s+delta,0.0) # set robot move forward
    else: 
    	set_speed(0,0)


if __name__ == '__main__':

    rospy.init_node('lds_distance-servoing')
    rospy.Subscriber(subTopicName, cardinal_direction, distance_servoing)
    rospy.spin()