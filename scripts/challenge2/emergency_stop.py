#! /usr/bin/env python
# -*- coding: utf-8 -*-

"""auteur: Pierre Joyet (3407684) - Marine CORNET (3531423) - Projet ROS- M1 SAR 2020"""

import rospy

from geometry_msgs.msg import Twist
from challenge_project.msg import cardinal_direction

pubTopicName = "/cmd_vel"
subTopicName = "/lds_distance"

def front_dist(lds):
    pub = rospy.Publisher(pubTopicName,Twist,queue_size=3)
    l_s = rospy.get_param("/linear_scale")
    twist = Twist()

    print lds.N

    if lds.N <=0.5:
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        pub.publish(twist)

    else :
	twist.linear.x = l_s
        twist.angular.z = 0.0
        pub.publish(twist)   


if __name__ == '__main__':

    rospy.init_node('teleop_lidar-emergency')
    rospy.Subscriber(subTopicName, cardinal_direction, front_dist)
    rospy.spin()