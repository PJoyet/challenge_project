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
    d = rospy.get_param("/distance_constante")
    twist = Twist()


    if lds.N <=0.6:
        delta = (lds.N-d)*0.6

        if lds.N <= d:
            twist.linear.x = -l_s+delta
            twist.angular.z = 0.0
            pub.publish(twist)

        else :
            twist.linear.x = l_s+delta
            twist.angular.z = 0.0
            pub.publish(twist)
    else: 
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        pub.publish(twist)

if __name__ == '__main__':

    rospy.init_node('lds_distance-servoing')
    rospy.Subscriber(subTopicName, cardinal_direction, front_dist)
    rospy.spin()