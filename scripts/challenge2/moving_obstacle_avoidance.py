#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""auteur: Pierre Joyet (3407684) - Marine CORNET (3531423) - Projet ROS- M1 SAR 2020"""

import rospy
import numpy as np
from geometry_msgs.msg import Twist
from challenge_project.msg import cardinal_direction

pubTopicName = "/cmd_vel"
subTopicName = "/lds_distance"

def front_dist(lds):
    pub = rospy.Publisher(pubTopicName,Twist,queue_size=3)
    l_s = rospy.get_param("/linear_scale")
    a_s = rospy.get_param("/angular_scale")
    d = rospy.get_param("/distance_constante")
    twist = Twist()

    minDist = min(lds.N,lds.S,lds.E,lds.W,lds.NW,lds.NE,lds.SE,lds.SW)

    if minDist<1.2:
        if minDist==lds.N:
            print 'N',lds.N
            delta = (lds.N-d)*0.6

            if lds.N <= d:
                twist.linear.x = -l_s+delta
                twist.angular.z = 0.0
                pub.publish(twist)

            else :
                twist.linear.x = l_s+delta
                twist.angular.z = 0.0
                pub.publish(twist)

        elif minDist==lds.S:
            print 'S',lds.S
            twist.linear.x = 0
            twist.angular.z = np.pi*a_s
            pub.publish(twist)

        elif minDist==lds.E:
            print 'E',lds.E
            twist.linear.x = 0
            twist.angular.z = -(np.pi/2)*a_s
            pub.publish(twist)

        elif minDist==lds.W:
            print 'W',lds.W
            twist.linear.x = 0
            twist.angular.z = (np.pi/2)*a_s
            pub.publish(twist)

        elif minDist==lds.NW:
            print 'NW',lds.NW
            twist.linear.x = 0
            twist.angular.z = (np.pi/4)*a_s
            pub.publish(twist)

        elif minDist==lds.NE:
            print 'NE',lds.NE
            twist.linear.x = 0
            twist.angular.z = -(np.pi/4)*a_s
            pub.publish(twist)

        elif minDist==lds.SW:
            print 'SW',lds.SW
            twist.linear.x = 0
            twist.angular.z = (2*np.pi/3)*a_s
            pub.publish(twist)

        elif minDist==lds.SE:
            print 'SE',lds.SE
            twist.linear.x = 0
            twist.angular.z = -(2*np.pi/3)*a_s
            pub.publish(twist)

    else:
        print minDist
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        pub.publish(twist)          

if __name__ == '__main__':

    rospy.init_node('lds_moving-obstacle-avoidance')
    rospy.Subscriber(subTopicName, cardinal_direction, front_dist)
    rospy.spin()