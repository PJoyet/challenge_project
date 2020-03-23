"""auteur: Pierre Joyet (3407684) - Marine CORNET (3531423) - Projet ROS- M1 SAR 2020"""

import rospy
import sys, termios, tty
from geometry_msgs.msg import Twist
from challenge_project.msg import FBLR_distance as Dist


def front_dist(data):
    pub = rospy.Publisher('/cmd_vel',Twist,queue_size=3)
    l_s = rospy.get_param("/linear_scale")
    d = rospy.get_param("/distance_constante")
    twist = Twist()

    if data.Left <= d + 0.21:
        twist.linear.x = - l_s
        twist.angular.z = 0.0
        pub.publish(twist)

    elif data.Back <= d + 0.21:
        twist.linear.x = - l_s
        twist.angular.z = 0.0
        pub.publish(twist)

    elif data.Front <= d + 0.21:
        twist.linear.x = - l_s
        twist.angular.z = 0.0
        pub.publish(twist)

    elif data.Right <= d + 0.21:
        twist.linear.x = - l_s
        twist.angular.z = 0.0
        pub.publish(twist)

    else :
	twist.linear.x = l_s
        twist.angular.z = 0.0
        pub.publish(twist) 

    
   


if __name__ == '__main__':

    rospy.init_node('teleop_lidar-emergency')

    rospy.Subscriber("front_lidar", Dist, front_dist)

    rospy.spin()
