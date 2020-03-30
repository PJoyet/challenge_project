#!/usr/bin/env python
# -*- coding: utf-8 -*-
import numpy
from math import pi
import sys

import rospy
from sensor_msgs.msg import LaserScan
from challenge_project.msg import FBLR_distance as Dist

def rad2deg(rad):
    return round(rad*(180/pi))
 
def callback(msg,pub):
    dist = Dist()
    angle = rospy.get_param("/mean_angle")/2
    delta = int(round(angle*len(msg.ranges)/(rad2deg(msg.angle_max)-rad2deg(msg.angle_min))))
    
    F = []
    B = []
    L1 = []
    L2 = []
    R = []

    for i in range (3*len(msg.ranges)//4-delta,3*len(msg.ranges)//4+delta) :
	if msg.ranges[i] < 30 and msg.ranges[i] > -30:
	    F.append(msg.ranges[i])

    for i in range (len(msg.ranges)//4-delta,len(msg.ranges)//4+delta) :
	if msg.ranges[i] < 30 and msg.ranges[i] > -30:
	    B.append(msg.ranges[i])

    for i in range (0,delta) :
	if msg.ranges[i] < 30 and msg.ranges[i] > -30:
	    L1.append(msg.ranges[i])

    for i in range (delta,-1,-1) :
	if msg.ranges[i] < 30 and msg.ranges[i] > -30:
	    L2.append(msg.ranges[i])

    for i in range (len(msg.ranges)//2-delta,len(msg.ranges)//2+delta) :
	if msg.ranges[i] < 30 and msg.ranges[i] > -30:
	    R.append(msg.ranges[i])

    dist.Front = numpy.mean(F)
    dist.Back = numpy.mean(B)
    dist.Left = (numpy.mean(L1) + numpy.mean(L2))/2
    dist.Right = numpy.mean(R)
	
    #dist.Front = numpy.mean(msg.ranges[3*len(msg.ranges)//4-delta:3*len(msg.ranges)//4+delta])
    #dist.Back = numpy.mean(msg.ranges[len(msg.ranges)//4-delta:len(msg.ranges)//4+delta])
    #dist.Left = (numpy.mean(msg.ranges[0:delta])+numpy.mean(msg.ranges[delta:-1]))/2
    #dist.Right = numpy.mean(msg.ranges[len(msg.ranges)//2-delta:len(msg.ranges)//2+delta])
    pub.publish(dist)

    print("1",dist.Front) 
    print("2",dist.Back)
    print("3",dist.Left)
    print("4",dist.Right)

if __name__ == '__main__':
    rospy.init_node('lidar_data')
    pub = rospy.Publisher('front_lidar', Dist, queue_size=3)

    if sys.argv[1] == 'turtlebot':
        sub = rospy.Subscriber('/scan', LaserScan, callback,pub)
    if sys.argv[1] == 'robot_with_sensor':
        sub = rospy.Subscriber('/mybot/scan', LaserScan, callback,pub)
    rospy.spin()
