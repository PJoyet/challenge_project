#!/usr/bin/env python
# -*- coding: utf-8 -*-

from numpy import mean

import rospy
from sensor_msgs.msg import LaserScan
from challenge_project.msg import cardinal_direction

subTopicName = "/scan"
pubTopicName = "/lds_distance"


def distance(lds):
    pub   = rospy.Publisher(pubTopicName, cardinal_direction, queue_size=3)
    dist  = cardinal_direction()
    delta = int(len(lds.ranges)/16)+1
    N1  = []
    N2  = []
    NW = []
    W  = []
    SW = []
    S  = []
    SE = []
    E = []
    NE = []

    for i in range (15*delta,len(lds.ranges)) :
        if lds.ranges[i] < lds.range_max and lds.ranges[i] > lds.range_min:
            N1.append(lds.ranges[i])

    for i in range (0,delta) :
        if lds.ranges[i] < lds.range_max and lds.ranges[i] > lds.range_min:
            N2.append(lds.ranges[i])

    for i in range (delta,3*delta) :
        if lds.ranges[i] < lds.range_max and lds.ranges[i] > lds.range_min:
            NW.append(lds.ranges[i])

    for i in range (3*delta,5*delta) :
        if lds.ranges[i] < lds.range_max and lds.ranges[i] > lds.range_min:
            W.append(lds.ranges[i])

    for i in range (5*delta,7*delta) :
        if lds.ranges[i] < lds.range_max and lds.ranges[i] > lds.range_min:
            SW.append(lds.ranges[i])

    for i in range (7*delta,9*delta) :
        if lds.ranges[i] < lds.range_max and lds.ranges[i] > lds.range_min:
            S.append(lds.ranges[i])

    for i in range (9*delta,11*delta) :
        if lds.ranges[i] < lds.range_max and lds.ranges[i] > lds.range_min:
            SE.append(lds.ranges[i])

    for i in range (11*delta,13*delta) :
        if lds.ranges[i] < lds.range_max and lds.ranges[i] > lds.range_min:
            E.append(lds.ranges[i])

    for i in range (13*delta,15*delta) :
        if lds.ranges[i] < lds.range_max and lds.ranges[i] > lds.range_min:
            NE.append(lds.ranges[i])

    if len(N1)==0:
        N1 = lds.range_max
    if len(N2)==0:
        N2 = lds.range_max
    if len(S)==0:
        S = lds.range_max
    if len(E)==0:
        E = lds.range_max
    if len(W)==0:
        W = lds.range_max
    if len(NW)==0:
        NW = lds.range_max
    if len(NE)==0:
        NE = lds.range_max
    if len(SW)==0:
        SW = lds.range_max
    if len(SE)==0:
        SE = lds.range_max

    dist.E  = mean(E)
    dist.NW = mean(NW)
    dist.W  = mean(W)
    dist.SW = mean(SW)
    dist.S  = mean(S)
    dist.SE = mean(SE)
    dist.N  = (mean(N1)+mean(N2))/2
    dist.NE = mean(NE)



    pub.publish(dist)
    print N1,N2,S,E,W,NW,NE,SW,SE

if __name__ == '__main__':
    rospy.init_node('lds_data')
    sub = rospy.Subscriber(subTopicName, LaserScan, distance)

    rospy.spin()