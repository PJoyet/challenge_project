#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy, cv2
import numpy as np
from math import tan,pi,sqrt,atan2,atan

BLACK = np.uint8([[000,000,000],[ 55, 55, 55]])
WHITE = np.uint8([[200,200,200],[255,255,255]])
RED   = np.uint8([[000,000,250],[000,000,255]])
GREEN = np.uint8([[000,200,000],[000,255,000]])
BLUE  = np.uint8([[100,000,000],[255,000,000]])
YELLOW= np.uint8([[000,230,230],[000,255,255]])
ORANGE= np.uint8([[000, 75,230],[ 25,100,255]])


def LineDetection(image,color,colorformat="rgb",nbPoints=20):
    height = image.shape[0]
    width  = image.shape[1]
    points = [(0,0)]

    if color == 'BLACK' or color == 'black':
        color = BLACK
    elif color == 'WHITE' or color == 'white':
        color = WHITE
    elif color == 'RED' or color == 'red':
        color = RED
    elif color == 'GREEN' or color == 'green':
        color = GREEN
    elif color == 'BLUE' or color == 'blue':
        color = BLUE 
    elif color == 'YELLOW' or color == 'yellow':
        color = YELLOW
    elif color == 'ORANGE' or color == 'orange':
        color = ORANGE 
    else :
        color = np.fliplr(np.uint8(color))

    if colorformat == 'HSV' or colorformat == 'hsv':
        
        color = np.fliplr(color)
        color = cv2.cvtColor(np.array([color]), cv2.COLOR_BGR2HSV)[0]


    for i in range(height//nbPoints,height,height//nbPoints):
        row = image[i-height//nbPoints:i]
        mask = cv2.inRange(row,color[0],color[1])
        M = cv2.moments(mask)
        if M['m00'] > 0:
            cx = int (M[ 'm10' ] /M[ 'm00' ] )
            cy = int (M[ 'm01' ] /M[ 'm00' ] )
            points.append((cx,cy+i-height//nbPoints))
    return points[::-1]	

def get_angle(pt1,pt2):
    X = pt2[0]-pt1[0]
    Y = pt2[1]-pt1[1]
    return (atan2(Y,X)-pi/2)

def PointRallyingSpeed(point1,point2,imgshape,gains,camInfo):
    height = imgshape[0]
    width   = imgshape[1]

    dist_bas =  camInfo['height']*tan((pi-camInfo['fov'])/2-camInfo['pitch'])
    dist_haut = camInfo['height']*tan(pi/2-camInfo['pitch']) 

    delta_X1 = -(point1[0]-width//2)
    delta_Y1 = height-point1[1]
    delta_X2 = -(point2[0]-width//2)
    delta_Y2 = height-point2[1]

    Y1 = delta_Y1*(dist_haut-dist_bas)/(height/2)+dist_bas
    dist_x1 = Y1*tan(camInfo['fov']/2)
    X1 = delta_X1*(dist_x1/width/2)

    Y2 = delta_Y2*(dist_haut-dist_bas)/(height/2)+dist_bas
    dist_x2 = Y2*tan(camInfo['fov']/2)
    X2 = delta_X2*(dist_x2/width/2)

    pt1 = (X1,Y1)
    pt2 = (X2,Y2)

    angle1 = get_angle((0,0),pt1) 
    angle2 = get_angle(pt1,pt2)

    linear = sqrt(pt1[0]**2 + pt1[1]**2)
    linear = (linear*gains[0])
    angular = (- (gains[1]*angle1) - (gains[2]*angle2))

    return linear,angular