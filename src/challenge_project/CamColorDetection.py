#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""author: Pierre Joyet (3407684) - Marine CORNET (3531423) - ROS Project - M1 SAR 2020"""

import rospy, cv2
import numpy as np
from math import tan,pi,sqrt,atan2,atan

#Colors BGR
BLACK = np.uint8([[000,000,000],[ 55, 55, 55]])
WHITE = np.uint8([[200,200,200],[255,255,255]])
RED   = np.uint8([[000,000,250],[000,000,255]])
GREEN = np.uint8([[000,200,000],[000,255,000]])
BLUE  = np.uint8([[100,000,000],[255,000,000]])
YELLOW= np.uint8([[000,230,230],[000,255,255]])
ORANGE= np.uint8([[000, 75,230],[ 25,100,255]])


def LineDetection(image,color,colorformat="rgb",nbPoints=20):
    """
    #Returns a list of coordinate of color center.

    #Parameters:
    #    image (list): The image where is the color.
    #    color (str/tuple): The color to find (can be RGB tuple)
    #    colorformat (str): The color format of the image
    #    nbPoints (int): the maximum number of point to find

    #Returns:
    #    points(list): The list of coordinate.  
    """

    # Shape of the image
    height = image.shape[0]
    width  = image.shape[1]

    # Initialization of point list
    points = [(0,0)]

    # Color choise
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
        color = np.fliplr(np.uint8(color)) # RGB to BGR convertion

    if colorformat == 'HSV' or colorformat == 'hsv':
        color = np.fliplr(color) # BGR to RGB convertion for hsv conversion
        color = cv2.cvtColor(np.array([color]), cv2.COLOR_BGR2HSV)[0]

    # Moment calculation,for nbPoints strip, of the mask to find the center of the color
    for i in range(height//nbPoints,height,height//nbPoints):
        strip = image[i-height//nbPoints:i]
        mask = cv2.inRange(strip,color[0],color[1])
        M = cv2.moments(mask)
        if M['m00'] > 0:
            cx = int (M[ 'm10' ] /M[ 'm00' ] )
            cy = int (M[ 'm01' ] /M[ 'm00' ] )
            points.append((cx,cy+i-height//nbPoints))

    return points[::-1]	# Return reverse list

def get_angle(pt1,pt2):
    # Return angle of vector 
    X = pt2[0]-pt1[0]
    Y = pt2[1]-pt1[1]
    return (atan2(Y,X)-pi/2)

def PointRallyingSpeed(point1,point2,imgshape,gains,camInfo):
    """
    #Returns the linear and angular speed for reaching a real physical point
    # from two points camera image coordinates.

    #Parameters:
    #    point1 (tuple): The points to reach for the robot.
    #    point2 (tuple): The point for the reach angle 
    #    imageshape (tuple): The shape of the image
    #    gains (tuple): The gains of angular and linear control speed
    #    camInfo (Dict): The camera information

    #Returns:
    #    linear (float): The linear speed.
    #     anglular (float): The angular speed.  
    """

    # Shape of the image
    height = imgshape[0]
    width   = imgshape[1]

    # Physical distance calculation of the up and bottom of the image from trigonometry
    dist_bottom =  camInfo['height']*tan((pi-camInfo['fov'])/2-camInfo['pitch'])
    dist_up = camInfo['height']*tan(pi/2-camInfo['pitch']) 

    # Centering and botomm-up the origin of the two points
    delta_X1 = -(point1[0]-width//2)
    delta_Y1 = height-point1[1]
    delta_X2 = -(point2[0]-width//2)
    delta_Y2 = height-point2[1]


    # Physical distance calculation of the point 1 from trigonometry
    Y1 = delta_Y1*(dist_up-dist_bottom)/(height/2)+dist_bottom
    dist_x1 = Y1*tan(camInfo['fov']/2)
    X1 = delta_X1*(dist_x1/width/2)

    # Physical distance calculation of the point 2 from trigonometry
    Y2 = delta_Y2*(dist_up-dist_bottom)/(height/2)+dist_bottom
    dist_x2 = Y2*tan(camInfo['fov']/2)
    X2 = delta_X2*(dist_x2/width/2)

    # Making tuple coodinates
    pt1 = (X1,Y1)
    pt2 = (X2,Y2)

    # Getting the angle of point 1 and origin
    angle1 = get_angle((0,0),pt1) 
    # Getting the angle of point 1 and point 2
    angle2 = get_angle(pt1,pt2)

    # Calculation of linear and angular speed
    linear = sqrt(pt1[0]**2 + pt1[1]**2)
    linear = (linear*gains[0])
    angular = (- (gains[1]*angle1) - (gains[2]*angle2))

    return linear,angular