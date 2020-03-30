#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy, cv2
import numpy as np
from math import tan,pi,sqrt,atan2,atan
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class ColorDetection:
    
    def __init__(self,image):
        self.img = image
        self.height = image.shape[0]
        self.width = image.shape[1]
        self.bridge = CvBridge()
        
    def get_color_center(color):
        cx = None
        cy = None
        mask = cv2.inRange(self.img,color[0],color[1])
        M = cv2.moments(mask)
        if M['m00'] > 0:
            cx = int (M[ 'm10' ] /M[ 'm00' ] )
            cy = int (M[ 'm01' ] /M[ 'm00' ] )
        return cx,cy
        

