#!/usr/bin/env python
# -*- coding: utf-8 -*-

'''
Created on 2015/02/11

@author: spiralray
'''

import shuttle

import numpy as np
import copy

import sys
import roslib
import rospy
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker
from _shuttle_msg import shuttle_msg

updated = 0

def updateMarker(point_list):
    marker = Marker()
    marker.header.frame_id = "/map"
    marker.points = point_list
    marker.type = marker.POINTS
    marker.action = marker.ADD
    marker.scale.x = 0.1
    marker.scale.y = 0.1
    marker.scale.z = 0.1
    marker.color.a = 1.0
    marker.color.r = 1.0
    marker.color.g = 1.0
    marker.color.b = 0.0
    markerPub.publish(marker)
    
def predictOrbit(mu):
    point_list = []
    p = Point()
    k = shuttle.Shuttle( mu )
    for var in range(0, 1000):
        p.x = k.mu[0]
        p.y = k.mu[1]
        p.z = k.mu[2]
        point_list.append(copy.copy(p))
        if k.mu[2] <= 0:
            break
        k.predict(0.01)
        
    updateMarker(point_list)
    

def callback(msg):
    #print msg
    
    for i in range(0, 9):
        s.mu[i,0] = msg.data[i]
    predictOrbit(copy.copy(s.mu))

def time_callback(event):
    for i in range(0, 10):
        s.predict(0.005)
    predictOrbit(copy.copy(s.mu))
    
if __name__ == '__main__':

    argv = rospy.myargv(sys.argv)
    rospy.init_node('shutle_kalman_disp')
    
    s = shuttle.Shuttle( np.mat([[0],[0],[0],[0],[0],[0],[0],[0],[0]]) )
    
    markerPub = rospy.Publisher('/shuttle/orbit', Marker, queue_size=1)
    
    rospy.Subscriber("/shuttle/status", shuttle_msg, callback)
    rospy.Timer(rospy.Duration(0.05), time_callback)
    rospy.spin()
    
    