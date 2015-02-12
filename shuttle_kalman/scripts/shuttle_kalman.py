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
from geometry_msgs.msg import PointStamped
from visualization_msgs.msg import Marker

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
    for var in range(0, 200):
        p.x = k.mu[0]
        p.y = k.mu[1]
        p.z = k.mu[2]
        point_list.append(copy.copy(p))
        if k.mu[2] <= 0:
            break
        k.predict(0.005)
        
    updateMarker(point_list)
    

def callback(msg):
    global updated
    global lastmsg
    global s
    
    if updated == 0:
        updated = 1;
    
    else:
        dt = msg.header.stamp.to_sec() - lastmsg.header.stamp.to_sec()
        if dt > 0.3:
            updated = 1;
            
        elif updated == 1:
            updated = 2
            s = shuttle.Shuttle( np.mat([
                    [msg.point.x],[msg.point.y],[msg.point.z],
                    [ (msg.point.x - lastmsg.point.x)/dt],[(msg.point.y - lastmsg.point.y)/dt],[(msg.point.z - lastmsg.point.z)/dt],
                    [0],[0],[0]
                ]) )
        else:
            for var in range(0, 10):
                s.predict(dt/10)
            s.update( np.mat([ [msg.point.x],[msg.point.y],[msg.point.z] ]) )
            
            print s.mu.T
            
            predictOrbit(copy.copy(s.mu))
        
    lastmsg = msg
        
if __name__ == '__main__':

    argv = rospy.myargv(sys.argv)
    rospy.init_node('shutle_kalman')
    
    s = shuttle.Shuttle( np.mat([[0],[0],[0],[0],[0],[0],[0],[0],[0]]) )
    
    lastmsg = PointStamped()
    lastmsg.header.stamp = rospy.Time.now()
    
    markerPub = rospy.Publisher('/shuttle/kalman', Marker, queue_size=1)
    
    rospy.Subscriber("/shuttle/point", PointStamped, callback)
    rospy.spin()
    
    