#!/usr/bin/env python
# -*- coding: utf-8 -*-

'''
Created on 2015/02/11

@author: spiralray
'''

import shuttle

import numpy as np

import sys
import roslib
import rospy
from geometry_msgs.msg import Point
from geometry_msgs.msg import PointStamped
from _shuttle_msg import shuttle_msg

updated = 0
    

def callback(msg):
    global updated
    global lastmsg
    global s
    
    if updated == 0:
        updated = 1;
    
    else:
        dt = msg.header.stamp.to_sec() - lastmsg.header.stamp.to_sec()
        if dt < 0:
            print "recieved data is too old!"
        elif dt > 0.3:
            updated = 1;
            
        elif updated == 1:
            updated = 2
            s = shuttle.Shuttle( np.mat([
                    [msg.point.x],[msg.point.y],[msg.point.z],
                    [ (msg.point.x - lastmsg.point.x)/dt],[(msg.point.y - lastmsg.point.y)/dt],[(msg.point.z - lastmsg.point.z)/dt],
                    [0],[0],[0]
                ]) )
        else:
            for var in range(0, 20):
                s.predict(dt/20)
            dt = msg.header.stamp.to_sec() - lastmsg.header.stamp.to_sec()
            s.update( np.mat([ [msg.point.x],[msg.point.y],[msg.point.z] ]) )
            #print s.mu.T
            
            pubmsg = shuttle_msg()
            pubmsg.stamp = msg.header.stamp
            pubmsg.data = []
            for i in range(0, 9):
                pubmsg.data.append(s.mu[i,0])
            pub.publish(pubmsg)
            
        
    lastmsg = msg
        
if __name__ == '__main__':

    argv = rospy.myargv(sys.argv)
    rospy.init_node('shutle_kalman')
    
    s = shuttle.Shuttle( np.mat([[0],[0],[0],[0],[0],[0],[0],[0],[0]]) )
    
    lastmsg = PointStamped()
    lastmsg.header.stamp = rospy.Time.now()
    
    pub = rospy.Publisher('/shuttle/status', shuttle_msg, queue_size=1)
    rospy.Subscriber("/shuttle/point", PointStamped, callback)
    rospy.Subscriber("/shuttle/point2", PointStamped, callback)
    rospy.spin()
    
    