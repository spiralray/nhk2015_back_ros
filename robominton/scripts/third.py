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

import std_msgs.msg
from geometry_msgs.msg import Point
from geometry_msgs.msg import PoseStamped
from _shuttle_msg import shuttle_msg
from geometry_msgs.msg import PointStamped

from visualization_msgs.msg import Marker

import math

racket_length = 0.480

mode = 0

home_x = 1.3
home_y = -4.5
home_z = 0

def publishHome():
    msg = PointStamped()
    msg.header.frame_id = "/map"
    msg.header.stamp = rospy.Time.now()
    msg.point.x = home_x
    msg.point.y = home_y
    msg.point.z = home_z
    pointPub.publish(msg)

def getTransformMatrixToRacketCoordinate():
    R = np.mat([
         [1,0,0,-pose.position.x],
         [0,1,0,-pose.position.y],
         [0,0,1,0],
         [0,0,0,1]
        ])
    
    yaw = -math.atan2(2.0*(pose.orientation.x*pose.orientation.y + pose.orientation.w*pose.orientation.z), pose.orientation.w*pose.orientation.w + pose.orientation.x*pose.orientation.x - pose.orientation.y*pose.orientation.y - pose.orientation.z*pose.orientation.z);
    #print yaw
    
    Rt = np.mat([
          [math.cos(-yaw),math.sin(-yaw),0,0],
          [-math.sin(-yaw),math.cos(-yaw),0,0],
          [0,0,1,0],
          [0,0,0,1]
        ])
    A =  np.mat([
         [1,0,0,0],
         [0,1,0,0.],
         [0,0,1,-0.75],
         [0,0,0,1]
        ])
    return A*Rt*R
    
def predictOrbit(mu):
    k = shuttle.Shuttle( mu )
    
    T = getTransformMatrixToRacketCoordinate()
    
    p = np.mat( [[k.mu[0]],[k.mu[1]],[k.mu[2]], [1] ] )
    t=T*p
    
    msg = PointStamped()
    msg.header.frame_id = "/map"
    msg.header.stamp = rospy.Time.now()
    msg.point.x = k.mu[0]
    msg.point.y = k.mu[1]
    msg.point.z = k.mu[2]
    shuttlePub.publish(msg)
    
    if k.mu[4] > 0:
        #rospy.logwarn('Already recieved')
        publishHome()
        return
    
    if t[2,0] <= -4:
        #rospy.logwarn('Shuttle is under ground') 
        publishHome()
        swingPub.publish( std_msgs.msg.Float32(0) )
        return
    
    elif t[2,0] <= 0.04:
        #rospy.logwarn('Shuttle is under ground')
        if math.sqrt(t[0,0]**2 + t[1,0]**2) < 2.0:
            swingPub.publish( std_msgs.msg.Float32(1.0) )
        return
    
    #print t.T
    
    swingPub.publish( std_msgs.msg.Float32(0) )
    
    for var in range(0, 500):
        p = np.mat( [[k.mu[0]],[k.mu[1]],[k.mu[2]], [1] ] )
        
        if k.mu[2] < -1:
            #rospy.logwarn( 'Shuttle has passed through the racket')
            publishHome()
            return
        
        t=T*p
        if t[2,0] <= 0:
            
            msg = PointStamped()
            msg.header.frame_id = "/map"
            msg.header.stamp = rospy.Time.now() + rospy.Duration(0.01 * var)
            msg.point.x = k.mu[0]
            msg.point.y = k.mu[1]
            msg.point.z = k.mu[2]
            
            if msg.point.x > 3:
                msg.point.x = 3
            elif msg.point.x < -3:
                msg.point.x = -3
                
            if msg.point.y > -1:
                msg.point.y = -1
            elif msg.point.y < -6:
                msg.point.y = -6
            
            pointPub.publish(msg)
            
            #print msg.header.stamp.to_sec()
            
            return
        k.predict(0.01)
        
    publishHome()

def poseCallback(msg):
    global pose
    pose = msg.pose
    
def modeCallback(msg):
    global mode
    mode = msg.data

def shuttleCallback(msg):
    global time
    time = msg.stamp.to_sec()
    for i in range(0, 9):
        s.mu[i,0] = msg.data[i]

def time_callback(event):
    global time
    
    now = rospy.Time.now().to_sec()
    t = now - time
    time = now
    
    for var in range(0, 4):
        s.predict(t/4)
    
    predictOrbit(copy.copy(s.mu))
    
if __name__ == '__main__':

    argv = rospy.myargv(sys.argv)
    rospy.init_node('shutle_kalman_third')
    
    pose = PoseStamped().pose
    s = shuttle.Shuttle( np.mat([[0],[0],[0],[0],[0],[0],[0],[0],[0]]) )
        
    rospy.Subscriber("/robot/pose", PoseStamped, poseCallback)
    
    time = rospy.Time.now().to_sec()
    rospy.Subscriber("/shuttle/status", shuttle_msg, shuttleCallback)
    
    pointPub = rospy.Publisher('/robot/targetpoint', PointStamped, queue_size=1)
    shuttlePub = rospy.Publisher('/shuttle/now', PointStamped, queue_size=1)
    swingPub = rospy.Publisher('/auto/swing', std_msgs.msg.Float32, queue_size=1)
    
    rospy.Timer(rospy.Duration(0.02), time_callback)
    
    rospy.spin()
    
    
