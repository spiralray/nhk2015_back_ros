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
from geometry_msgs.msg import PoseStamped
from _shuttle_msg import shuttle_msg
import math

def getTransformMatrixToRacketCoordinate():
    R = np.mat([
         [1,0,0,-pose.position.x],
         [0,1,0,-pose.position.y],
         [0,0,1,0],
         [0,0,0,1]
        ])
    
    yaw = math.atan2(2.0*(pose.orientation.x*pose.orientation.y + pose.orientation.w*pose.orientation.z), pose.orientation.w*pose.orientation.w + pose.orientation.x*pose.orientation.x - pose.orientation.y*pose.orientation.y - pose.orientation.z*pose.orientation.z);
    #print yaw
    
    Rt = np.mat([
          [math.cos(-yaw),0,math.sin(-yaw),0],
          [0,1,0,0],
          [-math.sin(-yaw),0,math.cos(-yaw),0],
          [0,0,0,1]
        ])
    A =  np.mat([
          [1,0,0,0],
         [0,1,0,-0.206],
         [0,0,1,-0.726],
         [0,0,0,1]
        ])
    At =  np.mat([
          [1,0,0,0],
          [0,math.cos(-math.pi/3),math.sin(-math.pi/3),0],
          [0,-math.sin(-math.pi/3),math.cos(-math.pi/3),0],
          [0,0,0,1]
        ])
    return At*A*Rt*R
    
def predictOrbit(mu):
    k = shuttle.Shuttle( mu )
    
    T = getTransformMatrixToRacketCoordinate()
    
    for var in range(0, 200):
        if k.mu[2] < 0:
            break
        p = np.mat( [[k.mu[0]],[k.mu[1]],[k.mu[2]], [1] ] )
        t=T*p
        if t[2,0] <= 0:
            print t.T
            break
        k.predict(0.01)

def poseCallback(msg):
    global pose
    #print msg
    pose = msg.pose
    
def shuttleCallback(msg):
    #print msg
    
    for i in range(0, 9):
        s.mu[i,0] = msg.data[i]
    predictOrbit(copy.copy(s.mu))

    
if __name__ == '__main__':

    argv = rospy.myargv(sys.argv)
    rospy.init_node('shutle_kalman_arm')
    
    pose = PoseStamped().pose
    s = shuttle.Shuttle( np.mat([[0],[0],[0],[0],[0],[0],[0],[0],[0]]) )
    
    rospy.Subscriber("/robot/pose", PoseStamped, poseCallback)
    rospy.Subscriber("/shuttle/status", shuttle_msg, shuttleCallback)
    rospy.spin()
    
    