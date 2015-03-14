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

import math

racket_length = 0.560

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
         [0,1,0,-0.222],
         [0,0,1,-0.658],
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
    
    p = np.mat( [[k.mu[0]],[k.mu[1]],[k.mu[2]], [1] ] )
    t=T*p
    if t[2,0] <= 0:
        return
    
    for var in range(0, 300):
        '''
        if k.mu[2] < 0:
            break
        '''
        p = np.mat( [[k.mu[0]],[k.mu[1]],[k.mu[2]], [1] ] )
        t=T*p
        if t[2,0] <= 0:
            #print t.T
            '''
            y = t[1,0]/racket_length
            if y > 1:
                racket_spin = 0
            elif y < -1:
                racket_spin = math.pi
            else:
                racket_spin = math.fabs( math.pi/2 - math.asin(y))
            if t[0,0] < 0:
                racket_spin = -racket_spin
            '''
            
            racket_spin = -math.atan2(-t[0,0], -t[1,0])
            
            racket_x = math.sin(racket_spin)*racket_length
            slide_x = t[0,0] - racket_x
                
                
            if slide_x > 0.24:
                slide_x = 0.24
            elif slide_x < -0.24:
                slide_x = -0.24
                
            roll_pub.publish( std_msgs.msg.Float32(racket_spin) )
            slide_pub.publish( std_msgs.msg.Float32(slide_x) )
                
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
    #predictOrbit(copy.copy(s.mu))

def time_callback(event):
    s.predict(0.01)
    s.predict(0.01)
    predictOrbit(copy.copy(s.mu))
    
if __name__ == '__main__':

    argv = rospy.myargv(sys.argv)
    rospy.init_node('shutle_kalman_arm')
    
    pose = PoseStamped().pose
    s = shuttle.Shuttle( np.mat([[0],[0],[0],[0],[0],[0],[0],[0],[0]]) )
    
    slide_pub = rospy.Publisher('/mb1/motor1', std_msgs.msg.Float32, queue_size=1)
    roll_pub = rospy.Publisher('/mb1/motor2', std_msgs.msg.Float32, queue_size=1)
    
    rospy.Subscriber("/robot/pose", PoseStamped, poseCallback)
    rospy.Subscriber("/shuttle/status", shuttle_msg, shuttleCallback)
    
    rospy.Timer(rospy.Duration(0.02), time_callback)
    
    rospy.spin()
    
    