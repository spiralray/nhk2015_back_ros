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

racket_length = 0.480

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
          [0,math.cos(-math.pi/4),math.sin(-math.pi/4),0],
          [0,-math.sin(-math.pi/4),math.cos(-math.pi/4),0],
          [0,0,0,1]
        ])
    return At*A*Rt*R
    
def predictOrbit(mu):
    k = shuttle.Shuttle( mu )
    
    T = getTransformMatrixToRacketCoordinate()
    
    p = np.mat( [[k.mu[0]],[k.mu[1]],[k.mu[2]], [1] ] )
    t=T*p
    if t[2,0] <= -1.5:
        roll_pub.publish( std_msgs.msg.Float32(-math.pi) )
        slide_pub.publish( std_msgs.msg.Float32(0) )
        return
    
    for var in range(0, 200):
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
                
            while racket_spin - roll > math.pi:
                racket_spin -= 2*math.pi
                
            while racket_spin - roll < -math.pi:
                racket_spin += 2*math.pi
                
            roll_pub.publish( std_msgs.msg.Float32(racket_spin) )
            
            #rospy.logerr("%d", var)
            
            if var < 60:
                slide_pub.publish( std_msgs.msg.Float32(slide_x) )
            else:
                slide_pub.publish( std_msgs.msg.Float32(0.0) )
                
            return
        k.predict(0.01)
        
    roll_pub.publish( std_msgs.msg.Float32(-math.pi) )
    slide_pub.publish( std_msgs.msg.Float32(0) )

def poseCallback(msg):
    global pose
    #print msg
    pose = msg.pose
    
def modeCallback(msg):
    global mode
    #print msg
    mode = msg.data

def enc1Callback(msg):
    global slide
    #print msg
    slide = msg.data
    
def enc2Callback(msg):
    global roll
    #print msg
    roll = msg.data
    
def shuttleCallback(msg):
    global time
    #print msg
    time = msg.stamp.to_sec()
    for i in range(0, 9):
        s.mu[i,0] = msg.data[i]
    #predictOrbit(copy.copy(s.mu))

def time_callback(event):
    s.predict(0.01)
    s.predict(0.01)
    if mode == 0:
        servearm_pub.publish( std_msgs.msg.Float32(0.01) )
        predictOrbit(copy.copy(s.mu))
        
    elif mode == 1:
        roll_target = -2.43
        slide_target = -0.079
        roll_pub.publish( std_msgs.msg.Float32(roll_target) )
        slide_pub.publish( std_msgs.msg.Float32(slide_target) )
        if abs( roll_target - roll ) < math.pi/180 and abs( slide_target - slide ) < 0.01:
            servearm_pub.publish( std_msgs.msg.Float32(0.29) )
    
if __name__ == '__main__':

    argv = rospy.myargv(sys.argv)
    rospy.init_node('shutle_kalman_arm')
    
    pose = PoseStamped().pose
    s = shuttle.Shuttle( np.mat([[0],[0],[0],[0],[0],[0],[0],[0],[0]]) )
    
    slide_pub = rospy.Publisher('/mb1/motor1', std_msgs.msg.Float32, queue_size=1)
    roll_pub = rospy.Publisher('/mb1/motor2', std_msgs.msg.Float32, queue_size=1)
    
    servearm_pub = rospy.Publisher('/mb2/motor', std_msgs.msg.Float32, queue_size=1)
    
    rospy.Subscriber("/robot/pose", PoseStamped, poseCallback)
    
    mode = 0
    rospy.Subscriber("/robot/mode", std_msgs.msg.Int32, modeCallback)
    
    slide = 0
    rospy.Subscriber("/mb1/enc1", std_msgs.msg.Float32, enc1Callback)
    roll = 0
    rospy.Subscriber("/mb1/enc2", std_msgs.msg.Float32, enc2Callback)
    
    rospy.Subscriber("/shuttle/status", shuttle_msg, shuttleCallback)
    
    rospy.Timer(rospy.Duration(0.02), time_callback)
    
    rospy.spin()
    
    
