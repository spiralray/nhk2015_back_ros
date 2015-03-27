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
import tf

import std_msgs.msg
from geometry_msgs.msg import Point
from geometry_msgs.msg import PoseStamped
from _shuttle_msg import shuttle_msg
from geometry_msgs.msg import PointStamped

from visualization_msgs.msg import Marker

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
         [0,0,1,-0.670],
         [0,0,0,1]
        ])
    At =  np.mat([
          [1,0,0,0],
          [0,math.cos(-math.pi/4),math.sin(-math.pi/4),0],
          [0,-math.sin(-math.pi/4),math.cos(-math.pi/4),0],
          [0,0,0,1]
        ])
    Rev =  np.mat([
         [1,0,0,0],
         [0,-1,0,0],
         [0,0,1,0],
         [0,0,0,1]
        ])
    return Rev*At*A*Rt*R
    

def poseCallback(msg):
    global pose
    pose = msg.pose

def enc1Callback(msg):
    global slide
    slide = msg.data
    
def enc2Callback(msg):
    global roll
    roll = msg.data
    
def enc3Callback(msg):
    global swing
    swing = msg.data

def time_callback(event):
    
    T = getTransformMatrixToRacketCoordinate()
    p = np.mat( [
        [slide],
        [0],
        [0],
        [1] ] )
    
    t = np.linalg.solve(T, p)
    
    yaw = -math.atan2(2.0*(pose.orientation.x*pose.orientation.y + pose.orientation.w*pose.orientation.z), pose.orientation.w*pose.orientation.w + pose.orientation.x*pose.orientation.x - pose.orientation.y*pose.orientation.y - pose.orientation.z*pose.orientation.z)
    #q1 = tf.transformations.quaternion_from_euler(0, -math.pi/2+roll, yaw)
    #q2 = tf.transformations.quaternion_from_euler(0, 0, -math.pi/4)
    #quaternion = tf.transformations.quaternion_multiply(q2,q1)
    
    q1 = tf.transformations.quaternion_from_euler(0, -math.pi/2+roll, 0)
    q2 = tf.transformations.quaternion_from_euler(math.pi/4, 0, 0)
    q3 = tf.transformations.quaternion_from_euler(0, 0, yaw)
    quaternion = tf.transformations.quaternion_multiply(tf.transformations.quaternion_multiply(q3,q2),q1)
    
    marker = Marker ()
    marker.header.frame_id = "/map";
    marker.header.stamp = rospy.Time.now ()
    marker.type = Marker.ARROW
    marker.action = Marker.ADD
    marker.pose.position.x = t[0,0]
    marker.pose.position.y = t[1,0]
    marker.pose.position.z = t[2,0]
    marker.pose.orientation.x = quaternion[0]
    marker.pose.orientation.y = quaternion[1]
    marker.pose.orientation.z = quaternion[2]
    marker.pose.orientation.w = quaternion[3]
    marker.scale.x = racket_length
    marker.scale.y = 0.05
    marker.scale.z = 0.05
    marker.color.a = 1.0
    marker.color.r = 1.0
    marker.color.g = 0.0
    marker.color.b = 0.0
    markerPub.publish (marker)
    
if __name__ == '__main__':

    argv = rospy.myargv(sys.argv)
    rospy.init_node('arm_disp')
    
    pose = PoseStamped().pose
    rospy.Subscriber("/robot/pose", PoseStamped, poseCallback)
        
    slide = 0
    rospy.Subscriber("/mb1/enc1", std_msgs.msg.Float32, enc1Callback)
    roll = 0
    rospy.Subscriber("/mb1/enc2", std_msgs.msg.Float32, enc2Callback)
    swing = 0
    rospy.Subscriber("/mb1/enc3", std_msgs.msg.Float32, enc3Callback)
    
    markerPub = rospy.Publisher('/robot/arm', Marker, queue_size=1)
    
    rospy.Timer(rospy.Duration(0.04), time_callback)
    
    rospy.spin()
    
    
