#!/usr/bin/env python
# -*- coding: utf-8 -*-

'''
Created on 2015/02/26

@author: spiralray
'''

import sys
import yaml
import roslib
roslib.load_manifest("kondo");

import rospy
import std_msgs.msg
from kondo.msg import servo

import serial
import struct
import time
import threading
import math

def callback(msg):
    kinect.publish( std_msgs.msg.Float32(-msg.angle ) )

def highCallback(msg):
    global high
    high = msg.data

def targetCallback(msg):
    pubmsg = servo()
    pubmsg.id = 1
    pubmsg.angle = -msg.data
    s.publish( pubmsg )

def timer_callback(msg):
    pubmsg = servo()
    pubmsg.id = 1
    if high == True:
        pubmsg.angle = -rospy.get_param('~angle')-0.4
    else:
        pubmsg.angle = -rospy.get_param('~angle')
    s.publish( pubmsg )

if __name__ == '__main__':
    argv = rospy.myargv(sys.argv)
    rospy.init_node('servonhk')

    kinect = rospy.Publisher('/kinect/angle', std_msgs.msg.Float32, queue_size=1)
    #rospy.Subscriber("/kinect/targetangle", std_msgs.msg.Float32, targetCallback)
    rospy.Subscriber("/servo/rx", servo, callback)

    high=False
    rospy.Subscriber("/kinect/high", std_msgs.msg.Bool, highCallback)
    s = rospy.Publisher('/servo/tx', servo, queue_size=1)

    rospy.Timer(rospy.Duration(0.03), timer_callback)
    rospy.spin()
