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
from _servo import servo

import serial
import struct
import time
import threading
import math

def callback(msg):
    kinect.publish( std_msgs.msg.Float32(msg.angle - 144*(0.000383495*6/5) ) )
    
if __name__ == '__main__':
    argv = rospy.myargv(sys.argv)
    rospy.init_node('servonhk')
    
    kinect = rospy.Publisher('/kinect/angle', std_msgs.msg.Float32, queue_size=1)
    rospy.Subscriber("/servo/rx", servo, callback)
    rospy.spin()
    