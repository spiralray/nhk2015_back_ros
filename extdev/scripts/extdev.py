#!/usr/bin/env python
# -*- coding: utf-8 -*-

'''
Created on 2015/02/26

@author: spiralray
'''

import sys
import yaml
import roslib
roslib.load_manifest("extdev");

import rospy
from geometry_msgs.msg import Point
from geometry_msgs.msg import PointStamped

import serial
import struct
import time
import threading
import math

def callback(msg):
	s.write(str(msg.header.stamp.to_sec())+','+str(msg.point.x)+','+str(msg.point.y)+','+str(msg.point.z)+'\n')
    
if __name__ == '__main__':
    argv = rospy.myargv(sys.argv)
    rospy.init_node('extdev')
    
    try:
        port = rospy.get_param('~port')
        rospy.loginfo('Parameter %s has value %s', rospy.resolve_name('~port'), port)
    except:
        rospy.logerr("Set correct port to %s", rospy.resolve_name('~port'))
        exit()
    
    s = serial.Serial(port, 115200, timeout=0.1)
    
    rospy.Subscriber("/shuttle/point", PointStamped, callback)
    rospy.spin()
    
    s.close()