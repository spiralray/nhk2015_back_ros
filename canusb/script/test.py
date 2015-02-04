#!/usr/bin/env python
# -*- coding: utf-8 -*-

'''
Created on 2015/01/30

@author: spiralray
'''

import sys
import yaml
import roslib
roslib.load_manifest("canusb");

import struct
import rospy
from _CAN import CAN

import serial
import time
import threading

def talker():
    pub = rospy.Publisher('cantx', CAN, queue_size=100)
    rospy.init_node('canusb_test', anonymous=True)
    r = rospy.Rate(10) # 10Hz
    count = 0
    while not rospy.is_shutdown():
        msg = CAN()
        msg.stdId = 1
        msg.extId = -1
        msg.data = struct.pack('B', count%256) + 'ABCDE'
        pub.publish( msg )
        
        msg = CAN()
        msg.stdId = 100
        msg.extId = 25
        msg.data = struct.pack('i', count)
        pub.publish( msg )
        
        count += 1
        
        r.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException: pass
