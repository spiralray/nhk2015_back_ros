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

import rospy
from std_msgs.msg import Int16MultiArray

import serial
import time
import threading

def talker():
    pub = rospy.Publisher('cantx', Int16MultiArray, queue_size=100)
    rospy.init_node('canusb_test', anonymous=True)
    r = rospy.Rate(10) # 10Hz
    count = 0
    while not rospy.is_shutdown():
        msg = Int16MultiArray()
        msg.data = [0x321,1,10,count]
        count = (count + 1)%256
        pub.publish( msg )
        r.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException: pass
