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

class MyThread(threading.Thread):

    def run(self):
    	while flag_shutdown == 0:
        	if s.readable():
        	 	sentence = s.readline().strip().split(",")
        	  	if sentence[0] != '':
        	  		print sentence
        	  		msg = PointStamped()
        	  	 	msg.header.stamp = rospy.Time.from_sec( float(sentence[0]) )
                    msg.header.frame_id = "map"
        	  	 	msg.point.x = float(sentence[1])
        	  	 	msg.point.y = float(sentence[2])
        	  	 	msg.point.z = float(sentence[3])
        	  	 	pub.publish(msg)
        	  	 	
         	else:
        	  	time.sleep(0.001)
        return
    
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
    
    pub = rospy.Publisher('/shuttle/point2', PointStamped, queue_size=1)
    thread = MyThread()
    
    flag_shutdown = 0
    thread.start()
    rospy.spin()
    flag_shutdown = 1
    
    thread.join(0.1)
    s.close()