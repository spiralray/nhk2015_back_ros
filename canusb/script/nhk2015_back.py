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
import std_msgs.msg

import struct
import time
import threading

def callback(msg):
    if msg.data[0] == 0x310:
        data =  struct.unpack('f', ''.join(chr(i) for i in msg.data[1:]) )[0]
        omnienc1.publish( std_msgs.msg.Float32(data) )
    elif msg.data[0] == 0x311:
        data =  struct.unpack('f', ''.join(chr(i) for i in msg.data[1:]) )[0]
        omnienc2.publish( std_msgs.msg.Float32(data) )
    elif msg.data[0] == 0x312:
        data =  struct.unpack('f', ''.join(chr(i) for i in msg.data[1:]) )[0]
        omnienc3.publish( std_msgs.msg.Float32(data) )
    
    elif msg.data[0] == 0x320:
        data =  struct.unpack('f', ''.join(chr(i) for i in msg.data[1:]) )[0]
        mb1enc1.publish( std_msgs.msg.Float32(data) )
    elif msg.data[0] == 0x321:
        data =  struct.unpack('f', ''.join(chr(i) for i in msg.data[1:]) )[0]
        mb1enc2.publish( std_msgs.msg.Float32(data) )
    elif msg.data[0] == 0x322:
        data =  struct.unpack('f', ''.join(chr(i) for i in msg.data[1:]) )[0]
        mb1enc3.publish( std_msgs.msg.Float32(data) )
    
def omni1(msg):
    send = Int16MultiArray()
    send.data = [0x110]
    for char in struct.pack('f', msg.data):
        send.data.append( ord(char) ) 
    pub.publish( send )
    
def omni2(msg):
    send = Int16MultiArray()
    send.data = [0x111]
    for char in struct.pack('f', msg.data):
        send.data.append( ord(char) ) 
    pub.publish( send )
    
def omni3(msg):
    send = Int16MultiArray()
    send.data = [0x112]
    for char in struct.pack('f', msg.data):
        send.data.append( ord(char) ) 
    pub.publish( send )
    
def mb1motor1(msg):
    send = Int16MultiArray()
    send.data = [0x120]
    for char in struct.pack('f', msg.data):
        send.data.append( ord(char) ) 
    pub.publish( send )
    
def mb1motor2(msg):
    send = Int16MultiArray()
    send.data = [0x121]
    for char in struct.pack('f', msg.data):
        send.data.append( ord(char) ) 
    pub.publish( send )
    
def mb1swing(msg):
    send = Int16MultiArray()
    send.data = [0x122]
    for char in struct.pack('f', msg.data):
        send.data.append( ord(char) ) 
    pub.publish( send )
    
     
def listener():
    rospy.Subscriber("canrx", Int16MultiArray, callback)
    rospy.Subscriber("/omni/motor1", std_msgs.msg.Float32, omni1)
    rospy.Subscriber("/omni/motor2", std_msgs.msg.Float32, omni2)
    rospy.Subscriber("/omni/motor3", std_msgs.msg.Float32, omni3)
    
    rospy.Subscriber("/mb1/motor1", std_msgs.msg.Float32, mb1motor1)
    rospy.Subscriber("/mb1/motor2", std_msgs.msg.Float32, mb1motor2)
    rospy.Subscriber("/mb1/swing", std_msgs.msg.Float32, mb1swing)
    
    rospy.spin()
            
if __name__ == '__main__':
    argv = rospy.myargv(sys.argv)
    rospy.init_node('canMachine')
    
    pub = rospy.Publisher('cantx', Int16MultiArray, queue_size=100)
    
    omnienc1 = rospy.Publisher('/omni/enc1', std_msgs.msg.Float32, queue_size=1)
    omnienc2 = rospy.Publisher('/omni/enc2', std_msgs.msg.Float32, queue_size=1)
    omnienc3 = rospy.Publisher('/omni/enc3', std_msgs.msg.Float32, queue_size=1)
    
    mb1enc1 = rospy.Publisher('/mb1/enc1', std_msgs.msg.Float32, queue_size=1)
    mb1enc2 = rospy.Publisher('/mb1/enc2', std_msgs.msg.Float32, queue_size=1)
    mb1enc3 = rospy.Publisher('/mb1/enc3', std_msgs.msg.Float32, queue_size=1)
    
                
    listener()
    print ""
    print "close"