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
import std_msgs.msg
from _CAN import CAN

import struct
import time
import threading
from math import pi

def callback(msg):
    if msg.stdId == 0x310:
        data =  struct.unpack('f', msg.data )[0]
        omnienc1.publish( std_msgs.msg.Float32(data) )
    elif msg.stdId == 0x311:
        data =  struct.unpack('f', msg.data )[0]
        omnienc2.publish( std_msgs.msg.Float32(data) )
    elif msg.stdId == 0x312:
        data =  struct.unpack('f', msg.data )[0]
        omnienc3.publish( std_msgs.msg.Float32(data) )
    
    elif msg.stdId == 0x320:
        data =  struct.unpack('f', msg.data )[0]
        mb1enc1.publish( std_msgs.msg.Float32(data) )
    elif msg.stdId == 0x321:
        data =  struct.unpack('f', msg.data )[0]
        mb1enc2.publish( std_msgs.msg.Float32(data) )
    elif msg.stdId == 0x322:
        data =  struct.unpack('f', msg.data )[0]
        mb1enc3.publish( std_msgs.msg.Float32(data) )
    
    elif  msg.stdId == 0x240:
        data =  struct.unpack('f', msg.data )[0]
        mb2enc.publish( std_msgs.msg.Float32(data) )
    
    #Encoders for dead reckoning
    elif msg.stdId == 0x330:
        data =  struct.unpack('ii', msg.data )
        encX.publish( std_msgs.msg.Int32(data[0]) )
        encY.publish( std_msgs.msg.Int32(data[1]) )
        
    #Data of R1350N
    elif msg.stdId == 0x220:
        data =  struct.unpack('h', msg.data )[0]
        r1350.publish( std_msgs.msg.Float32(data/100 * pi /180 ) )
    
def omni1(msg):
    send = CAN()
    send.stdId = 0x110
    send.extId = -1
    send.data = struct.pack('f', msg.data)
    pub.publish( send )
    
def omni2(msg):
    send = CAN()
    send.stdId = 0x111
    send.extId = -1
    send.data = struct.pack('f', msg.data)
    pub.publish( send )
    
def omni3(msg):
    send = CAN()
    send.stdId = 0x112
    send.extId = -1
    send.data = struct.pack('f', msg.data)
    pub.publish( send )
    
def mb1motor1(msg):
    send = CAN()
    send.stdId = 0x120
    send.extId = -1
    send.data = struct.pack('f', msg.data)
    pub.publish( send )
    
def mb1motor2(msg):
    send = CAN()
    send.stdId = 0x121
    send.extId = -1
    send.data = struct.pack('f', msg.data)
    pub.publish( send )
    
def mb1swing(msg):
    if mode !=2:
        send = CAN()
        send.stdId = 0x122
        send.extId = -1
        send.data = struct.pack('f', msg.data)
        pub.publish( send )
    
def mb2motor(msg):
    send = CAN()
    send.stdId = 0x140
    send.extId = -1
    send.data = struct.pack('f', msg.data)
    pub.publish( send )
    
def handCallback(msg):
    send = CAN()
    send.stdId = 0x150
    send.extId = -1
    if msg.data:
        send.data = [0xff]
    else:
        send.data = [0x00]
    pub.publish( send )
    
def modeCallback(msg):
    global mode
    mode = msg.data
    
def autoSwingCallback(msg):
    if mode ==2:
        send = CAN()
        send.stdId = 0x122
        send.extId = -1
        send.data = struct.pack('f', msg.data)
        pub.publish( send )
        
if __name__ == '__main__':
    argv = rospy.myargv(sys.argv)
    rospy.init_node('canMachine')
    
    pub = rospy.Publisher('cantx', CAN, queue_size=100)
    
    omnienc1 = rospy.Publisher('/omni/enc1', std_msgs.msg.Float32, queue_size=1)
    omnienc2 = rospy.Publisher('/omni/enc2', std_msgs.msg.Float32, queue_size=1)
    omnienc3 = rospy.Publisher('/omni/enc3', std_msgs.msg.Float32, queue_size=1)
    
    mb1enc1 = rospy.Publisher('/mb1/enc1', std_msgs.msg.Float32, queue_size=1)
    mb1enc2 = rospy.Publisher('/mb1/enc2', std_msgs.msg.Float32, queue_size=1)
    mb1enc3 = rospy.Publisher('/mb1/enc3', std_msgs.msg.Float32, queue_size=1)
    
    mb2enc = rospy.Publisher('/mb2/enc', std_msgs.msg.Float32, queue_size=1)
    
    encX = rospy.Publisher('/encX', std_msgs.msg.Int32, queue_size=1)
    encY = rospy.Publisher('/encY', std_msgs.msg.Int32, queue_size=1)
    r1350 = rospy.Publisher('/yaw', std_msgs.msg.Float32, queue_size=1)
    
                
    rospy.Subscriber("canrx", CAN, callback)
    rospy.Subscriber("/omni/motor1", std_msgs.msg.Float32, omni1)
    rospy.Subscriber("/omni/motor2", std_msgs.msg.Float32, omni2)
    rospy.Subscriber("/omni/motor3", std_msgs.msg.Float32, omni3)
    
    rospy.Subscriber("/mb1/motor1", std_msgs.msg.Float32, mb1motor1)
    rospy.Subscriber("/mb1/motor2", std_msgs.msg.Float32, mb1motor2)
    rospy.Subscriber("/mb1/swing", std_msgs.msg.Float32, mb1swing)
    
    rospy.Subscriber("/mb2/motor", std_msgs.msg.Float32, mb2motor)
    
    rospy.Subscriber("/hand", std_msgs.msg.Byte, handCallback)
    
    mode = 0
    rospy.Subscriber("/robot/mode", std_msgs.msg.Int32, modeCallback)
    rospy.Subscriber("/auto/swing", std_msgs.msg.Float32, autoSwingCallback)
    
    rospy.spin()
    
    print ""
    print "close"