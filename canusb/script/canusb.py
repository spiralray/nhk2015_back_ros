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
import rospkg

import serial
import time

# http://oxon.hatenablog.com/entry/20111108/1320680175
class MySerial(serial.Serial):
    """
    Wrapper for Serial
    """
    try:
        import io
    except ImportError:
        # serial.Serial inherits serial.FileLike
        pass
    else:
        def readline(self):
            """
            Overrides io.RawIOBase.readline which cannot handle with '\r' delimiters
            """
            ret = ''
            while True:
                c = self.read(1)
                if c == '':
                    return ret
                elif c == '\r':
                    return ret + c
                else:
                    ret += c

if __name__ == '__main__':
    ser = MySerial('/dev/ttyUSB0', 115200, timeout=0.01)
    ser.write("\r\r")
    line = ser.readline()
    line = ser.readline()
    
    ser.write("C\r")   # Close port
    ret = ord( ser.read() )
    
    while 1:
        line = ser.readline()
        if line == "":
            break
    
    #Check version
    ser.write("V\r")
    line = ser.readline()
    print "Version: " + line
    
    #Check version
    ser.write("N\r")
    line = ser.readline()
    print "Serial Number: " + line
    
    #Disable timestamp
    ser.write("Z0\r")   # Setup 500Kbit
    ret = ord( ser.read() )
    if ret == 13:
        print "Disable timestamp: Success"
    else:
        print "Disable timestamp: Fail"
    
    #Set baudrate
    ser.write("S6\r")   # Setup 500Kbit
    ret = ord( ser.read() )
    if ret == 13:
        print "Set baudrate as 500kbps Success"
    else:
        print "Set baudrate as 500kbps Fail"
    
    #Open port
    ser.write("O\r")   # Open port
    ret = ord( ser.read() )
    if ret == 13:
        print "Open port: Success"
    else:
        print "Open port: Fail"
    
    while True:
        
        try:
            line = ser.readline()
            if line != "":
                print line
        
        except KeyboardInterrupt:
            print ""
            print "Break"
            break
        
        
    #Close port
    ser.write("C\r")   # Close port
    ret = ord( ser.read() )
    if ret == 13:
        print "Close port: Success"
    else:
        print "Close port: Fail"
        
    
    
    
    
    ser.close()
