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
class CanUSB(serial.Serial):
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
                    
        def init(self):
            self.write("\r\r")
            while 1:
                line = self.readline()
                if line == "":
                    break
            self.write("C\r")   # Close port
            ret = ord( self.read() )
        
        def version(self):  #Check version
            can.write("V\r")
            return can.readline()
        
        def serial(self):
            can.write("N\r")
            return can.readline()
        
        def setTimestamp(self, enable=0):
            if enable == True:
                can.write("Z1\r")
            else:
                can.write("Z0\r")
            ret = ord( can.read() )
            if ret == 13:
                return 0
            else:
                return -1
        
        def setBaud(self, baud):
            can.write(baud+"\r")
            ret = ord( can.read() )
            if ret == 13:
                return 0
            else:
                return -1
            
        def start(self):
            can.write("O\r")   # Open port
            ret = ord( can.read() )
            if ret == 13:
                return 0
            else:
                return -1
        
        def close(self):
            #Close port
            self.write("C\r")   # Close port
            ret = ord( self.read() )
            if ret == 13:
                print "Close port: Success"
            else:
                print "Close port: Fail"
            serial.Serial.close(self)   #Call method of super class

if __name__ == '__main__':
    can = CanUSB('/dev/ttyUSB0', 115200, timeout=0.01)
    
    can.init()
    print "Version: " + can.version()
    print "Serial Number: " + can.serial()
    
    #Disable timestamp
    if can.setTimestamp(False) != 0:
        print "Disable timestamp: Failed!"
    
    #Set baud rate to 500kbps
    if can.setBaud("S6") != 0:
        print "Disable baudrate: Failed!"
    
    #Open port
    if can.start() != 0:
        print "Open port: Fail"
    
    while True:
        
        try:
            line = can.readline()
            if line != "":
                print line
        
        except KeyboardInterrupt:
            print ""
            print "Break"
            break
    
    can.close()
