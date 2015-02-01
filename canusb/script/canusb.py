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
                    if ret == 'z':
                        self.status = 0
                    return ret + c
                elif c == chr(7):
                    self.status = -1
                    return c
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
            self.status = 0
        
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
            if can.read() == chr(13):
                return 0
            else:
                return -1
        
        def setBaud(self, baud):
            can.write(baud+"\r")
            if can.read() == chr(13):
                return 0
            else:
                return -1
            
        def start(self):
            can.write("O\r")   # Open port
            if can.read() == chr(13):
                return 0
            else:
                return -1
        
        def close(self):
            #Close port
            self.write("C\r")   # Close port
            if can.read() == chr(13):
                print "Close port: Success"
            else:
                print "Close port: Fail"
            serial.Serial.close(self)   #Call method of super class
        
        def send(self, stdid, dlc, data):
            if self.status == 1:
                return 1
            self.d = ''
            for i in range(0, dlc):
                self.d += "{0:02d}".format(data[i])
            self.status = 1
            self.write("t{0:03X}{1:d}{2:s}\r".format(stdid,dlc,self.d))
            #print "t{0:03X}{1:02d}{2:s}\r".format(stdid,dlc,self.d)
            return 0
        
        def analyze(self, str):
            command = str[:1]
            if command == 't':
                self.data = [ int(str[1:4],16), int(str[4:5],16) ]
                for i in range(0, self.data[1]):
                    self.data.append( int(str[5+i*2:7+i*2],16) )
                return self.data
            else:
                return 0
            

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
            #if line == "z\r" or line == "Z\r":
            #    print "Transmit Success"
            if line == chr(7):
                print "Transmit Failed"
            elif line != "" and line != "z\r" and line != "Z\r":
                print line
                print can.analyze(line)
                '''
                res = can.send(0x321, 5, [1, 2, 3, 4, 5] )
                if res  == 1:
                    print "Device busy"
                elif res != 0:
                    print "Transmit Failed"
                '''
                    
        
        except KeyboardInterrupt:
            print ""
            print "Break"
            break
    
    can.close()
