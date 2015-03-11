# canusb
ROS node of LAWICEL CANUSB

##Install
clone this project into ~/catkin_ws/src

    cd ~/catkin_ws/src
    git clone https://github.com/spiralray/canusb.git
    cd ..
    catkin_make

##Usage

    rosparam set /canusb/baud 500k #Set 500Kbps
    rosparam set /canusb/port /dev/ttyUSB0 #Set CANUSB port
    rosrun canusb canusb.py

This node publish received messages on "/canrx". 

    rostopic echo /canrx

#rosmsg
This node uses original message formtat "CAN"  
This format contains  

* time timestamp
* uint16 stdId
* int32 extId
* uint8[] data

When you transmit messages in standard format, you have to set minus value to extId

##Supported baudrate
* 10k
* 20k
* 50k
* 100k
* 125k
* 250k
* 500k
* 800k
* 1m

You can choose baudrate using rosparam.

    rosparam set /canusb/baud  500k

Sample of transmitting CAN messages

    rosrun canusb test.py

