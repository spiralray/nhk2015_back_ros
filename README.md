ABU Robocon 2015 ROS programs
=================

[![](http://img.youtube.com/vi/TP8qb9XdHgs/0.jpg)](https://www.youtube.com/watch?v=TP8qb9XdHgs)

# Required devices
* Kinect V2 (To detect shuttles)
* UTM-30LX (To detect a pose of the robot using fences)
* LAWICEL CANUSB (To communicate with microcomputers)
* DUALSHOCK 3

# System requirements
* Ubuntu 14.04
* ROS Indigo
* Intel Core i7 (Quad core recommended)

# Required softwares

* [libfreenect2](https://github.com/OpenKinect/libfreenect2)
* [Sixaxis](https://help.ubuntu.com/community/Sixaxis)

# ROS nodes

This project contains following nodes.

* kinectv2 - Publish KinectV2 depth images
* shuttle_finder - Detect a shuttle from KinectV2 depth images
* shuttle_kalman - Estimate trajectory of a shuttle using Extended Kalman filter(EKF)
* laser2location - Detect a pose of the robot using LRF
* deadreckoning - Estimate a pose using two rotary encoders and a gyro sensor. This node also integrates poses published from laser2location using complementary filter.
* robominton - Calculate the best pose

![rosgraph](http://www.fortefibre.net/robots/2015/rosgraph.png)

# Log file
[DOWNLOAD](http://www.fortefibre.net/robots/2015/2015-05-24-12-22-43.bag)

[![](http://img.youtube.com/vi/iEYQ2zMrW6Y/0.jpg)](https://www.youtube.com/watch?v=iEYQ2zMrW6Y)

## How to play

```Bash
roslaunch robominton debug.launch
```

then

```Bash
rosbag play 2015-05-24-12-22-43.bag
```



# ロボットの動かし方

## PCの接続
グラナイトにG-Tune、Jr.にAlienwareを搭載する。
搭載後、各USBをPCに接続する。

- KinectとCANUSBはハブを介さず直接PCに接続する
- KinectはUSB3.0のコネクタにさす

## DualShock3の接続
ターミナルで

```Bash
sixad -start
```

として、コントローラのPSボタンを押す。

* BluetoothのドングルをUSBポートにあらかじめさしておくこと
* 赤いコントローラはグラナイト用、白いコントローラはJr.用

## ROSの起動
新しいターミナルで

```Bash
roscore
```

## プログラムの実行

1. センサーの電源(12V)を入れる
2. グラナイトのラケットを、中央上向きになるように手で合わせる(電源投入時、それがホームポジションとなる)
3. ロボットをフィールドにおいて、マイコンの電源を入れる
4. 以下のコマンドを使用し、プログラム実行

### グラナイトの場合

```Bash
roslaunch robominton robominton.launch
```

### Jr.の場合

```Bash
roslaunch robominton third.launch
```

コマンド実行時、赤いログが流れたら何らかのエラーが起きている。
Ctrl+Cでプログラムを停止し、エラー内容に従って各デバイスがコンピュータに正常に接続されていることを確認すること。

## プログラム実行後
緊急停止を解除し、駆動部に電源を供給する。 

* グラナイトのラケットが引き絞られ、自動で下向きに回転するはず。
* 引き絞られた後に下向きに回転しない時は、ラケット付近に付いているメイン基板のリセットスイッチを押す。

#LICENSE

Copyright (c) 2015 Kentaro Tanaka

http://spiralray.net/

Released under the MIT license