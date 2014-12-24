/*
 * ps3.h
 *
 *  Created on: 2014/12/23
 *      Author: fortefibre
 */

#ifndef PS3_H_
#define PS3_H_


// note on plain values:
// buttons are either 0 or 1
// button axes go from 0 to -1
// stick axes go from 0 to +/-1

#define PS3_BUTTON_SELECT            0
#define PS3_BUTTON_STICK_LEFT        1
#define PS3_BUTTON_STICK_RIGHT       2
#define PS3_BUTTON_START             3
#define PS3_BUTTON_CROSS_UP          4
#define PS3_BUTTON_CROSS_RIGHT       5
#define PS3_BUTTON_CROSS_DOWN        6
#define PS3_BUTTON_CROSS_LEFT        7
#define PS3_BUTTON_REAR_LEFT_2       8
#define PS3_BUTTON_REAR_RIGHT_2      9
#define PS3_BUTTON_REAR_LEFT_1       10
#define PS3_BUTTON_REAR_RIGHT_1      11
#define PS3_BUTTON_ACTION_TRIANGLE   12
#define PS3_BUTTON_ACTION_CIRCLE     13
#define PS3_BUTTON_ACTION_CROSS      14
#define PS3_BUTTON_ACTION_SQUARE     15
#define PS3_BUTTON_PAIRING           16

enum PS3_AXIS{
	PS3_AXIS_STICK_LEFT_LEFTWARDS = 0,
	PS3_AXIS_STICK_LEFT_UPWARDS,
	PS3_AXIS_STICK_RIGHT_LEFTWARDS,
	PS3_AXIS_STICK_RIGHT_UPWARDS,
	PS3_AXIS_ACCELEROMETER_LEFT,
	PS3_AXIS_ACCELEROMETER_FORWARD,
	PS3_AXIS_ACCELEROMETER_UP,
	PS3_AXIS_GYRO_YAW,
	PS3_AXIS_BUTTON_CROSS_UP,
	PS3_AXIS_BUTTON_CROSS_RIGHT,
	PS3_AXIS_BUTTON_CROSS_DOWN,
	PS3_AXIS_BUTTON_CROSS_LEFT,
	PS3_AXIS_BUTTON_REAR_LEFT_2,
	PS3_AXIS_BUTTON_REAR_RIGHT_2,
	PS3_AXIS_BUTTON_REAR_LEFT_1,
	PS3_AXIS_BUTTON_REAR_RIGHT_1,
	PS3_AXIS_BUTTON_ACTION_TRIANGLE,
	PS3_AXIS_BUTTON_ACTION_CIRCLE,
	PS3_AXIS_BUTTON_ACTION_CROSS,
	PS3_AXIS_BUTTON_ACTION_SQUARE
};





#endif /* PS3_H_ */
