#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Byte.h>
#include "ps3.h"

#ifndef M_PI
	#define M_PI 3.1415926535897932
#endif


class Machine
{
public:
	Machine();

private:
	void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
	void calcOmniWheel( float joyrad, float joypow );

	ros::NodeHandle mh;

	int linear_, angular_;
	double l_scale_, a_scale_;
	ros::Publisher swing_pub;
	ros::Publisher air_pub;

	ros::Subscriber joy_sub;

	ros::Publisher motor1_pub;
	ros::Publisher motor2_pub;
	ros::Publisher motor3_pub;

	ros::Publisher mode_pub;

	std_msgs::Int32 mode;

	float target_speed[3];

};


Machine::Machine()
{
	swing_pub = mh.advertise<std_msgs::Float32>("mb1/swing", 1);
	air_pub = mh.advertise<std_msgs::Byte>("/hand", 1);

	joy_sub = mh.subscribe<sensor_msgs::Joy>("joy", 10, &Machine::joyCallback, this);

	motor1_pub = mh.advertise<std_msgs::Float32>("/omni/motor1", 1);
	motor2_pub = mh.advertise<std_msgs::Float32>("/omni/motor2", 1);
	motor3_pub = mh.advertise<std_msgs::Float32>("/omni/motor3", 1);

	mode_pub = mh.advertise<std_msgs::Int32>("/robot/mode", 1);

	target_speed[0] = target_speed[1] = target_speed[2] = 0.0f;

}

void Machine::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{

	if( joy->buttons[PS3_BUTTON_SELECT] && mode.data != 1 ){
		mode.data = 1;
		mode_pub.publish(mode);

		std_msgs::Byte air;
		air.data = 0;
		air_pub.publish(air);
	}
	if( joy->buttons[PS3_BUTTON_START] && mode.data != 0 ){

		std_msgs::Float32 wheel1, wheel2, wheel3;
		wheel1.data = 0.0f;
		wheel2.data = 0.0f;
		wheel3.data = 0.0f;

		motor1_pub.publish(wheel1);
		motor2_pub.publish(wheel2);
		motor3_pub.publish(wheel3);

		ros::Duration(0.2).sleep();

		std_msgs::Byte air;
		air.data = 1;
		air_pub.publish(air);

		ros::Duration(0.18).sleep();

		std_msgs::Float32 swing;
		swing.data = 1.0f;
		swing_pub.publish(swing);

		ros::Duration(0.3).sleep();

		mode.data = 0;
		mode_pub.publish(mode);
	}
	else{

		std_msgs::Float32 swing;
		if(joy->buttons[PS3_BUTTON_ACTION_SQUARE]) swing.data = 0.5f;
		else if(joy->buttons[PS3_BUTTON_ACTION_TRIANGLE]) swing.data = 1.0f;
		else  swing.data = 0.0f;
		swing_pub.publish(swing);

		float joyx = - joy->axes[PS3_AXIS_STICK_LEFT_LEFTWARDS];
		float joyy = joy->axes[PS3_AXIS_STICK_LEFT_UPWARDS];

		float joyspin = ((-joy->axes[PS3_AXIS_BUTTON_REAR_LEFT_1]) - (-joy->axes[PS3_AXIS_BUTTON_REAR_RIGHT_1]))/2;

		float joyrad = atan2(joyy, joyx);
		float joyslope = sqrt( pow(joyx, 2) + pow(joyy, 2));

		float tmp;
		tmp = fabs( M_PI/2 - fabs(joyrad) );
		tmp = M_PI/4 - fabs( tmp - M_PI/4 );
		joyslope = joyslope/sqrt( 1 + pow( tan(tmp), 2) );

		//ROS_INFO("%.3f %.3f", joyrad, joyslope);

		calcOmniWheel( joyrad, 3.5*joyslope );

		std_msgs::Float32 wheel1, wheel2, wheel3;
		wheel1.data = target_speed[0] - joyspin;
		wheel2.data = target_speed[1] - joyspin;
		wheel3.data = target_speed[2] - joyspin;

		motor1_pub.publish(wheel1);
		motor2_pub.publish(wheel2);
		motor3_pub.publish(wheel3);

	}

	//ROS_INFO("%.3f", joyspin);
	//ROS_INFO("%.3f %.3f", joyrad, joyslope);
	//ROS_INFO("%.3f %.3f %.3f", target_speed[0], target_speed[1], target_speed[2] );
}

void Machine::calcOmniWheel( float joyrad, float joyslope ){

    if(joyslope > 0){
        target_speed[0] = joyslope * -sin( joyrad - M_PI/6 );
        target_speed[1] = joyslope * -cos( joyrad );
        target_speed[2] = joyslope * -sin( joyrad - M_PI*5/6 );
    }
    else{
    	target_speed[0] = target_speed[1] = target_speed[2] = 0.0f;
    }

}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "Machine");
	Machine machine;

	ros::spin();
}
