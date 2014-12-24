#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>

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
	void calcOmniWheel( float joydeg, float joypow );

	ros::NodeHandle mh;

	int linear_, angular_;
	double l_scale_, a_scale_;
	ros::Publisher swing_pub;
	ros::Subscriber joy_sub;

	ros::Publisher motor1_pub;
	ros::Publisher motor2_pub;
	ros::Publisher motor3_pub;

	float target_speed[3];

};


Machine::Machine()
{
	swing_pub = mh.advertise<std_msgs::Float32>("mb1/swing", 1);
	joy_sub = mh.subscribe<sensor_msgs::Joy>("joy", 10, &Machine::joyCallback, this);

	motor1_pub = mh.advertise<std_msgs::Float32>("/omni/motor1", 1);
	motor2_pub = mh.advertise<std_msgs::Float32>("/omni/motor2", 1);
	motor3_pub = mh.advertise<std_msgs::Float32>("/omni/motor3", 1);

	target_speed[0] = target_speed[1] = target_speed[2] = 0.0f;

}

void Machine::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{

	std_msgs::Float32 swing;

	if(joy->buttons[PS3_BUTTON_ACTION_SQUARE]) swing.data = 0.5f;
	else if(joy->buttons[PS3_BUTTON_ACTION_TRIANGLE]) swing.data = 1.0f;
	else  swing.data = 0.0f;

	swing_pub.publish(swing);

	float joyx = - joy->axes[PS3_AXIS_STICK_LEFT_LEFTWARDS];
	float joyy = joy->axes[PS3_AXIS_STICK_LEFT_UPWARDS];

	float joyspin = ((-joy->axes[PS3_AXIS_BUTTON_REAR_LEFT_1]) - (-joy->axes[PS3_AXIS_BUTTON_REAR_RIGHT_1]))/2;

	float joydeg = atan2(joyy, joyx);
	float joyslope = sqrt( pow(joyx, 2) + pow(joyy, 2));

	calcOmniWheel( joydeg, 2.5*joyslope );

	std_msgs::Float32 wheel1, wheel2, wheel3;
	wheel1.data = target_speed[0] - joyspin;
	wheel2.data = target_speed[1] - joyspin;
	wheel3.data = target_speed[2] - joyspin;

	motor1_pub.publish(wheel1);
	motor2_pub.publish(wheel2);
	motor3_pub.publish(wheel3);

	ROS_INFO("%.3f", joyspin);
	//ROS_INFO("%.3f %.3f", joydeg, joyslope);
	//ROS_INFO("%.3f %.3f %.3f", target_speed[0], target_speed[1], target_speed[2] );
}

void Machine::calcOmniWheel( float joydeg, float joyslope ){

    if(joyslope > 0){
        target_speed[0] = joyslope * -sin( joydeg - M_PI/6 );
        target_speed[1] = joyslope * -cos( joydeg );
        target_speed[2] = joyslope * -sin( joydeg - M_PI*5/6 );
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
