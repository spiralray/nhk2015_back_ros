#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Int32.h>

#include "ps3.h"

class Machine
{
public:
	Machine();

private:
	void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);

	ros::NodeHandle mh;

	int linear_, angular_;
	double l_scale_, a_scale_;
	ros::Publisher swing_pub;
	ros::Subscriber joy_sub;

};


Machine::Machine()
{
	swing_pub = mh.advertise<std_msgs::Int32>("mb1/swing", 1);
	joy_sub = mh.subscribe<sensor_msgs::Joy>("joy", 10, &Machine::joyCallback, this);

}

void Machine::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{

	std_msgs::Int32 swing;

	swing.data = joy->buttons[PS3_BUTTON_ACTION_SQUARE];

	swing_pub.publish(swing);

	ROS_INFO("%d", swing);
}


int main(int argc, char** argv)
{
	ros::init(argc, argv, "Machine");
	Machine machine;

	ros::spin();
}
