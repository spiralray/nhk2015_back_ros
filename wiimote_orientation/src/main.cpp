#include "ros/ros.h"
#include <sensor_msgs/Imu.h>

#include "MadgwickAHRS.h"

ros::Publisher pub;
MadgwickAHRS ahrs;

sensor_msgs::Imu imu;

void imuCallback(const sensor_msgs::Imu::ConstPtr& msg)
{
	static bool isFirst = true;
	double duration = 0;

	if(!isFirst){
		duration = msg->header.stamp.toSec() - imu.header.stamp.toSec();
		ahrs.update( duration, msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z, msg->linear_acceleration.x/9.81, msg->linear_acceleration.y/9.81, msg->linear_acceleration.z/9.81 );
	}
	else{
		ahrs.update(0, msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z, msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z );
		isFirst = false;
	}
	imu = *msg;

	float q[4];
	ahrs.getQuaternion(q);

	imu.header.frame_id = "/wii";
	imu.orientation.w = q[0];
	imu.orientation.x = q[1];
	imu.orientation.y = q[2];
	imu.orientation.z = q[3];

	pub.publish(imu);

	float yaw, pitch, roll;
	ahrs.getYawPitchRoll(yaw, pitch, roll);

	ROS_INFO( "%.5f %.5f %.5f %.5f",duration, yaw*180/M_PI, pitch*180/M_PI, roll*180/M_PI  );

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "wii");
  ros::NodeHandle n;

  pub = n.advertise<sensor_msgs::Imu>("/wii", 1000);

  ros::Subscriber subscriber = n.subscribe("/imu/data", 2, imuCallback);

  ros::spin();

  return 0;
}
