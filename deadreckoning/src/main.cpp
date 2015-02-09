#include "ros/ros.h"
#include "std_msgs/Int32.h"
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PoseStamped.h>

#ifndef M_PI
	#define M_PI 3.1415926535897932
#endif

#define ENCODER_RESOLUTION 500.0f	// [P/R]
#define OMNIWHEEL_DIAMETER 0.040f	// [m]

#define DISTANCE_LRF_TO_CENTER 0.455	//[m]

#define YAW_RATIO		0.030
#define LOCATION_RATIO	0.025

ros::Publisher pub;
geometry_msgs::PoseStamped pose_msg;

float yaw =0.0f;
float yawfirst = 0.0f;
int old_x = 0;
int old_y = 0;

bool laserReady = false;

void encXCallback(const std_msgs::Int32::ConstPtr& msg)
{
	static bool isFirst = true;

	if(!laserReady) return;

	//ROS_INFO("X: [%d]", msg->data);
	if(!isFirst){
		pose_msg.pose.position.x += (msg->data-old_x)/(ENCODER_RESOLUTION*4)*(M_PI*OMNIWHEEL_DIAMETER) * cos(yaw);
		pose_msg.pose.position.y -= (msg->data-old_x)/(ENCODER_RESOLUTION*4)*(M_PI*OMNIWHEEL_DIAMETER) * sin(yaw);
	}
	else{
		isFirst = false;
	}
	old_x = msg->data;
	pub.publish(pose_msg);
}
void encYCallback(const std_msgs::Int32::ConstPtr& msg)
{
	static bool isFirst = true;

	if(!laserReady) return;

	//ROS_INFO("Y: [%d]", msg->data);
	if(!isFirst){
		pose_msg.pose.position.x += (msg->data-old_y)/(ENCODER_RESOLUTION*4)*(M_PI*OMNIWHEEL_DIAMETER) * cos(M_PI/2 + yaw);
		pose_msg.pose.position.y -= (msg->data-old_y)/(ENCODER_RESOLUTION*4)*(M_PI*OMNIWHEEL_DIAMETER) * sin(M_PI/2 + yaw);
	}
	else{
		isFirst = false;
	}
	old_y = msg->data;
	pub.publish(pose_msg);
	ROS_INFO("[%f][%f][%f]", yaw*180/M_PI, pose_msg.pose.position.x, pose_msg.pose.position.y);
}

void imuCallback(const sensor_msgs::Imu::ConstPtr& msg)
{
	static bool isFirst = true;

	if(!laserReady) return;

	//ROS_INFO("IMU: [%f][%f][%f][%f]", msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z );
	if(!isFirst){
		//float roll = atan2(2.0*(msg->orientation.y*msg->orientation.z + msg->orientation.w*msg->orientation.x), msg->orientation.w*msg->orientation.w - msg->orientation.x*msg->orientation.x - msg->orientation.y*msg->orientation.y + msg->orientation.z*msg->orientation.z);
		//float pitch = asin(-2.0*(msg->orientation.x*msg->orientation.z - msg->orientation.w*msg->orientation.y));
		yaw = -atan2(2.0*(msg->orientation.x*msg->orientation.y + msg->orientation.w*msg->orientation.z), msg->orientation.w*msg->orientation.w + msg->orientation.x*msg->orientation.x - msg->orientation.y*msg->orientation.y - msg->orientation.z*msg->orientation.z)-yawfirst;

		float cr2 = cos(0*0.5);
		float cp2 = cos(0*0.5);
		float cy2 = cos(yaw*0.5);
		float sr2 = sin(0*0.5);
		float sp2 = sin(0*0.5);
		float sy2 = sin(yaw*0.5);

		pose_msg.pose.orientation.w = cr2*cp2*cy2 + sr2*sp2*sy2;
		pose_msg.pose.orientation.x = sr2*cp2*cy2 - cr2*sp2*sy2;
		pose_msg.pose.orientation.y = cr2*sp2*cy2 + sr2*cp2*sy2;
		pose_msg.pose.orientation.z = cr2*cp2*sy2 - sr2*sp2*cy2;

		pub.publish(pose_msg);
		ROS_INFO("[%f][%f][%f]", yaw*180/M_PI, pose_msg.pose.position.x, pose_msg.pose.position.y);
	}
	//ROS_INFO("yaw pitch roll: [%f][%f][%f]", yaw,pitch,roll);
	else{
		isFirst = false;
		yawfirst = -atan2(2.0*(msg->orientation.x*msg->orientation.y + msg->orientation.w*msg->orientation.z), msg->orientation.w*msg->orientation.w + msg->orientation.x*msg->orientation.x - msg->orientation.y*msg->orientation.y - msg->orientation.z*msg->orientation.z);
	}
}

void laserCallback(const geometry_msgs::PoseStamped::ConstPtr& msg){
	//ROS_INFO("IMU: [%f][%f][%f][%f]", msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z );
	if(laserReady){
		float tmpyaw = -atan2(2.0*(msg->pose.orientation.x*msg->pose.orientation.y + msg->pose.orientation.w*msg->pose.orientation.z), msg->pose.orientation.w*msg->pose.orientation.w + msg->pose.orientation.x*msg->pose.orientation.x - msg->pose.orientation.y*msg->pose.orientation.y - msg->pose.orientation.z*msg->pose.orientation.z)-yawfirst;
		float tmpx = msg->pose.position.x;
		float tmpy = msg->pose.position.y;

		while(  yaw-tmpyaw > M_PI )		tmpyaw += M_PI;
		while(  yaw-tmpyaw < -M_PI )	tmpyaw -= M_PI;

		yaw = yaw*(1.0f-YAW_RATIO) + 0.01 * tmpyaw;
		pose_msg.pose.position.x = pose_msg.pose.position.x*(1.0f-LOCATION_RATIO)+tmpx*LOCATION_RATIO;
		pose_msg.pose.position.y = pose_msg.pose.position.y*(1.0f-LOCATION_RATIO)+tmpy*LOCATION_RATIO;


		pub.publish(pose_msg);
		ROS_INFO("[%f][%f][%f]", yaw*180/M_PI, msg->pose.position.x, msg->pose.position.y);
	}
	//ROS_INFO("yaw pitch roll: [%f][%f][%f]", yaw,pitch,roll);
	else{
		yaw = -atan2(2.0*(msg->pose.orientation.x*msg->pose.orientation.y + msg->pose.orientation.w*msg->pose.orientation.z), msg->pose.orientation.w*msg->pose.orientation.w + msg->pose.orientation.x*msg->pose.orientation.x - msg->pose.orientation.y*msg->pose.orientation.y - msg->pose.orientation.z*msg->pose.orientation.z)-yawfirst;
		pose_msg.pose.position.x = msg->pose.position.x;
		pose_msg.pose.position.y = msg->pose.position.y;
		laserReady = true;
	}
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "deadreckoning");

	ros::NodeHandle n;

	ros::Subscriber subX = n.subscribe("encX", 1000, encXCallback);
	ros::Subscriber subY = n.subscribe("encY", 1000, encYCallback);
	ros::Subscriber subIMU = n.subscribe("imu", 1000, imuCallback);
	ros::Subscriber subPose = n.subscribe("lrf_pose", 1000, laserCallback);

	pose_msg.header.frame_id = "/map";
	pose_msg.pose.position.x = pose_msg.pose.position.y = pose_msg.pose.position.z = 0.0f;
	pose_msg.pose.orientation.x = pose_msg.pose.orientation.y = pose_msg.pose.orientation.z = pose_msg.pose.orientation.w = 0.0f;

	pub = n.advertise<geometry_msgs::PoseStamped>("/robot/pose", 5);
	ros::spin();

	return 0;
}
