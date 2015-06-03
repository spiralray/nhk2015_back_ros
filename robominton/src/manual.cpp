#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Byte.h>
#include <std_msgs/Bool.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>

#include <vector>
#include <algorithm>    // std::copy
#include <iterator>     // std::back_inserter

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
	void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
	void timerCallback(const ros::TimerEvent& event);
	void calcOmniWheel( double sec, float rad, float speed );
	void enc1Callback(const std_msgs::Float32::ConstPtr& msg);
	void enc2Callback(const std_msgs::Float32::ConstPtr& msg);
	void enc3Callback(const std_msgs::Float32::ConstPtr& msg);
	void targetpointCallback(const geometry_msgs::PointStamped::ConstPtr& msg);

	ros::NodeHandle nh;

	int linear_, angular_;
	double l_scale_, a_scale_;
	ros::Publisher swing_pub;
	ros::Publisher air_pub;

	ros::Subscriber joy_sub;
	ros::Subscriber pose_sub;
	ros::Timer timer;
	ros::Subscriber enc1_sub, enc2_sub, enc3_sub;

	ros::Publisher motor1_pub;
	ros::Publisher motor2_pub;
	ros::Publisher motor3_pub;

	ros::Publisher kinectHigh_pub;

	ros::Publisher mode_pub;

	ros::Subscriber targetPoint_sub;

	std_msgs::Int32 mode;

	pthread_mutex_t	pose_mutex;  // MUTEX
	geometry_msgs::Pose pose;

	sensor_msgs::Joy joy;

	float target_speed[3];
	float encoder[3];

	bool target_recieved;
	double target_time;
	float target_x, target_y;
	float target_yaw;

	float dt;
	float MAX_ACCEL;

	bool joy_recieved;

};


Machine::Machine()
{
	MAX_ACCEL = 8.0;
	dt = 0.05;

	joy_recieved = false;
	target_recieved = false;

	encoder[0] = encoder[1] = encoder[2] = 0.0f;

	swing_pub = nh.advertise<std_msgs::Float32>("/mb1/swing", 1);
	air_pub = nh.advertise<std_msgs::Byte>("/hand", 1);

	joy_sub = nh.subscribe<sensor_msgs::Joy>("joy", 10, &Machine::joyCallback, this);
	pose_sub = nh.subscribe("/robot/pose", 10, &Machine::poseCallback, this);
	timer = nh.createTimer(ros::Duration(this->dt), &Machine::timerCallback, this);

	enc1_sub = nh.subscribe("/omni/enc1", 1, &Machine::enc1Callback, this);
	enc2_sub = nh.subscribe("/omni/enc2", 1, &Machine::enc2Callback, this);
	enc3_sub = nh.subscribe("/omni/enc3", 1, &Machine::enc3Callback, this);


	motor1_pub = nh.advertise<std_msgs::Float32>("/omni/motor1", 1);
	motor2_pub = nh.advertise<std_msgs::Float32>("/omni/motor2", 1);
	motor3_pub = nh.advertise<std_msgs::Float32>("/omni/motor3", 1);

	kinectHigh_pub = nh.advertise<std_msgs::Bool>("/kinect/high", 1);

	mode_pub = nh.advertise<std_msgs::Int32>("/robot/mode", 1);

	targetPoint_sub = nh.subscribe("/robot/targetpoint", 1, &Machine::targetpointCallback, this);

	target_speed[0] = target_speed[1] = target_speed[2] = 0.0f;
	target_yaw = 0.0f;

	target_time = 0;

}

void Machine::poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg){
	//ROS_INFO("poseCallback");
	//pthread_mutex_lock( &pose_mutex );
	pose = msg->pose;
	//pthread_mutex_unlock( &pose_mutex );
}


void Machine::joyCallback(const sensor_msgs::Joy::ConstPtr& j)
{
	joy_recieved = true;

	joy.buttons.clear();
	joy.axes.clear();
	copy(j->buttons.begin(), j->buttons.end(), back_inserter(joy.buttons) );
	copy(j->axes.begin(), j->axes.end(), back_inserter(joy.axes) );
}

void Machine::enc1Callback(const std_msgs::Float32::ConstPtr& msg){
	encoder[0] = msg->data;
}
void Machine::enc2Callback(const std_msgs::Float32::ConstPtr& msg){
	encoder[1] = msg->data;
}
void Machine::enc3Callback(const std_msgs::Float32::ConstPtr& msg){
	encoder[2] = msg->data;
}

void Machine::targetpointCallback(const geometry_msgs::PointStamped::ConstPtr& msg){
	target_recieved = true;
	target_time = msg->header.stamp.toSec();
	target_x = msg->point.x;
	target_y = msg->point.y;
}

void Machine::timerCallback(const ros::TimerEvent& event){

	if( !joy_recieved ) return;

	if( joy.buttons[PS3_BUTTON_REAR_RIGHT_1] ){
		std_msgs::Bool high;
		high.data = true;
		kinectHigh_pub.publish(high);
	}
	else{
		std_msgs::Bool high;
		high.data = false;
		kinectHigh_pub.publish(high);
	}

	if( joy.buttons[PS3_BUTTON_SELECT] && mode.data != 1 ){
		mode.data = 1;
		mode_pub.publish(mode);

		std_msgs::Byte air;
		air.data = 0;
		air_pub.publish(air);
	}
	if( (joy.buttons[PS3_BUTTON_START] || joy.buttons[PS3_BUTTON_PAIRING] ) && mode.data == 1 ){

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

		if( joy.buttons[PS3_BUTTON_PAIRING] )	ros::Duration(0.228).sleep();
		else									ros::Duration(0.18).sleep();


		mode.data = 0;
		mode_pub.publish(mode);

		std_msgs::Float32 swing;
		if( joy.buttons[PS3_BUTTON_PAIRING] )	swing.data = 0.85f;
		else									swing.data = 0.9f;
		swing_pub.publish(swing);

		ros::Duration(0.3).sleep();
	}
	else{

		if( mode.data != 1 ){
			if( joy.buttons[PS3_BUTTON_ACTION_CROSS] ){
				mode.data = 2;
				mode_pub.publish(mode);
			}
			else if(mode.data == 2){
				mode.data = 0;
				mode_pub.publish(mode);
			}
		}
		std_msgs::Float32 swing;
		if(joy.buttons[PS3_BUTTON_ACTION_SQUARE]) swing.data = 0.5f;
		else if(joy.buttons[PS3_BUTTON_ACTION_TRIANGLE]) swing.data = 1.0f;
		else  swing.data = 0.0f;
		swing_pub.publish(swing);

		float joyx = - joy.axes[PS3_AXIS_STICK_LEFT_LEFTWARDS];
		float joyy = joy.axes[PS3_AXIS_STICK_LEFT_UPWARDS];

		float yaw = -atan2(2.0*(pose.orientation.x*pose.orientation.y + pose.orientation.w*pose.orientation.z), pose.orientation.w*pose.orientation.w + pose.orientation.x*pose.orientation.x - pose.orientation.y*pose.orientation.y - pose.orientation.z*pose.orientation.z);
		if( std::abs(yaw) > M_PI/4 ){
			std_msgs::Float32 wheel1, wheel2, wheel3;
			wheel1.data = 0;
			wheel2.data = 0;
			wheel3.data = 0;
			motor1_pub.publish(wheel1);
			motor2_pub.publish(wheel2);
			motor3_pub.publish(wheel3);
			ROS_ERROR("Out of control");
			return;
		}

#define MAX_SPIN 0.3

		float spin;

		if( joy.buttons[PS3_BUTTON_REAR_LEFT_1] || mode.data == 1 ){
			//If service or serve-recieve
			target_yaw = M_PI/15;
		}
		else{
			target_yaw = 0.0f;
		}

		spin = (yaw+target_yaw)*2.0;
		if( spin > MAX_SPIN ) spin = MAX_SPIN;
		else if( spin < -MAX_SPIN ) spin = -MAX_SPIN;
		else if( spin < 0.005) spin = 0;

		float joyrad = atan2(joyy, joyx);
		float joyslope = sqrt( pow(joyx, 2) + pow(joyy, 2));

		double time_lasts = target_time - ros::Time::now().toSec();

		if( target_recieved && joy.buttons[PS3_BUTTON_ACTION_CIRCLE] && time_lasts > -0.2 ){	//Automatic Mode
			float diff_x = target_x - pose.position.x;
			float diff_y = target_y - pose.position.y;

			float target_deg = std::atan2(diff_y,diff_x) - yaw;
			float target_speed = std::sqrt( std::pow(diff_x, 2) + std::pow(diff_y, 2) ) * 2;
			if( target_speed > 3 ) target_speed = 3;
			//else if( target_speed > 0.05 && target_speed < 0.2 ) target_speed = 0.2;

			if( time_lasts > 0){
				float min_speed = sqrt( pow(diff_x,2) + pow(diff_y,2) ) / time_lasts;
				if( min_speed > target_speed ) target_speed = min_speed;
			}

			//ROS_INFO("%.3f %.3f", target_deg, target_speed);
			calcOmniWheel( event.current_real.toSec() - event.last_real.toSec() , target_deg, target_speed );
		}
		else{	//Manual Mode
			float tmp;
			tmp = fabs( M_PI/2 - fabs(joyrad) );
			tmp = M_PI/4 - fabs( tmp - M_PI/4 );
			joyslope = joyslope/sqrt( 1 + pow( tan(tmp), 2) );
			//ROS_INFO("%.3f %.3f", joyrad, joyslope);
			calcOmniWheel( event.current_real.toSec() - event.last_real.toSec() , joyrad, 2.5*joyslope );
		}

		std_msgs::Float32 wheel1, wheel2, wheel3;
		wheel1.data = target_speed[0] - spin;
		wheel2.data = target_speed[1] - spin;
		wheel3.data = target_speed[2] - spin;

		motor1_pub.publish(wheel1);
		motor2_pub.publish(wheel2);
		motor3_pub.publish(wheel3);

		//ROS_INFO("%.3f", spin);
		//ROS_INFO("%.3f %.3f", joyrad, joyslope);
	}


	//ROS_INFO("%.3f %.3f %.3f", target_speed[0], target_speed[1], target_speed[2] );
}

void Machine::calcOmniWheel( double sec, float rad, float speed ){

	float speed_u= 0.0, speed_v=  0.0;
	float t[3];

	float acc = MAX_ACCEL*sec;

	//ROS_INFO("%lf", sec);

	//------------------------------------------------------------ Calc now speed
	speed_u -= encoder[0]*sin( -M_PI/6 );
	speed_v -= encoder[0]*cos( -M_PI/6 );

	speed_u -= encoder[1]*sin( M_PI/2 );
	speed_v -= encoder[1]*cos( M_PI/2 );

	speed_u -= encoder[2]*sin( -M_PI*5/6 );
	speed_v -= encoder[2]*cos( -M_PI*5/6 );

	//------------------------------------------------------------ Calc final target speeds of each wheels
	if( speed > 0.0 ){
		t[0] = speed * -sin( rad - M_PI/6 );
		t[1] = speed * -cos( rad );
		t[2] = speed * -sin( rad - M_PI*5/6 );

		//------------------------------------------------------------ Prevent slip: slow acceleration
		// We want to get max(k)
		//
		// 0 <= k <= 1
		// -acc <= k*t[0]-encoder[0] << acc
		// -acc <= k*t[1]-encoder[1] << acc
		// -acc <= k*t[2]-encoder[2] << acc
		//------------------------------------------------------------
		// 0 <= k <= 1
		// -acc + encoder[0] <= k*t[0] << acc + encoder[0]
		// -acc + encoder[1] <= k*t[1] << acc + encoder[1]
		// -acc + encoder[2] <= k*t[2] << acc + encoder[2]
		//------------------------------------------------------------

		bool flag_in_range = true;

		float k = 1.0f;
		for(int i=0; i<3; i++){

			if( t[i] != 0.0f ){
				float min_k;
				float max_k;

				if( t[i] > 0 ){
					min_k = (-acc + encoder[i])/t[i];
					max_k = ( acc + encoder[i])/t[i];
				}
				else{
					max_k = (-acc + encoder[i])/t[i];
					min_k = ( acc + encoder[i])/t[i];
				}

				if( min_k > 1.0f || max_k < 0 ){
					flag_in_range = false;
					break;
				}

				if( k > max_k ) k = max_k;

			}
			else{
				if( !(-acc + encoder[i] <= 0.0f && 0.0f <= acc + encoder[i]) ){
					flag_in_range = false;
					break;
				}
			}
		}

		//ROS_INFO("%d %.3f %.3f %.3f",flag_in_range==true?1:0, k*t[0], k*t[1], k*t[2] );

		if( flag_in_range == true){
			for(int i=0; i<3; i++) target_speed[i] = k*t[i];
			return;
		}

	}

	//------------------------------------------------------------ reduce speed
	t[0] = t[1] = t[2] = 0.0f;

	float max_enc = std::abs( std::max({encoder[0],encoder[1],encoder[2]}) );

	if( max_enc > acc ){
		float rate = (max_enc-acc)/max_enc;
		for(int i=0; i<3; i++) target_speed[i] = rate*encoder[i];
	}
	else{
		target_speed[0] = target_speed[1] = target_speed[2] = 0.0;
	}

}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "Machine");
	Machine machine;

	ros::spin();
}
