#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PoseStamped.h>

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

float motor1 = 0.0f;
float motor2 = 0.0f;

float enc1 = 0.0f;
float enc2 = 0.0f;

void motor1callback(const std_msgs::Float32::ConstPtr& msg){
	motor1 = msg->data;
}
void motor2callback(const std_msgs::Float32::ConstPtr& msg){
	motor2 = msg->data;
}

void enc1callback(const std_msgs::Float32::ConstPtr& msg){
	enc1 = msg->data;
}
void enc2callback(const std_msgs::Float32::ConstPtr& msg){
	enc2 = msg->data;
}

void timerCallback(const ros::TimerEvent&){
	cv::Mat img = cv::Mat::zeros(300, 600, CV_8UC3);

	cv::line(img, cv::Point(300+300*motor1, 150), cv::Point( 300+300*motor1+100*cos(CV_PI/2-motor2) , 150-100*sin(CV_PI/2-motor2) ), cv::Scalar(200,0,0), 10, CV_AA);

	cv::line(img, cv::Point(300+300*enc1, 150), cv::Point( 300+300*enc1+100*cos(CV_PI/2-enc2) , 150-100*sin(CV_PI/2-enc2) ), cv::Scalar(0,200,200), 10, CV_AA);

	cv::imshow("drawing", img);
	cv::waitKey(1);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "racketDisp");

	ros::NodeHandle n;

	ros::Subscriber subMotor1 = n.subscribe("/mb1/motor1", 10, motor1callback);
	ros::Subscriber subMotor2 = n.subscribe("/mb1/motor2", 10, motor2callback);

	ros::Subscriber subEnc1 = n.subscribe("/mb1/enc1", 10, enc1callback);
	ros::Subscriber subEnc2 = n.subscribe("/mb1/enc2", 10, enc2callback);

	ros::Timer timer = n.createTimer(ros::Duration(0.033), timerCallback);

	cv::namedWindow("drawing", CV_WINDOW_AUTOSIZE|CV_WINDOW_FREERATIO);

	ros::spin();

	return 0;
}
