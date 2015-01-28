#include <ros/ros.h>

#include <opencv2/opencv.hpp>
#include <cv.h>
#include <highgui.h>
#include <cvblob.h>
#include <opencv2/imgproc/imgproc.hpp>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <visualization_msgs/Marker.h>	//for displaying points of a shuttle
#include <geometry_msgs/PoseArray.h>	//for publish points of a shuttle

#include <pthread.h>
#include <math.h>

#ifndef M_PI
#define M_PI	3.14159265358979
#endif

using namespace cv;
using namespace cvb;

pthread_mutex_t	mutexLeft, mutexRight;
Mat left_frame;
Mat right_frame;

ros::Time left_timestamp;
ros::Time right_timestamp;

bool left_recieved = false;
bool right_recieved = false;

bool endflag = false;

cv::BackgroundSubtractorGMG left_backGroundSubtractor;
cv::BackgroundSubtractorGMG right_backGroundSubtractor;

Mat leftMask, rightMask;


void thread_main();

void leftCallback(const sensor_msgs::ImageConstPtr& msg)
{
	cv_bridge::CvImagePtr cv_ptr_depth;
	try{
		cv_ptr_depth = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
	}

	catch (cv_bridge::Exception& e){
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}
	pthread_mutex_lock( &mutexLeft );

	left_backGroundSubtractor(cv_ptr_depth->image, leftMask);
	left_frame = cv_ptr_depth->image;

	left_timestamp = ros::Time::now();
	left_recieved = true;
	pthread_mutex_unlock( &mutexLeft );
	// convert message from ROS to openCV

}


void rightCallback(const sensor_msgs::ImageConstPtr& msg)
{
	cv_bridge::CvImagePtr cv_ptr_depth;
	try{
		cv_ptr_depth = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
	}

	catch (cv_bridge::Exception& e){
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}
	pthread_mutex_lock( &mutexRight );

	right_backGroundSubtractor(cv_ptr_depth->image, rightMask);
	right_frame = cv_ptr_depth->image;

	right_timestamp = ros::Time::now();
	right_recieved = true;
	pthread_mutex_unlock( &mutexRight );
	// convert message from ROS to openCV

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "image_listener");
  ros::NodeHandle nh;

  cv::startWindowThread();

  image_transport::ImageTransport it(nh);
  image_transport::Subscriber left_sub = it.subscribe("/stereo/left/image_rect_color", 1, leftCallback);
  image_transport::Subscriber right_sub = it.subscribe("/stereo/right/image_rect_color", 1, rightCallback);

  pthread_t thread;
  pthread_create( &thread, NULL, (void* (*)(void*))thread_main, NULL );

  ros::spin();

  endflag = true;
  pthread_join( thread, NULL );
  destroyAllWindows();

  return 0;
}

void thread_main(){
	ROS_INFO("New thread Created.");
	pthread_detach( pthread_self( ));

	namedWindow( "leftFrame", WINDOW_AUTOSIZE );
	namedWindow( "rightFrame", WINDOW_AUTOSIZE );


	while(!endflag){

		Mat foreGroundLeft, outputLeft;
		Mat foreGroundRight, outputRight;

		//wait for recieve new frame
		while( !left_recieved || !right_recieved ){
			sleep(1);
		}

		//Get new frame
		pthread_mutex_lock( &mutexLeft );
		pthread_mutex_lock( &mutexRight );
		cv::Mat leftMat;
		left_frame.copyTo(leftMat);
		ros::Time l_timestamp = left_timestamp;

		cv::Mat rightMat;
		right_frame.copyTo(rightMat);
		ros::Time r_timestamp = right_timestamp;

		leftMask.copyTo(foreGroundLeft);
		rightMask.copyTo(foreGroundRight);

		pthread_mutex_unlock( &mutexLeft );
		pthread_mutex_unlock( &mutexRight );



		//cv::erode(foreGroundLeft, foreGroundLeft, cv::Mat() );
		//cv::dilate(foreGroundLeft, foreGroundLeft, cv::Mat());
		cv::bitwise_and(leftMat, leftMat, outputLeft, foreGroundLeft);	// 入力画像にマスク処理を行う


		//cv::erode(foreGroundRight, foreGroundRight, cv::Mat() );
		//cv::dilate(foreGroundRight, foreGroundRight, cv::Mat());
		cv::bitwise_and(rightMat, rightMat, outputRight, foreGroundRight);	// 入力画像にマスク処理を行う

		cv::imshow("leftFrame", outputLeft);
		cv::imshow("rightFrame", outputRight);

	}
}
