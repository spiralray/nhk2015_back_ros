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

pthread_mutex_t	mutex;  // MUTEX
Mat left_frame;
ros::Time left_timestamp;
bool recieved = false;
bool endflag = false;


void thread_main();

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
	cv_bridge::CvImagePtr cv_ptr_depth;
	try{
		cv_ptr_depth = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_8UC3);
	}

	catch (cv_bridge::Exception& e){
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}
	pthread_mutex_lock( &mutex );
	left_frame = cv_ptr_depth->image;
	left_timestamp = ros::Time::now();
	recieved = true;
	pthread_mutex_unlock( &mutex );
	// convert message from ROS to openCV



}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "image_listener");
  ros::NodeHandle nh;

  cv::startWindowThread();

  image_transport::ImageTransport it(nh);
  image_transport::Subscriber sub = it.subscribe("/stereo/left/image_rect_color", 1, imageCallback);

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

	//namedWindow( "depth_image", WINDOW_AUTOSIZE );
	//namedWindow( "output", WINDOW_AUTOSIZE );
	namedWindow( "leftRaw", WINDOW_AUTOSIZE );
	namedWindow( "leftFrame", WINDOW_AUTOSIZE );

	cv::BackgroundSubtractorGMG backGroundSubtractor;
	//cv::BackgroundSubtractorMOG backGroundSubtractor;
	//cv::BackgroundSubtractorMOG2 backGroundSubtractor;


	while(!endflag){

		Mat foreGroundLeft, outputLeft;

		//wait for recieve new frame
		while(recieved == false){
			sleep(1);
		}

		//Get new frame
		pthread_mutex_lock( &mutex );
		cv::Mat leftMat;
		left_frame.copyTo(leftMat);
		ros::Time timestamp = left_timestamp;
		pthread_mutex_unlock( &mutex );

		cv::cvtColor( leftMat, leftMat, CV_BGR2RGB);

		backGroundSubtractor(leftMat, foreGroundLeft);

		cv::erode(foreGroundLeft, foreGroundLeft, cv::Mat() );
		cv::dilate(foreGroundLeft, foreGroundLeft, cv::Mat());

		// 入力画像にマスク処理を行う
		cv::bitwise_and(leftMat, leftMat, outputLeft, foreGroundLeft);

		cv::imshow("leftRaw", leftMat);
		cv::imshow("leftFrame", outputLeft);

	}
}
