#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <visualization_msgs/Marker.h>

#include <opencv2/opencv.hpp>
#include <cv.h>
#include <highgui.h>
#include <cvblob.h>

#include <pthread.h>

using namespace cv;
using namespace cvb;

pthread_mutex_t	mutex;  // MUTEX
Mat depth_frame;
bool recieved = false;
bool endflag = false;

void thread_main();

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
	cv_bridge::CvImagePtr cv_ptr_depth;
	try{
		cv_ptr_depth = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1);
	}

	catch (cv_bridge::Exception& e){
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}
	pthread_mutex_lock( &mutex );
	depth_frame = cv_ptr_depth->image;
	recieved = true;
	pthread_mutex_unlock( &mutex );
	// convert message from ROS to openCV



}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "kinect2_points");
  ros::NodeHandle nh;

  cv::startWindowThread();

  image_transport::ImageTransport it(nh);
  image_transport::Subscriber sub = it.subscribe("/kinect/depth", 1, imageCallback);

  pthread_t thread;
  pthread_create( &thread, NULL, (void* (*)(void*))thread_main, NULL );

  ros::spin();

  endflag = true;
  pthread_join( thread, NULL );
  destroyAllWindows();

  return 0;
}

double RawDepthToMeters(int depthValue)
{
	return (double)depthValue / 1000;
}
geometry_msgs::Point DepthToWorld(int x, int y, int depthValue)
{
	double fx_d = 0.0027697133333333;
	double fy_d = -0.00271;
	double cx_d = 256;
	double cy_d = 214;

	geometry_msgs::Point result;
	double depth = RawDepthToMeters(depthValue);
	result.y = -(float)((x - cx_d) * depth * fx_d);
	result.z = (float)((y - cy_d) * depth * fy_d);
	result.x = (float)(depth);
	return result;
}

void thread_main(){
	ROS_INFO("New thread Created.");
	pthread_detach( pthread_self( ));

	ros::NodeHandle n;
	ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("kinect2_points", 10);

	CvBlobs blobs;

	while(!endflag){

		visualization_msgs::Marker points;
		points.header.frame_id = "/laser";
		points.header.stamp = ros::Time::now();
		points.ns = "points_and_lines";
		points.action = visualization_msgs::Marker::ADD;
		points.pose.orientation.w = 1.0;

		points.id = 0;
		points.type = visualization_msgs::Marker::POINTS;

		// POINTS markers use x and y scale for width/height respectively
		points.scale.x = 0.01;
		points.scale.y = 0.01;

		points.color.r = 0.5f;
		points.color.g = 1.0f;
		points.color.b = 0.5f;
		points.color.a = 0.5;

		while(recieved == false){
			cv::waitKey(1);
		}
		pthread_mutex_lock( &mutex );
		cv::Mat depthMat(depth_frame);
		pthread_mutex_unlock( &mutex );

		for(int y=0;y<424;y+=7){
			for(int x=0; x<512; x+=7){
				if( depthMat.at<unsigned short>(y,x) >100 && depthMat.at<unsigned short>(y,x) < 10000 ){
					geometry_msgs::Point p = DepthToWorld(x, y, depthMat.at<unsigned short>(y,x));
					points.points.push_back(p);
				}
			}
		}

		marker_pub.publish(points);
	}
}
