#include <ros/ros.h>

#include <opencv2/opencv.hpp>
#include <cv.h>
#include <highgui.h>
#include <cvblob.h>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <visualization_msgs/Marker.h>	//for displaying points of a shuttle
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <std_msgs/Float32.h>

#include <pthread.h>
#include <math.h>

#ifndef M_PI
#define M_PI	3.14159265358979
#endif

#define MARGIN_AROUND	30

using namespace cv;
using namespace cvb;

pthread_mutex_t	mutex;  // MUTEX
Mat depth_frame;
ros::Time depth_timestamp;
bool recieved = false;
bool endflag = false;

pthread_mutex_t	pose_mutex;  // MUTEX
geometry_msgs::Pose _pose;

float kinect_pitch=0.0f;

class KinectV2{
protected:
	float kinect_rad;
	float kinect_sin,kinect_cos;
public:
	double offset_x, offset_y, offset_z;

	KinectV2(){
		setKinectRad(0.0f*M_PI/180);
		offset_x = offset_y = offset_z = 0.0f;
	}

	void setKinectRad(float rad){
		kinect_rad = rad;
		kinect_sin = sin(rad);
		kinect_cos = cos(rad);
	}

	float RawDepthToMeters(int depthValue)
	{
		return (double)depthValue / 1000;
	}
	geometry_msgs::Point DepthToWorld(int x, int y, int depthValue)
	{
		float fx_d = 0.0027697133333333;
		float fy_d = -0.00271;
		float cx_d = 256;
		float cy_d = 214;

		geometry_msgs::Point result;
		float depth = RawDepthToMeters(depthValue);

		float tx = -(float)((x - cx_d) * depth * fx_d);
		float ty = (float)((y - cy_d) * depth * fy_d);
		float tz = (float)(depth);

		result.x = tx+offset_x;
		result.z = ty*kinect_cos + tz*kinect_sin+offset_z;
		result.y = ty*kinect_sin + tz*kinect_cos+offset_y;
		return result;
	}
};

KinectV2 kinect;

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
	depth_timestamp = ros::Time::now();
	recieved = true;
	pthread_mutex_unlock( &mutex );
	// convert message from ROS to openCV



}

void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg){
	//ROS_INFO("poseCallback");
	pthread_mutex_lock( &pose_mutex );
	_pose = msg->pose;
	pthread_mutex_unlock( &pose_mutex );

}

void servoCallback(const std_msgs::Float32::ConstPtr& msg){
	//ROS_INFO("poseCallback");
	kinect.setKinectRad(msg->data);
}


void transformToGlobalFrame( geometry_msgs::Point* output, const geometry_msgs::Point* shuttle, const geometry_msgs::Pose* robot){
	float yaw = -atan2(2.0*(robot->orientation.x*robot->orientation.y + robot->orientation.w*robot->orientation.z), robot->orientation.w*robot->orientation.w + robot->orientation.x*robot->orientation.x - robot->orientation.y*robot->orientation.y - robot->orientation.z*robot->orientation.z);
	output->x = robot->position.x + shuttle->x*cos(yaw) + shuttle->y*sin(yaw);
	output->y = robot->position.y + shuttle->x*sin(yaw) + shuttle->y*cos(yaw);
	output->z = robot->position.z + shuttle->z;
}
int main(int argc, char **argv)
{
  ros::init(argc, argv, "shuttleFinder");
  ros::NodeHandle nh;

  ros::NodeHandle local_nh("~");

  if (!local_nh.hasParam("offset_x")){
	  kinect.offset_x = 0.0f;
  }
  else{
	  if (!local_nh.getParam("offset_x", kinect.offset_x)){
		  ROS_ERROR("parameter offset_x is invalid.");
		  return -1;
	  }
  }

  if (!local_nh.hasParam("offset_y")){
	  kinect.offset_y = 0.0f;
  }
  else{
	  if (!local_nh.getParam("offset_y", kinect.offset_y)){
		  ROS_ERROR("parameter offset_y is invalid.");
		  return -1;
	  }
  }

  if (!local_nh.hasParam("offset_z")){
	  kinect.offset_z = 0.0f;
  }
  else{
	  if (!local_nh.getParam("offset_z", kinect.offset_z)){
		  ROS_ERROR("parameter offset_z is invalid.");
		  return -1;
	  }
  }

  ROS_INFO("offset_x=%.3f, offset_y=%.3f, offset_z=%.3f",kinect.offset_x, kinect.offset_y, kinect.offset_z);



  cv::startWindowThread();

  image_transport::ImageTransport it(nh);
  image_transport::Subscriber sub = it.subscribe("/kinect/depth", 1, imageCallback);

  ros::Subscriber subPose = nh.subscribe("/robot/pose", 10, poseCallback);
  ros::Subscriber subAngle = nh.subscribe("/kinect/angle", 10, servoCallback);

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
	namedWindow( "frame", WINDOW_AUTOSIZE );
	namedWindow( "bin", WINDOW_AUTOSIZE );
	namedWindow( "shuttle", WINDOW_AUTOSIZE );


	Mat depthMat8bit;
	Mat depthMask(424, 512,CV_8U);

	cv::BackgroundSubtractorGMG backGroundSubtractor;
	//cv::BackgroundSubtractorMOG backGroundSubtractor;
	//cv::BackgroundSubtractorMOG2 backGroundSubtractor;

	ros::NodeHandle n;
	ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("/shuttle/marker", 1);
	ros::Publisher shuttle_pub = n.advertise<geometry_msgs::PointStamped>("/shuttle/point", 10);

	CvBlobs blobs;

	geometry_msgs::Pose lastPoint;
	bool is_shuttle_found_at_last_frame = false;
	cv::Point lastMinPoint(0,0);
	int lastMin = 65535;

	ros::Time timestamp = ros::Time::now();

	geometry_msgs::Pose robot_pose;

	while(!endflag){

		visualization_msgs::Marker points;
		points.header.frame_id = "/map";
		points.ns = "points_and_lines";
		points.action = visualization_msgs::Marker::ADD;
		points.pose.orientation.w = 1.0;
		points.lifetime = ros::Duration(1000.0);

		points.id = 0;
		points.type = visualization_msgs::Marker::POINTS;

		// POINTS markers use x and y scale for width/height respectively
		points.scale.x = 0.1;
		points.scale.y = 0.1;

		// Points are green
		points.color.r = 1.0f;
		points.color.a = 1.0;

		//wait for recieve new frame
		while(recieved == false){
			ros::Duration(0.001).sleep();
		}

		cv::Mat depthMat;

		//Get new frame

		pthread_mutex_lock( &pose_mutex );
		pthread_mutex_lock( &mutex );
		robot_pose = _pose;
		pthread_mutex_unlock( &pose_mutex );

		if( timestamp == depth_timestamp){
			pthread_mutex_unlock( &mutex );
			continue;
		}

		depth_frame.copyTo(depthMat);
		timestamp = depth_timestamp;

		recieved = false;

		pthread_mutex_unlock( &mutex );

		cv::Mat depthMat8bit(depthMat);

		points.header.stamp = timestamp;

		geometry_msgs::PointStamped shuttle;
		shuttle.header.stamp = timestamp;
		shuttle.header.frame_id = "map";


		depthMat8bit.convertTo(depthMat8bit, CV_8U, 255.0 / 8000.0);

		cv::erode(depthMat8bit, depthMat8bit, cv::Mat() );
		cv::dilate(depthMat8bit, depthMat8bit, cv::Mat());

		cv::threshold(depthMat8bit, depthMask, 500*255.0 / 8000.0 , 255, cv::THRESH_BINARY_INV);
		depthMask.copyTo(depthMat8bit, depthMask);

		IplImage depth8bit = depthMat8bit;
		IplImage *frame = cvCreateImage(cvGetSize(&depth8bit), IPL_DEPTH_8U, 3);
		cvCvtColor( &depth8bit , frame, CV_GRAY2BGR );

		Mat foreGroundMask;
		Mat output;

		backGroundSubtractor(depthMat8bit, foreGroundMask);

		// 入力画像にマスク処理を行う
		//cv::bitwise_and(depthMat8bit, depthMat8bit, output, foreGroundMask);

		//-----------------------------------------------------------Search around a point which the shuttle is detected on the last frame

		bool shuttle_found = false;

		if( is_shuttle_found_at_last_frame == true ){
			//ROS_INFO("A shuttle is detected on the last frame");

#define MARGIN_NEARPIXEL	45

			cv::Mat depth8bit(depthMat);
			depth8bit.convertTo(depth8bit, CV_8U, 255.0 / 8000.0);

			cv::Rect aroundRect;
			aroundRect.x		= lastMinPoint.x-MARGIN_NEARPIXEL;
			aroundRect.y		= lastMinPoint.y-MARGIN_NEARPIXEL;
			aroundRect.width	= MARGIN_NEARPIXEL*2;
			aroundRect.height	= MARGIN_NEARPIXEL*3;

			if( aroundRect.x < 0)	aroundRect.x = 0;
			if( aroundRect.y < 0)	aroundRect.y = 0;

			if( aroundRect.x + aroundRect.width >=512 )	aroundRect.width = 511 - aroundRect.x;
			if( aroundRect.y + aroundRect.height >=424 )aroundRect.height = 423 - aroundRect.y;

			//lastMin

			cv::Mat roi8(depth8bit, aroundRect);
			cv::Mat roi16(depthMat, aroundRect);

			roi8.copyTo(roi8);

			Mat dMask(roi8.rows, roi8.cols,CV_8U);

			cv::threshold(roi8, dMask, (lastMin+100)*255.0 / 8000.0 , 255, cv::THRESH_BINARY);
			dMask.copyTo(roi8, dMask);

			cv::threshold(roi8, dMask, (lastMin-500)*255.0 / 8000.0 , 255, cv::THRESH_BINARY_INV);
			dMask.copyTo(roi8, dMask);
#if 1
			cv::threshold(roi8, dMask, 254 , 255, cv::THRESH_BINARY);

			cv::bitwise_not(dMask,dMask);
			cv::erode(dMask, dMask, cv::Mat() );
			cv::erode(dMask, dMask, cv::Mat() );
			cv::dilate(dMask, dMask, cv::Mat());
			cv::dilate(dMask, dMask, cv::Mat());
			cv::dilate(dMask, dMask, cv::Mat());
			cv::bitwise_not(dMask,dMask);


			dMask.copyTo(roi8, dMask);
#endif

			cv::Point minPoint(0,0);
			int min = 65535;
			int count =0;

			geometry_msgs::Point center;
			center.x = 0.0f;
			center.y = 0.0f;
			center.z = 0.0f;


			for (int y = 0; y < aroundRect.height; y++)
			{
				for (int x = 0; x < aroundRect.width; x++)
				{
					int val8bit = roi8.at<unsigned char>(y,x);

					if (val8bit > 0 && val8bit < 0xff){
						count++;
						geometry_msgs::Point p = kinect.DepthToWorld( aroundRect.x+x, aroundRect.y+y, roi16.at<unsigned short>(y,x) );
						center.x += p.x;
						center.y += p.y;
						center.z += p.z;
					}

					if (val8bit != 0 && val8bit < min)
					{
						min = val8bit;
						minPoint.x= x;
						minPoint.y = y;
					}
				}
			}

			if( count > 0){
				center.x /= count;
				center.y /= count;
				center.z /= count;
			}

			cv::Mat colorTmp;
			cv::cvtColor(roi8, colorTmp, CV_GRAY2BGR);
			cv::circle(colorTmp, minPoint, 10, CV_RGB(0,255,0),3);

			cv::imshow("shuttle", colorTmp);

			minPoint.x += aroundRect.x;
			minPoint.y += aroundRect.y;
			min = depthMat.at<unsigned short>(minPoint.y,minPoint.x);
			//int nearPointIndex = minPoint.x + (minPoint.y) * depthMat.cols;
			geometry_msgs::Point nearest_p = kinect.DepthToWorld(minPoint.x, minPoint.y, min);

			cvCircle(frame, minPoint, 10, CV_RGB(0,255,0),3);

			//ROS_INFO("%d", count);
			if( count >= 40 ){
				shuttle_found = true;

				geometry_msgs::Point point;
				transformToGlobalFrame(&point, &nearest_p, &robot_pose);
				points.points.push_back(point);
				shuttle.point = point;

				//ROS_INFO("%.4f, %f, %f, %f", timestamp.toSec(), nearest_p.x, nearest_p.y, nearest_p.z );

				lastMinPoint = minPoint;
				lastMin = min;
			}

		}


		if( !shuttle_found ){
			//-----------------------------------------------------------
			blobs.clear();

			IplImage dstImg = foreGroundMask;
			IplImage *labelImg = cvCreateImage(cvGetSize(&dstImg), IPL_DEPTH_LABEL, 1);
			cvLabel(&dstImg, labelImg, blobs);
			cvFilterByArea(blobs, 20, 10000);

			//IplImage iplImage = foreGroundMask;
			//cvCvtColor(&iplImage, frame, CV_GRAY2BGR );

			IplImage *imgOut = cvCreateImage(cvGetSize(&dstImg), IPL_DEPTH_8U, 3); cvZero(imgOut);

			//cvRenderBlobs(labelImg, blobs, frame, frame, CV_BLOB_RENDER_BOUNDING_BOX);
			//cvUpdateTracks(blobs, tracks, 200., 5);
			//cvRenderTracks(tracks, frame, frame, CV_TRACK_RENDER_ID|CV_TRACK_RENDER_BOUNDING_BOX);

			for (CvBlobs::const_iterator it=blobs.begin(); it!=blobs.end(); ++it){

				bool is_shuttle = false;

				//-------------------------------------------------------Detect nearest point
				cv::Rect roi_rect;
				roi_rect.x	= it->second->minx;
				roi_rect.y	= it->second->miny;
				roi_rect.width = it->second->maxx - it->second->minx;
				roi_rect.height = it->second->maxy - it->second->miny;

				if( roi_rect.width > 130 ||  roi_rect.height > 130){	//Too large
					continue;
				}

				cv::Point minPoint(0,0);
				int min = 65535;

				cv::Mat roi8(depthMat8bit, roi_rect);

				for (int y = 0; y < roi_rect.height; y++)
				{
					for (int x = 0; x < roi_rect.width; x++)
					{
						int val8bit = roi8.at<unsigned short>(y,x);
						//int val = roi.data[y*roi_rect.width + x];
						if (val8bit != 0 && val8bit < min)
						{
							min = val8bit;
							minPoint.x= x;
							minPoint.y = y;
						}
					}
				}
				minPoint.x += roi_rect.x;
				minPoint.y += roi_rect.y;
				min = depthMat.at<unsigned short>(minPoint.y,minPoint.x);
				//int nearPointIndex = minPoint.x + (minPoint.y) * depthMat.cols;
				geometry_msgs::Point nearest_p = kinect.DepthToWorld(minPoint.x, minPoint.y, min);

				//-------------------------------------------------------Filter by depth
				if(minPoint.x < 50 || minPoint.x > 512-50 ||
						minPoint.y < 50 || minPoint.y > 424-50){
					continue;
				}

				if( nearest_p.y < 1.0 ){
					continue;
				}
#if 1
				if( atan2(nearest_p.z, nearest_p.x) < 15*M_PI/180){
					continue;
				}
#endif
				//-------------------------------------------------------Check around the point
				cv::Rect aroundRect;
				aroundRect.x		= roi_rect.x-MARGIN_AROUND;
				aroundRect.y		= roi_rect.y-MARGIN_AROUND;
				aroundRect.width	= roi_rect.width+MARGIN_AROUND*2;
				aroundRect.height	= roi_rect.height+MARGIN_AROUND*2;

				if( aroundRect.x < 0)	aroundRect.x = 0;
				if( aroundRect.y < 0)	aroundRect.y = 0;

				if( aroundRect.x + aroundRect.width >=512 )	aroundRect.width = 511 - aroundRect.x;
				if( aroundRect.y + aroundRect.height >=424 )aroundRect.height = 423 - aroundRect.y;

				//-------------------------------------------------------Check using binary image
				cv::Mat roi8bit(depthMat8bit, aroundRect);
				Mat bin_img;

				int threshold8bit = (min+700) * 255.0 / 8000.0;
				threshold(roi8bit, bin_img, threshold8bit, 255, THRESH_BINARY_INV);

				cv::dilate(bin_img, bin_img, cv::Mat());



				CvBlobs roiBlobs;

				roiBlobs.clear();
				IplImage dstRoiBinImg = bin_img;
				IplImage *blob_labelImg = cvCreateImage(cvGetSize( &dstRoiBinImg ), IPL_DEPTH_LABEL, 1);
				cvLabel( &dstRoiBinImg  , blob_labelImg, roiBlobs);
				cvFilterByArea(roiBlobs, 50, 100000);

				cvRectangle(frame, cvPoint(aroundRect.x, aroundRect.y), cvPoint(aroundRect.x + aroundRect.width, aroundRect.y + aroundRect.height), CV_RGB(0,0,255), 1);

				is_shuttle = false;

				if(roiBlobs.size() > 1){	//If it is a shuttle, count of blob must be 1
					continue;
				}

				for (CvBlobs::const_iterator it=roiBlobs.begin(); it!=roiBlobs.end(); ++it){
					cv::Rect rect;
					rect.x	= it->second->minx;
					rect.y	= it->second->miny;
					rect.width = it->second->maxx - it->second->minx;
					rect.height = it->second->maxy - it->second->miny;

					if( rect.width > 50 ||  rect.height > 50){	//Too large
						continue;
					}

					if( rect.x > MARGIN_AROUND/4 && rect.y > MARGIN_AROUND/4 &&
							rect.x+rect.width <  aroundRect.width - MARGIN_AROUND*3/4 &&
							rect.y+rect.height <  aroundRect.height - MARGIN_AROUND*3/4
					){
						is_shuttle = true;
						break;
					}
				}

				if( !is_shuttle ){
					continue;
				}

				cv::imshow("bin", bin_img);

#if 0
				////-------------------------------------------------------Check around the point

				cv::Mat roi_around(depthMat, aroundRect);

				cvRectangle(frame, cvPoint(aroundRect.x, aroundRect.y), cvPoint(aroundRect.x + aroundRect.width, aroundRect.y + aroundRect.height), CV_RGB(255,0,255), 1);

				is_shuttle = true;
				for (int y = 0; y < aroundRect.height; y++)
				{
					for (int x = 0; x < aroundRect.width; x++)
					{
						int val = roi_around.at<unsigned short>(y,x);
						//int val = depthMat.at<unsigned short>(aroundRect.y+y,aroundRect.x+x);
						if (val > min && val < min+500 )
						{
							geometry_msgs::Point p = kinect.DepthToWorld(aroundRect.x+x, aroundRect.y+y, val);
							float dist_pow2 = (p.x-nearest_p.x)*(p.x-nearest_p.x) + (p.y-nearest_p.y)*(p.y-nearest_p.y) + (p.z-nearest_p.z)*(p.z-nearest_p.z);
							if( dist_pow2 > 0.25f*0.25f && dist_pow2 < 0.5f*0.5f ){
								is_shuttle = false;
								goto not_shuttle;
							}
						}
					}
				}
				not_shuttle:
				if( !is_shuttle ){
					continue;
				}
#endif
				//-------------------------------------------------------Draw the point
				cvCircle(frame, minPoint, 10, CV_RGB(255,0,0),3);

				if(depthMat.at<unsigned short>(minPoint.y,minPoint.x) > 600){
					shuttle_found = true;

					//ROS_INFO("%.4f, %f, %f, %f", timestamp.toSec(), nearest_p.x, nearest_p.y, nearest_p.z );

					geometry_msgs::Point point;
					transformToGlobalFrame(&point, &nearest_p, &robot_pose);
					points.points.push_back(point);
					shuttle.point = point;

					lastMinPoint = minPoint;
					lastMin = min;
					break;
				}


			}

			cvReleaseImage(&labelImg);
			cvReleaseImage(&imgOut);
		}


		cv::imshow("frame", cvarrToMat(frame));
		cvReleaseImage(&frame);


		if( shuttle_found ){
			is_shuttle_found_at_last_frame = true;

			marker_pub.publish(points);
			shuttle_pub.publish(shuttle);
		}
		else{
			is_shuttle_found_at_last_frame = false;
		}

	}
}
