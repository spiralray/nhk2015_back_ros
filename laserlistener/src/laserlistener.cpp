#include <ros/ros.h>
#include "opencv2/opencv.hpp"
#include "sensor_msgs/LaserScan.h"
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PoseStamped.h>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include <pthread.h>

#include "tf/tf.h"

//#define CV_PRINT

int image_size;
int scale;
int threshold;

#define FIELD_WIDTH 8.420


#define DISTANCE_LRF_TO_CENTER	0.455	//[m]
#define FENCE_WIDTH				0.04	//[m]

pthread_mutex_t	mutex;  // MUTEX
sensor_msgs::LaserScan lasermsg;
ros::Time laser_timestamp;
bool recieved = false;
bool endflag = false;

typedef struct{
	double dist;
	double theta;
} Line;

void thread_main();

void LaserCallback(const sensor_msgs::LaserScan& msg)
{
	//ROS_INFO("Received [%f][%f][%f]", msg.scan_time, msg.angle_min, msg.angle_max );

	pthread_mutex_lock( &mutex );
	lasermsg = msg;
	laser_timestamp = ros::Time::now();
	recieved = true;
	pthread_mutex_unlock( &mutex );

}

int main(int argc, char** argv){

	ros::init(argc, argv, "laserlistener");
	ros::NodeHandle n;
	ros::NodeHandle local_nh("~");

	if (!local_nh.hasParam("image_size")){
		ROS_INFO("Parameter image_size is not defined. Now, it is set default value.");
		local_nh.setParam("image_size", 1000);
	}

	if (!local_nh.getParam("image_size", image_size)){
		ROS_ERROR("parameter image_size is invalid.");
		return -1;
	}
	ROS_INFO("image_size: %d",image_size);

	if (!local_nh.hasParam("scale")){
		ROS_INFO("Parameter scale is not defined. Now, it is set default value.");
		local_nh.setParam("scale", 70);
	}

	if (!local_nh.getParam("scale", scale)){
		ROS_ERROR("parameter scale is invalid.");
		return -1;
	}
	ROS_INFO("scale: %d",scale);

	if (!local_nh.hasParam("threshold")){
		ROS_INFO("Parameter threshold is not defined. Now, it is set default value.");
		local_nh.setParam("threshold", 60);
	}

	if (!local_nh.getParam("threshold", threshold)){
		ROS_ERROR("parameter threshold is invalid.");
		return -1;
	}
	ROS_INFO("threshold: %d",threshold);


	ros::Subscriber subscriber = n.subscribe("scan", 100, LaserCallback);

	pthread_t thread;
	pthread_create( &thread, NULL, (void* (*)(void*))thread_main, NULL );

	ros::spin();

	endflag = true;
	pthread_join( thread, NULL );

	return 0;
}

void thread_main(){
	ROS_INFO("New thread Created.");
	pthread_detach( pthread_self( ));

	ros::Publisher publisher;
	image_transport::Publisher image_pub;

	ros::NodeHandle n;
	publisher = n.advertise<visualization_msgs::Marker>("lines", 100);

	ros::Publisher pose_pub;
	pose_pub = n.advertise<geometry_msgs::PoseStamped>("lrf_pose", 100);

	geometry_msgs::PoseStamped pose;
	pose.header.frame_id = "/map";
	pose.pose.position.x = 0.0f;
	pose.pose.position.y = 0.0f;
	pose.pose.position.z = 0.0f;
	pose.pose.orientation.w = 0.0f;
	pose.pose.orientation.x = 0.0f;
	pose.pose.orientation.y = 0.0f;
	pose.pose.orientation.z = 0.0f;


	double yaw = 0.0;	//Pose of robot

	image_transport::ImageTransport it(n);
	image_pub = it.advertise("laser/image", 1);

	while(!endflag){

		while(recieved == false){
			ros::Duration(0.001).sleep();
		}
		pthread_mutex_lock( &mutex );
		sensor_msgs::LaserScan msg = lasermsg;
		ros::Time timestamp = laser_timestamp;
		pthread_mutex_unlock( &mutex );

		recieved = false;

		visualization_msgs::Marker line_list;
		line_list.header.frame_id = "/laser";
		line_list.header.stamp = timestamp;
		line_list.ns = "line";
		line_list.id = 0;
		line_list.type = visualization_msgs::Marker::LINE_LIST;
		line_list.action = visualization_msgs::Marker::ADD;

		line_list.scale.x = 0.01;

		line_list.color.r = 1.0;
		line_list.color.a = 1.0;

		pose.header.stamp = timestamp;


		//メモリの確保
		IplImage* img = cvCreateImage(cvSize(image_size,image_size), IPL_DEPTH_8U, 3);
		cvSet(img, cvScalar(0,0,0));

		float offset = -90*M_PI/180 + msg.angle_min;
		int points = (msg.angle_max - msg.angle_min)/msg.angle_increment;
		CvPoint pt1;
		for(int i=0;i<points-1;i++){
			pt1.x = image_size/2+(msg.ranges[i]*cos(offset+i*msg.angle_increment))*scale;
			pt1.y = image_size/2+(msg.ranges[i]*sin(offset+i*msg.angle_increment))*scale;
			cvCircle(img, pt1 , 0, CV_RGB(255,255,255));
		}

		//--------------------------------------------------------------------------------------hough transform
		Line lines[100];
		int linecount;

		IplImage* bin = cvCreateImage(cvSize(image_size,image_size), IPL_DEPTH_8U, 1);
		cvCvtColor(img, bin, CV_BGR2GRAY);  //グレイスケール画像に変換

		CvMemStorage* storage = cvCreateMemStorage(0);
		CvSeq* cvLines = 0;

		cvLines = cvHoughLines2( bin, storage, CV_HOUGH_STANDARD, 1, 0.5*CV_PI/180, threshold, 0, 0 );

		linecount = cvLines->total;

		for( int i = 0; i < MIN(cvLines->total,100); i++ )
		{
			float* line = (float*)cvGetSeqElem(cvLines,i);
			float rho = line[0];
			float theta = line[1];

			double a = cos(theta), b = sin(theta);
			double x0 = a*rho, y0 = b*rho;

			CvPoint pt1, pt2;
			pt1.x = cvRound(x0 + 1000*(-b));
			pt1.y = cvRound(y0 + 1000*(a));
			pt2.x = cvRound(x0 - 1000*(-b));
			pt2.y = cvRound(y0 - 1000*(a));
			cvLine( img, pt1, pt2, CV_RGB(255,0,0), 1, 8 );


			double _a, _b, _c;	//linear ax+by+c=0
			//x*cos(theta) + y*sin(theta) - rho = 0
			_a = sin(theta);
			_b = cos(theta);
			_c = -rho;

			//the distance from center point to the line
			double dist_px = abs(_a * image_size/2 + _b * image_size/2 + _c) / sqrt( _a*_a + _b*_b );

			//距離がマイナス = 方向が逆
			if( _a * image_size/2 + _b * image_size/2 + _c < 0 ){
				theta -= CV_PI;
			}

			while(theta < -CV_PI){
				theta += 2*CV_PI;
			}
			while(theta > CV_PI){
				theta -= 2*CV_PI;
			}


			double dist = dist_px / scale;

			//ROS_INFO("Hough:rho=%f theta=%f[deg]",dist, theta_deg);

			geometry_msgs::Point p;
			p.x = dist*cos(-CV_PI/2+theta)+cos(theta)*10;
			p.y = dist*sin(-CV_PI/2+theta)+sin(theta)*10;
			p.z = 0;

			line_list.points.push_back(p);

			p.x = dist*cos(-CV_PI/2+theta)-cos(theta)*10;
			p.y = dist*sin(-CV_PI/2+theta)-sin(theta)*10;
			p.z = 0;
			line_list.points.push_back(p);

			lines[i].dist = dist;
			lines[i].theta = -theta;

			line_list.lifetime = ros::Duration();
		}

		//-------------------------------------------------------------------------------------- Calculate the location
		Line *front = NULL;
		Line *right = NULL;
		Line *left  = NULL;

		for(int i=0;i<linecount;i++){
			double line_yaw = lines[i].theta - yaw;
			if( line_yaw > -CV_PI*3/4 && line_yaw < -CV_PI/4 && front == NULL ){
				front = &(lines[i]);
			}
			else if( line_yaw > -CV_PI/4 && line_yaw < CV_PI/4 && left == NULL ){
				left = &(lines[i]);
			}
			else if( (line_yaw < -CV_PI*3/4 || line_yaw > CV_PI*3/4) && right == NULL ){
				right = &(lines[i]);
				//ROS_INFO("right[%f][%f]", right->theta, right->dist);
			}
		}
		if( front != NULL || right != NULL || left != NULL ){

			if( front != NULL ){
				yaw = front->theta + CV_PI/2;
			}
			else if( left  != NULL ){
				yaw =  left->theta;
			}
			else if( right != NULL ){
				if(right->theta >=0)	yaw = right->theta-CV_PI;
				else					yaw = right->theta+CV_PI;
			}

			float cr2 = cos(0*0.5);
			float cp2 = cos(0*0.5);
			float cy2 = cos(yaw*0.5);
			float sr2 = sin(0*0.5);
			float sp2 = sin(0*0.5);
			float sy2 = sin(yaw*0.5);

			pose.pose.orientation.w = cr2*cp2*cy2 + sr2*sp2*sy2;
			pose.pose.orientation.x = sr2*cp2*cy2 - cr2*sp2*sy2;
			pose.pose.orientation.y = cr2*sp2*cy2 + sr2*cp2*sy2;
			pose.pose.orientation.z = cr2*cp2*sy2 - sr2*sp2*cy2;

		}

		if(front!=NULL){
			pose.pose.position.y = -front->dist;
		}

		if(left!=NULL){
			pose.pose.position.x = -FIELD_WIDTH/2 + left->dist;
		}
		else if(right!=NULL){
			pose.pose.position.x = FIELD_WIDTH/2 - right->dist;
		}
#if false
		if(left!=NULL)	std::cout << "L";
		else			std::cout << " ";
		if(front!=NULL)	std::cout << "F";
		else			std::cout << " ";
		if(right!=NULL) std::cout << "R";
		else			std::cout << " ";
		std::cout << std::endl;
#endif
		//double yaw_deg = yaw*180/CV_PI;
		//ROS_INFO("yaw=%f[deg] x=%f[m] y=%f[m]",yaw_deg, pose.pose.position.x, pose.pose.position.y );

		pose.pose.position.x += DISTANCE_LRF_TO_CENTER*cos(-yaw-M_PI/2);
		pose.pose.position.y += DISTANCE_LRF_TO_CENTER*sin(-yaw-M_PI/2);

		pose.pose.position.y += FENCE_WIDTH / 2;

		//pose.pose.position.x += DISTANCE_LRF_TO_CENTER*cos(yaw+M_PI/2);
		//pose.pose.position.y -= DISTANCE_LRF_TO_CENTER*sin(yaw+M_PI/2);

		//-------------------------------------------------------------------------------------- Publish
		if( front!=NULL && (left!=NULL || right!=NULL) ) pose_pub.publish(pose);
		publisher.publish(line_list);

		sensor_msgs::ImagePtr img_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", cv::cvarrToMat(img)).toImageMsg();
		image_pub.publish(img_msg);

		//-------------------------------------------------------------------------------------- Release
		cvReleaseMemStorage(&storage);
		cvReleaseImage(&bin);
		cvReleaseImage(&img);
	}

}
