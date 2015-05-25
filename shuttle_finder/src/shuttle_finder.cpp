#include <ros/ros.h>

#include <opencv2/opencv.hpp>
#include <cv.h>
#include <highgui.h>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Float32.h>

#include <pthread.h>
#include <math.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>

#include <pcl/features/normal_3d.h>

#include <pcl/kdtree/kdtree.h>

#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>

#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

#include <pcl/common/transforms.h>

#include "pcl_ros/point_cloud.h"

#include "pcl_kinect2.h"

//#define OPENCV_DEBUG

class PCSize
{
public:
    float length;///x方向
    float width;///y方向
    float height;///z方向
};


using namespace cv;


pthread_mutex_t	mutex;  // MUTEX
Mat depth_frame;
ros::Time depth_timestamp;
bool recieved = false;
bool endflag = false;

pthread_mutex_t	pose_mutex;  // MUTEX
geometry_msgs::Pose _pose;

float kinect_pitch=0.0f;

bool debug = false;

KinectV2 kinect;

double hight_low, hight_high;

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
	if( !debug ){
		depth_timestamp = cv_ptr_depth->header.stamp;
	}
	else{
		depth_timestamp = ros::Time::now();
	}
	recieved = true;
	pthread_mutex_unlock( &mutex );

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

bool searchShuttle( pcl::PointCloud<pcl::PointXYZ>::Ptr cloud , geometry_msgs::PointStamped &shuttle ){

	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
	tree->setInputCloud (cloud);
	std::vector<pcl::PointIndices> cluster_indices;
	pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;

	ec.setClusterTolerance (0.2);

	ec.setMinClusterSize (10);
	ec.setMaxClusterSize (150);
	ec.setSearchMethod (tree);
	ec.setInputCloud (cloud);
	ec.extract (cluster_indices);


	int count;
	double gx,gy,gz;

	//for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end(); ++it){
	std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin ();
	if( it == cluster_indices.end() ) return false;

	count = 0;
	gx = gy = gz = 0.0;

	for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++) {

		gx += cloud->points[*pit].x;
		gy += cloud->points[*pit].y;
		gz += cloud->points[*pit].z;
		count++;
	}

	gx = gx/count;
	gy = gy/count;
	gz = gz/count;

	shuttle.point.x = gx;
	shuttle.point.y = gy;
	shuttle.point.z = gz;

	return true;
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

  if (!local_nh.hasParam("debug")){
	  debug = false;
  }
  else{
	  if (!local_nh.getParam("debug", debug)){
		  ROS_ERROR("parameter debug is invalid.");
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

  if (!local_nh.hasParam("hight_low")){
	  hight_low = 2.0f;
  }
  else{
	  if (!local_nh.getParam("hight_low", hight_low)){
		  ROS_ERROR("parameter hight_low is invalid.");
		  return -1;
	  }
  }

  if (!local_nh.hasParam("hight_high")){
	  hight_high = 10.0f;
  }
  else{
	  if (!local_nh.getParam("hight_high", hight_high)){
		  ROS_ERROR("parameter hight_high is invalid.");
		  return -1;
	  }
  }

  ROS_INFO("offset_x=%.3f, offset_y=%.3f, offset_z=%.3f",kinect.offset_x, kinect.offset_y, kinect.offset_z);


  image_transport::ImageTransport it(nh);
  image_transport::Subscriber sub = it.subscribe("/kinect/depth", 1, imageCallback);

  ros::Subscriber subPose = nh.subscribe("/robot/pose", 10, poseCallback);
  ros::Subscriber subAngle = nh.subscribe("/kinect/angle", 10, servoCallback);

#ifdef OPENCV_DEBUG
  cv::startWindowThread();
#endif
  pthread_t thread;
  pthread_create( &thread, NULL, (void* (*)(void*))thread_main, NULL );

  ros::spin();

  endflag = true;
  pthread_join( thread, NULL );

#ifdef OPENCV_DEBUG
  destroyAllWindows();
#endif

  return 0;
}

void thread_main(){
	ROS_INFO("New thread Created.");
	pthread_detach( pthread_self( ));

	ros::NodeHandle n;
	ros::Publisher shuttle_pub = n.advertise<geometry_msgs::PointStamped>("/shuttle/point", 10);

	ros::Publisher pcl_pub;

	if( debug ){
		pcl_pub = n.advertise< sensor_msgs::PointCloud2 >("pclglobal", 1);
	}
	ros::Time timestamp = ros::Time::now();

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
	cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());
	cloud->height = 424;
	cloud->width = 512;
	cloud->is_dense = false;
	cloud->points.resize(512 * 424);

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);

#ifdef OPENCV_DEBUG
  namedWindow( "frame", WINDOW_AUTOSIZE );
#endif

	while(!endflag){

		bool shuttle_found = false;

		//wait for recieve new frame
		while(recieved == false){
			ros::Duration(0.001).sleep();
		}

		cv::Mat depth;

		//Get new frame

		pthread_mutex_lock( &pose_mutex );
		pthread_mutex_lock( &mutex );
		kinect.robot = _pose;
		pthread_mutex_unlock( &pose_mutex );

		if( timestamp == depth_timestamp){
			pthread_mutex_unlock( &mutex );
			continue;
		}

		depth_frame.copyTo(depth);
		timestamp = depth_timestamp;

		recieved = false;

		pthread_mutex_unlock( &mutex );

		geometry_msgs::PointStamped shuttle;
		shuttle.header.stamp = timestamp;
		shuttle.header.frame_id = "map";

#if 0
		cv::erode(depth, depth, cv::Mat() );
		cv::dilate(depth, depth, cv::Mat());

#ifdef OPENCV_DEBUG
	cv::imshow("frame", depth);
#endif

		kinect.createCloud(depth, cloud);
#else
		cv::Mat depthMask(depth);
		depthMask.convertTo(depthMask, CV_8U, 255.0 / 8000.0);

		cv::threshold(depthMask, depthMask, 1 , 255, cv::THRESH_BINARY);

		cv::erode(depthMask, depthMask, cv::Mat() );
		cv::dilate(depthMask, depthMask, cv::Mat());

		Mat dst;
		depth.copyTo(dst, depthMask);

#ifdef OPENCV_DEBUG
		cv::imshow("frame", dst);
#endif

		kinect.createCloud(dst, cloud);
#endif

		if( cloud->points.empty() ) continue;
		// Down sampling
		pcl::VoxelGrid<pcl::PointXYZ> sorVoxel;
		sorVoxel.setInputCloud (cloud);
		sorVoxel.setLeafSize (0.01f, 0.01f, 0.01f);
		sorVoxel.filter (*cloud_filtered);

#if 1
		if( cloud_filtered->points.empty() ) continue;
		//Remove noise
		pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
		sor.setInputCloud (cloud_filtered);
		sor.setMeanK (8);
		sor.setStddevMulThresh (1.0);
		sor.filter (*cloud_filtered);
#endif

		if( cloud_filtered->points.empty() ) continue;
		// Remove too near points
		pcl::PassThrough<pcl::PointXYZ> pass;
		pass.setInputCloud (cloud_filtered);
		pass.setFilterFieldName ("y");
		pass.setFilterLimits (1.0, 10.0);
		//pass.setFilterLimitsNegative (true);
		pass.filter (*cloud_filtered);

		Eigen::Affine3f matrix;
		kinect.getTransformMatrixToGlobalFrame(matrix);

		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_global (new pcl::PointCloud<pcl::PointXYZ>);
		pcl::transformPointCloud( *cloud_filtered, *cloud_global, matrix );

		if( cloud_global->points.empty() ) continue;
		pass.setInputCloud (cloud_global);
		pass.setFilterFieldName ("x");
		pass.setFilterLimits (-3.5, 3.5);
		pass.filter (*cloud_global);
#if 1
		if( debug ){
			sensor_msgs::PointCloud2 cloudmsg;
			pcl::toROSMsg (*cloud_global, cloudmsg);
			cloudmsg.header.stamp = timestamp;
			cloudmsg.header.frame_id = "map";
			pcl_pub.publish(cloudmsg);
		}
#endif
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_my_field (new pcl::PointCloud<pcl::PointXYZ>);
		pcl::copyPointCloud<pcl::PointXYZ>(*cloud_global, *cloud_my_field);

		if( cloud_global->points.empty() ) goto search_my_field;
		pass.setInputCloud (cloud_global);
		pass.setFilterFieldName ("z");
		pass.setFilterLimits (hight_low, hight_high);
		pass.filter (*cloud_global);

		if( cloud_global->points.empty() ) goto search_my_field;
		//ROS_ERROR("%d", cloud_global->points.size());

		shuttle_found = searchShuttle(cloud_global, shuttle);
		if( shuttle_found ){
			shuttle_pub.publish(shuttle);
			continue;
		}

		search_my_field:

		pass.setInputCloud (cloud_my_field);
		pass.setFilterFieldName ("y");
		pass.setFilterLimits (-6, -0.3);
		pass.filter (*cloud_my_field);

		pass.setInputCloud (cloud_my_field);
		pass.setFilterFieldName ("z");
		pass.setFilterLimits (1.3, hight_low+0.1);
		pass.filter (*cloud_my_field);

		if( cloud_my_field->points.empty() ) continue;
		shuttle_found = searchShuttle(cloud_my_field, shuttle);
		if( shuttle_found ){
			shuttle_pub.publish(shuttle);
		}


	}
}
