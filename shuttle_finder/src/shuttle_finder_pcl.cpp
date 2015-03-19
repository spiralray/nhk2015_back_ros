#include <ros/ros.h>

#include <opencv2/opencv.hpp>
#include <cv.h>
#include <highgui.h>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <visualization_msgs/Marker.h>	//for displaying points of a shuttle
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
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

#include <pcl/visualization/cloud_viewer.h>

#include <pcl/common/transforms.h>

#include "pcl_kinect2.h"


class PCSize
{
public:
    double length;///x方向
    double width;///y方向
    double height;///z方向
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

PCSize max_min(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)//点群の各XYZ方向の最大値と最小値を求める
{
    double min_x, min_y, min_z, max_x, max_y, max_z;
    PCSize size;
    min_x = 10000;
    min_y = 10000;
    min_z = 100000;
    max_x = -10000;
    max_y = -10000;
    max_z = -10000;

    for(int i = 0; i < cloud->points.size(); i++)
    {
        if(max_x < cloud->points[i].x)
            max_x = cloud->points[i].x;
        if(max_y < cloud->points[i].y)
            max_y = cloud->points[i].y;
        if(max_z < cloud->points[i].z)
            max_z = cloud->points[i].z;

        if(min_x > cloud->points[i].x)
            min_x = cloud->points[i].x;
        if(min_y > cloud->points[i].y)
            min_y = cloud->points[i].y;
        if(min_z > cloud->points[i].z)
            min_z = cloud->points[i].z;
    }

    size.length = abs(max_x) + abs(min_x);
    size.width = abs(max_y) + abs(min_y);
    size.height = abs(max_z) + abs(min_y);
    return size;
}

void Clustering(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
    PCSize size;
    size = max_min(cloud);
    double min = size.height;
    if(min > size.width)
        min = size.width;
    if(min > size.length)
        min = size.length;
    if(min > size.height)
        min = size.height;

    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud (cloud);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance (min / 10);
    ec.setMinClusterSize (10);//最小のクラスターの値を設定
    ec.setMaxClusterSize (25000);//最大のクラスターの値を設定
    ec.setSearchMethod (tree);//検索に使用する手法を指定
    ec.setInputCloud (cloud);//点群を入力
    ec.extract (cluster_indices);//クラスター情報を出力

    std::string name;
    name = "cluster";

    int j = 0;
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)//クラスターを1塊ごと出力
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
        for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++)
            cloud_cluster->points.push_back (cloud->points[*pit]); //*

        cloud_cluster->width = cloud_cluster->points.size ();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;

        std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << std::endl;
        std::stringstream ss;
        ss << name << "_p_" << j << ".pcd";//クラスター毎に名前を変化
        pcl::io::savePCDFileBinary(ss.str(), *cloud_cluster);
        j++;
    }
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


  image_transport::ImageTransport it(nh);
  image_transport::Subscriber sub = it.subscribe("/kinect/depth", 1, imageCallback);

  ros::Subscriber subPose = nh.subscribe("/robot/pose", 10, poseCallback);
  ros::Subscriber subAngle = nh.subscribe("/kinect/angle", 10, servoCallback);

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

	ros::NodeHandle n;
	ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("/shuttle/marker", 1);
	ros::Publisher shuttle_pub = n.advertise<geometry_msgs::PointStamped>("/shuttle/point", 10);

	ros::Time timestamp = ros::Time::now();

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
	cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());
	cloud->height = 424;
	cloud->width = 512;
	cloud->is_dense = false;
	cloud->points.resize(512 * 424);

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);

	const std::string cloudName = "rendered";
	pcl::visualization::PCLVisualizer::Ptr visualizer(new pcl::visualization::PCLVisualizer("Cloud Viewer"));
	visualizer->addPointCloud(cloud, cloudName);
	visualizer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, cloudName);
	visualizer->initCameraParameters();
	visualizer->setBackgroundColor(0, 0, 0);
	visualizer->setPosition(0, 0);
	visualizer->setSize(512, 424);
	visualizer->setShowFPS(true);
	visualizer->setCameraPosition(0, 0, 0, 0, -1, 0);

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
			//ros::Duration(0.001).sleep();
			visualizer->spinOnce(1);
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

		points.header.stamp = timestamp;

		geometry_msgs::PointStamped shuttle;
		shuttle.header.stamp = timestamp;
		shuttle.header.frame_id = "laser";

		kinect.createCloud(depth, cloud);

		// Down sampling
		pcl::VoxelGrid<pcl::PointXYZ> sorVoxel;
		sorVoxel.setInputCloud (cloud);
		sorVoxel.setLeafSize (0.01f, 0.01f, 0.01f);
		sorVoxel.filter (*cloud_filtered);

		// Remove too near points
		pcl::PassThrough<pcl::PointXYZ> pass;
		pass.setInputCloud (cloud_filtered);
		pass.setFilterFieldName ("z");
		pass.setFilterLimits (1.0, 10.0);
		//pass.setFilterLimitsNegative (true);
		pass.filter (*cloud_filtered);

		//Remove noise
		pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
		sor.setInputCloud (cloud_filtered);
		sor.setMeanK (50);
		sor.setStddevMulThresh (0.04);
		sor.filter (*cloud_filtered);

		Eigen::Affine3f matrix;
		kinect.getTransformMatrixToGlobalFrame(matrix);

		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_global (new pcl::PointCloud<pcl::PointXYZ>);
		pcl::transformPointCloud( *cloud_filtered, *cloud_global, matrix );

		visualizer->updatePointCloud(cloud_global, cloudName);
		visualizer->spinOnce(1);

		//Clustering(cloud);

	}
	visualizer->close();
}
