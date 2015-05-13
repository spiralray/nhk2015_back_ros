#include <ros/ros.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>
#include <pcl_ros/point_cloud.h>

#include <laser_geometry/laser_geometry.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>

#include <geometry_msgs/PoseStamped.h>

#define DISTANCE_LRF_TO_CENTER	0.455	//[m]

ros::Publisher pcl_pub;

laser_geometry::LaserProjection projector_;
geometry_msgs::Pose pose;

bool backward;

void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg){
	pose = msg->pose;
}

void getTransformMatrixToGlobalFrame( Eigen::Affine3f &matrix ){

	Eigen::Matrix4f r,t,y,p;

	const float cos_theta = cos( M_PI/2 );
	const float sin_theta = sin( M_PI/2 );

	r << \
				0, 1, 0, 0, \
				1, 0, 0, 0, \
				0, 0, 1, 0, \
				0, 0, 0, 1;

	t << \
			1, 0, 0, 0, \
			0, 1, 0, DISTANCE_LRF_TO_CENTER, \
			0, 0, 1, 0, \
			0, 0, 0, 1;

	float yaw = -atan2(2.0*(pose.orientation.x*pose.orientation.y + pose.orientation.w*pose.orientation.z), pose.orientation.w*pose.orientation.w + pose.orientation.x*pose.orientation.x - pose.orientation.y*pose.orientation.y - pose.orientation.z*pose.orientation.z);
	if( backward ) yaw += M_PI;
	const float cos_yaw = cos( -yaw );
	const float sin_yaw = sin( -yaw );

	y << \
			cos_yaw, 	-sin_yaw, 	0, 0, \
			sin_yaw, 	cos_yaw, 	0, 0, \
			0, 			0, 			1, 0, \
			0, 			0, 			0, 1;


	p << \
			1, 0, 0, pose.position.x, \
			0, 1, 0, pose.position.y, \
			0, 0, 1, pose.position.z, \
			0, 0, 0, 1;

	//ROS_ERROR("%.3f %.3f %.3f", pose.position.x, pose.position.y, pose.position.z);

	matrix = p*y*t*r;

}

void scanCallback (const sensor_msgs::LaserScan::ConstPtr& scan_in)
{
	sensor_msgs::PointCloud2 cloudmsg;
	projector_.projectLaser(*scan_in, cloudmsg);

	pcl::PointCloud<pcl::PointXYZ> cloud;
	pcl::fromROSMsg (cloudmsg, cloud);

	Eigen::Affine3f matrix;
	getTransformMatrixToGlobalFrame(matrix);
	pcl::transformPointCloud( cloud, cloud, matrix );

	pcl::toROSMsg (cloud, cloudmsg);
	cloudmsg.header.stamp = scan_in->header.stamp;
	cloudmsg.header.frame_id = "map";
	pcl_pub.publish(cloudmsg);

}

int main(int argc, char** argv){

	ros::init(argc, argv, "laserdisp");
	ros::NodeHandle n;

	if (!n.hasParam("/laser2location/backward")){
		ROS_INFO("Parameter /laser2location/backward is not defined.");
		return -1;
	}

	if (!n.getParam("/laser2location/backward", backward)){
		ROS_ERROR("parameter backward is invalid.");
		return -1;
	}

	ros::Subscriber subscriber = n.subscribe("scan", 100, scanCallback);
	ros::Subscriber subPose = n.subscribe("/robot/pose", 10, poseCallback);

	pcl_pub = n.advertise< sensor_msgs::PointCloud2 >("pcllaser", 1);

	ros::spin();

	return 0;
}
