/*
 * pcl_kinect2.h
 *
 *  Created on: 2015/03/19
 *      Author: fortefibre
 */

#ifndef PCL_KINECT2_H_
#define PCL_KINECT2_H_

#include <opencv2/opencv.hpp>

#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

class KinectV2{
protected:
	float kinect_rad;
	cv::Mat lookupX, lookupY;

public:
	double offset_x, offset_y, offset_z;
	geometry_msgs::Pose robot;

	KinectV2(){
		setKinectRad(0.0f*M_PI/180);
		offset_x = offset_y = offset_z = 0.0f;

		createLookup(512,424);
	}

	void setKinectRad(float rad){
		kinect_rad = rad;
	}

	void createLookup(size_t width, size_t height)
	{
		const float fx = 0.0027697133333333;
		const float fy = -0.00271;
		const float cx = 256;
		const float cy = 214;
		float *it;

		lookupY = cv::Mat(1, height, CV_32F);
		it = lookupY.ptr<float>();
		for(size_t r = 0; r < height; ++r, ++it)
		{
			*it = (r - cy) * fy;
		}

		lookupX = cv::Mat(1, width, CV_32F);
		it = lookupX.ptr<float>();
		for(size_t c = 0; c < width; ++c, ++it)
		{
			*it = (c - cx) * fx;
		}
	}

	void createCloud(const cv::Mat &depth, pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud)
	{
		const float badPoint = std::numeric_limits<float>::quiet_NaN();

		#pragma omp parallel for
		for(int r = 0; r < depth.rows; ++r)
		{
			pcl::PointXYZ *itP = &cloud->points[r * depth.cols];
			const uint16_t *itD = depth.ptr<uint16_t>(r);
			const float y = lookupY.at<float>(0, r);
			const float *itX = lookupX.ptr<float>();

			for(size_t c = 0; c < (size_t)depth.cols; ++c, ++itP, ++itD, ++itX)
			{
				register const float depthValue = *itD / 1000.0f;
				// Check for invalid measurements
				if(isnan(depthValue) || depthValue <= 0.001)
				{
					// not valid
					itP->x = itP->y = itP->z = badPoint;
					continue;
				}
				itP->x = *itX * depthValue;
				itP->y = depthValue;
				itP->z = y * depthValue;
			}
		}
	}

	void getTransformMatrixToGlobalFrame( Eigen::Affine3f &matrix ){

		Eigen::Matrix4f r,t,y,p;

		const float cos_theta = cos( kinect_rad );
		const float sin_theta = sin( kinect_rad );
		r << \
				1,         0,           0, 0, \
				0, cos_theta, - sin_theta, 0, \
				0, sin_theta,   cos_theta, 0, \
				0,          0,          0, 1;

		t << \
				1, 0, 0, offset_x, \
				0, 1, 0, offset_y, \
				0, 0, 1, offset_z, \
				0, 0, 0, 1;

		const float yaw = -atan2(2.0*(robot.orientation.x*robot.orientation.y + robot.orientation.w*robot.orientation.z), robot.orientation.w*robot.orientation.w + robot.orientation.x*robot.orientation.x - robot.orientation.y*robot.orientation.y - robot.orientation.z*robot.orientation.z);
		const float cos_yaw = cos( -yaw );
		const float sin_yaw = sin( -yaw );

		y << \
				cos_yaw, 	-sin_yaw, 	0, 0, \
				sin_yaw, 	cos_yaw, 	0, 0, \
				0, 			0, 			1, 0, \
				0, 			0, 			0, 1;


		p << \
				1, 0, 0, robot.position.x, \
				0, 1, 0, robot.position.y, \
				0, 0, 1, robot.position.z, \
				0, 0, 0, 1;

		//ROS_ERROR("%.3f %.3f %.3f", robot.position.x, robot.position.y, robot.position.z);

		matrix = p*y*t*r;

	}
};


#endif /* PCL_KINECT2_H_ */
