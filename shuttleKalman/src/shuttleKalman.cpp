#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <visualization_msgs/Marker.h>

#include <cv.h>
#include <highgui.h>
#include <ctype.h>
#include <math.h>
#include <opencv2/legacy/legacy.hpp>

#include <pthread.h>

//http://codereview.stackexchange.com/questions/8611/linux-c-timer-class-how-can-i-improve-the-accuracy
#include "TimerManager.h"
#include <iostream>
#include <fstream>
#include <sys/time.h>

#include <boost/random.hpp>

namespace ShuttleConst{
const double TIME_CALCULATE = 1.0;	//[s]
const double resist_coeff = 0.001075;//resistance coefficent of air [N/(m/s)^2]
const double dt = 0.005;			//[s]
const double gravity = 9.812;	//[m/s^2]
const double mass = 0.005;		//[kg]
}

using namespace ShuttleConst;


float gen_random_float(float min, float max)
{
    boost::mt19937 rng;
    boost::uniform_real<float> u(min, max);
    boost::variate_generator<boost::mt19937&, boost::uniform_real<float> > gen(rng, u);
    return gen();
}

class Shuttle{
public:

	ros::NodeHandle nh;

	ros::Publisher marker_pub;
	visualization_msgs::Marker shuttle_line;

	ros::Time timeLastUpdate;

	geometry_msgs::Pose oldPose;

	CvKalman* kalman;

	CvKalman* kalmanOrbit;

	int updated;

	Shuttle(){

#define MP	3	/* number of dimensions of observation vectors */
#define DP	9	/* number of dimensions of state vectors */
#define CP	1	/* number of dimensions of control vectors */

		updated = 0;
		timeLastUpdate = ros::Time::now();

		kalman = cvCreateKalman( DP, MP, CP );
		kalmanOrbit = cvCreateKalman( DP, MP, CP );;

		//cvSetIdentity( kalman->measurement_matrix, cvRealScalar(1) );
		cvSetIdentity( kalman->process_noise_cov, cvRealScalar(1e-5) );
		cvSetIdentity( kalman->measurement_noise_cov, cvRealScalar(1e-8) );
		cvSetIdentity( kalman->error_cov_post, cvRealScalar(1));


		float A[DP * DP];

		for( int i=0 ; i < DP * DP ; i++ ){
			A[i]  = 0.0;
		}

		A[0] = 1.0f;
		A[DP+1] = 1.0f;
		A[(DP+1)*2] = 1.0f;

		A[(DP+1)*3] = 1.0f;
		A[(DP+1)*4] = 1.0f;
		A[(DP+1)*5] = 1.0f;

		//A[(DP+1)*3 - DP*3] = dt;
		//A[(DP+1)*4 - DP*3] = dt;
		//A[(DP+1)*5 - DP*3] = dt;

		//A[(DP+1)*6 - DP*3] = dt;
		//A[(DP+1)*7 - DP*3] = dt;
		//A[(DP+1)*8 - DP*3] = dt;

		//A[(DP+1)*6 - 3] = c/m * dt;
		//A[(DP+1)*7 - 3] = c/m * dt;
		//A[(DP+1)*8 - 3] = c/m * dt;

		memcpy( kalman->transition_matrix->data.fl, A, sizeof(A));


		cvZero( kalman->measurement_matrix );
		kalman->measurement_matrix->data.fl[0] = 1.0f;
		kalman->measurement_matrix->data.fl[DP+1] = 1.0f;
		kalman->measurement_matrix->data.fl[(DP+1)*2] = 1.0f;

		cvZero( kalman->control_matrix );
		kalman->control_matrix->data.fl[ DP - 1 ] = -gravity;



		marker_pub = nh.advertise<visualization_msgs::Marker>("shuttle_kalman", 10);

		shuttle_line.header.frame_id = "/laser";
		shuttle_line.header.stamp = ros::Time::now();
		shuttle_line.ns = "shuttle_line";
		shuttle_line.id = 0;
		shuttle_line.type = visualization_msgs::Marker::LINE_LIST;
		shuttle_line.action = visualization_msgs::Marker::ADD;

		shuttle_line.scale.x = 0.03;

		shuttle_line.color.b = 1.0;
		shuttle_line.color.a = 1.0;
	}

	~Shuttle(){
		cvReleaseKalman(&kalman);
	}

	void update(const ros::Time time, const geometry_msgs::Pose& pose){

		double sec = time.toSec() - timeLastUpdate.toSec();

		if( sec > 0.3 || updated == 0 ){
			//Resampling
			updated = 1;

			float A[DP] = { pose.position.x, pose.position.y, pose.position.z, 0, 0, 0, 0, 0, 0 };
			memcpy( kalman->state_post->data.fl, A, sizeof(A));
		}
		else if( updated == 1 ){
			updated = 2;

			float A[DP] = { pose.position.x, pose.position.y, pose.position.z,
					-(pose.position.x - kalman->transition_matrix->data.fl[0])/sec,
					-(pose.position.y - kalman->transition_matrix->data.fl[1])/sec,
					-(pose.position.z - kalman->transition_matrix->data.fl[2])/sec,
					0, 0, 0 };
			memcpy( kalman->state_post->data.fl, A, sizeof(A));
		}
		else{
			updated = 3;


			float A[DP * DP];

			for( int i=0 ; i < DP * DP ; i++ ){
				A[i]  = 0.0;
			}

			A[0] = 1.0f;
			A[DP+1] = 1.0f;
			A[(DP+1)*2] = 1.0f;

			A[(DP+1)*3] = 1.0f;
			A[(DP+1)*4] = 1.0f;
			A[(DP+1)*5] = 1.0f;

			A[(DP+1)*3 - DP*3] = sec;
			A[(DP+1)*4 - DP*3] = sec;
			A[(DP+1)*5 - DP*3] = sec;

			A[(DP+1)*6 - DP*3] = sec;
			A[(DP+1)*7 - DP*3] = sec;
			A[(DP+1)*8 - DP*3] = sec;

			double v = sqrt( kalman->transition_matrix->data.fl[3]*kalman->transition_matrix->data.fl[3]+ kalman->transition_matrix->data.fl[4]*kalman->transition_matrix->data.fl[4] + kalman->transition_matrix->data.fl[5]*kalman->transition_matrix->data.fl[5]
			);	//speed of the shuttle

			double R = resist_coeff * v*v; //Air resistance

			A[(DP+1)*6 - 3] = R/mass * sec;
			A[(DP+1)*7 - 3] = R/mass * sec;
			A[(DP+1)*8 - 3] = R/mass * sec;

			memcpy( kalman->transition_matrix->data.fl, A, sizeof(A));

			predict( sec );


			float m[MP] = {pose.position.x, pose.position.y ,pose.position.z };
			CvMat measurement = cvMat(MP, 1, CV_32FC1, m);
			const CvMat *correction = cvKalmanCorrect(kalman, &measurement);

		}

		oldPose = pose;
		timeLastUpdate = time;

	}
	void predict(double time){

		if( updated >= 3  ){

			float c[CP] = { time };
			CvMat control = cvMat(CP, 1, CV_32FC1, c);

			const CvMat *prediction = cvKalmanPredict( kalman, &control );
		}
	}

	void calc( float *state){

		//init
		shuttle_line.points.clear();
		shuttle_line.header.stamp = timeLastUpdate;

		geometry_msgs::Point _last_point;
		geometry_msgs::Point _point;

		memcpy( kalmanOrbit->state_pre->data.fl, kalman->state_pre->data.fl, sizeof(float)*DP  );
		memcpy( kalmanOrbit->state_post->data.fl, kalman->state_post->data.fl, sizeof(float)*DP  );
		memcpy( kalmanOrbit->process_noise_cov->data.fl, kalman->process_noise_cov->data.fl, sizeof(float)*DP*DP  );
		memcpy( kalmanOrbit->measurement_matrix->data.fl, kalman->measurement_matrix->data.fl, sizeof(float)*MP*DP  );
		memcpy( kalmanOrbit->measurement_noise_cov->data.fl, kalman->measurement_noise_cov->data.fl, sizeof(float)*MP*MP  );
		memcpy( kalmanOrbit->error_cov_pre->data.fl, kalman->error_cov_pre->data.fl, sizeof(float)*DP*DP  );
		memcpy( kalmanOrbit->error_cov_post->data.fl, kalman->error_cov_post->data.fl, sizeof(float)*DP*DP  );
		memcpy( kalmanOrbit->gain->data.fl, kalman->gain->data.fl, sizeof(float)*DP*MP  );
		memcpy( kalmanOrbit->control_matrix->data.fl, kalman->control_matrix->data.fl, sizeof(float)*DP*CP  );


		float A[DP * DP];

		for( int i=0 ; i < DP * DP ; i++ ){
			A[i]  = 0.0;
		}

		A[0] = 1.0f;
		A[DP+1] = 1.0f;
		A[(DP+1)*2] = 1.0f;

		A[(DP+1)*3] = 1.0f;
		A[(DP+1)*4] = 1.0f;
		A[(DP+1)*5] = 1.0f;

		A[(DP+1)*3 - DP*3] = dt;
		A[(DP+1)*4 - DP*3] = dt;
		A[(DP+1)*5 - DP*3] = dt;

		A[(DP+1)*6 - DP*3] = dt;
		A[(DP+1)*7 - DP*3] = dt;
		A[(DP+1)*8 - DP*3] = dt;

		_last_point.x = kalmanOrbit->state_post->data.fl[0];
		_last_point.y = kalmanOrbit->state_post->data.fl[1];
		_last_point.z = kalmanOrbit->state_post->data.fl[2];


		for(double i=0; i < TIME_CALCULATE ; i+=dt){

			double v = sqrt( kalmanOrbit->transition_matrix->data.fl[3]*kalmanOrbit->transition_matrix->data.fl[3]+ kalmanOrbit->transition_matrix->data.fl[4]*kalmanOrbit->transition_matrix->data.fl[4] + kalmanOrbit->transition_matrix->data.fl[5]*kalmanOrbit->transition_matrix->data.fl[5]
			);	//speed of the shuttle

			double R = resist_coeff * v*v; //Air resistance

			A[(DP+1)*6 - 3] = R/mass * dt;
			A[(DP+1)*7 - 3] = R/mass * dt;
			A[(DP+1)*8 - 3] = R/mass * dt;

			memcpy( kalmanOrbit->transition_matrix->data.fl, A, sizeof(A));

			float c[CP] = { dt };
			CvMat control = cvMat(CP, 1, CV_32FC1, c);
			const CvMat *prediction = cvKalmanPredict( kalmanOrbit, &control );

			_point.x = kalmanOrbit->state_post->data.fl[0];
			_point.y = kalmanOrbit->state_post->data.fl[1];
			_point.z = kalmanOrbit->state_post->data.fl[2];

			shuttle_line.points.push_back(_last_point);
			shuttle_line.points.push_back(_point);
			_last_point = _point;
			if( _point.z <= 0.0f ) break;
		}
	}

	float *getNowState(){
		return kalman->state_post->data.fl;
	}

	void publish(){
		marker_pub.publish(shuttle_line);

	}

};

Shuttle *shuttle;


void pointsCallback(const geometry_msgs::PoseArray& posearray)
{
	if( !posearray.poses.empty() ){
		//ROS_INFO("update.");

		shuttle->update(posearray.header.stamp,posearray.poses.at(0));

		float *nowState = shuttle->getNowState();
		shuttle->calc(nowState);

		shuttle->publish();

	}
}

int main (int argc, char **argv)
{
	ros::init(argc, argv, "shuttle_listener");

	shuttle = new Shuttle();

	ros::Subscriber subscriber = shuttle->nh.subscribe("shuttle_points", 1, pointsCallback);

	ros::spin();

	delete shuttle;

	return 0;
}
