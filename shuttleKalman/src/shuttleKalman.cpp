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
const double mass = 0.00467;		//[kg]

const int freq = 30;

const int n_stat = 6;
const int n_particle = 5000;
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
	visualization_msgs::Marker shuttle_particle;
	visualization_msgs::Marker shuttle_line;

	CvConDensation *cond = 0;
	CvMat *lowerBound = 0;
	CvMat *upperBound = 0;

	ros::Time timeLastUpdate;

	geometry_msgs::Pose oldPose;

	CvKalman* kalman;
	CvMat* state;
	CvMat* process_noise;
	CvMat* measurement;
	CvRandState rng;

	int updated;

	Shuttle(){

#define MP	9	/* number of dimensions of observation vectors */
#define DP	9	/* number of dimensions of state vectors */
#define CP	9	/* number of dimensions of control vectors */

		updated = 0;
		timeLastUpdate = ros::Time::now();

		kalman = cvCreateKalman( DP, MP, CP );

		state = cvCreateMat( DP, 1, CV_32FC1 );
		process_noise = cvCreateMat( DP, 1, CV_32FC1 );
		measurement = cvCreateMat( MP, 1, CV_32FC1 );

		cvZero( measurement );

		cvRandSetRange( &rng, 0, 0.1, 0 );
		rng.disttype = CV_RAND_NORMAL;

		cvRand( &rng, state );

		//cvSetIdentity( kalman->measurement_matrix, cvRealScalar(1) );
		cvSetIdentity( kalman->process_noise_cov, cvRealScalar(1e-5) );
		cvSetIdentity( kalman->measurement_noise_cov, cvRealScalar(1e-4) );
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
		//kalman->control_matrix->data.fl[ DP*DP - 1 ] = gravity * dt;



		marker_pub = nh.advertise<visualization_msgs::Marker>("shuttle_particles", 10);

		shuttle_particle.header.frame_id = "/laser";
		shuttle_particle.header.stamp = ros::Time::now();
		shuttle_particle.ns = "shuttle_particle";
		shuttle_particle.id = 0;
		shuttle_particle.type = visualization_msgs::Marker::POINTS;
		shuttle_particle.action = visualization_msgs::Marker::ADD;

		shuttle_particle.scale.x = 0.03;
		shuttle_particle.scale.y = 0.03;
		shuttle_particle.scale.z = 0.03;

		shuttle_particle.color.g = 0.5;
		shuttle_particle.color.a = 1.0;

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
		cvReleaseConDensation (&cond);
		cvReleaseMat (&lowerBound);
		cvReleaseMat (&upperBound);
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
					-(pose.position.x - kalman->transition_matrix->data.fl[0]),
					-(pose.position.y - kalman->transition_matrix->data.fl[1]),
					-(pose.position.z - kalman->transition_matrix->data.fl[2]),
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

			cvZero( kalman->control_matrix );
			//kalman->control_matrix->data.fl[ DP*DP - 1 ] = gravity * sec;
			kalman->control_matrix->data.fl[ DP*DP - 1 ] = gravity;

			predict();


			float m[MP] = {pose.position.x, pose.position.y ,pose.position.z ,0,0,0,0,0,0 };
			CvMat measurement = cvMat(MP, 1, CV_32FC1, m);
			const CvMat *correction = cvKalmanCorrect(kalman, &measurement);

		}

		oldPose = pose;
		timeLastUpdate = time;

	}
	void predict(){

		if( updated >= 3  ){
			const CvMat *prediction = cvKalmanPredict(kalman);
		}
	}

	void calc( float *state){

		//init
		shuttle_line.points.clear();
		shuttle_line.header.stamp = timeLastUpdate;

		geometry_msgs::Point _last_point;
		geometry_msgs::Point _last_speed;


		_last_point.x = state[0];
		_last_point.y = state[1];
		_last_point.z = state[2];
		_last_speed.x = state[3];
		_last_speed.y = state[4];
		_last_speed.z = state[5];

		geometry_msgs::Point _point, _speed;

		for(double i=0; i < TIME_CALCULATE ; i+=dt){

			double v = sqrt( _last_speed.x*_last_speed.x + _last_speed.y*_last_speed.y + _last_speed.z*_last_speed.z );	//speed of a shuttle

			double R = resist_coeff * v*v; //Air resistance
			double Rx = -(_last_speed.x/v)*R;
			double Ry = -(_last_speed.y/v)*R;
			double Rz = -(_last_speed.z/v)*R;

			double Fx = Rx;	//force
			double Fy = Ry;
			double Fz = Rz - mass * gravity;

			double ax = Fx / mass;	//acceleration
			double ay = Fy / mass;
			double az = Fz / mass;

			_speed.x = _last_speed.x + ax*dt;
			_speed.y = _last_speed.y + ay*dt;
			_speed.z = _last_speed.z + az*dt;

			_point.x = _last_point.x + _speed.x*dt;
			_point.y = _last_point.y + _speed.y*dt;
			_point.z = _last_point.z + _speed.z*dt;

			shuttle_line.points.push_back(_last_point);
			shuttle_line.points.push_back(_point);
			//printf("%f, %f, %f, %f,  %f, %f, %f,  %f, %f, %f\n", i, _last_point.x, _last_point.y, _last_point.z, _point.x, _point.y, _point.z, _last_speed.x, _last_speed.y, _last_speed.z);
			_last_point = _point;
			_last_speed = _speed;
		}
	}

	float *getNowState(){
		return kalman->state_post->data.fl;
	}

	void publish(){
		shuttle_particle.points.clear();
		shuttle_particle.header.stamp = ros::Time::now();

		geometry_msgs::Point p;
#if 0
		for (int i = 0; i < n_particle; i++) {
			p.x = cond->flSamples[i][0];
			p.y = cond->flSamples[i][1];
			p.z = cond->flSamples[i][2];
			shuttle_particle.points.push_back(p);
		}
#endif
		marker_pub.publish(shuttle_particle);
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
