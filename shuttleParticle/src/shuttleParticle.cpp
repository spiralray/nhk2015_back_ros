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

pthread_mutex_t	mutex;  // MUTEX
bool will_shutdown = false;
void thread_main();

namespace ShuttleConst{
const double TIME_CALCULATE = 1.0;	//[s]
const double resist_coeff = 0.001075;//resistance coefficent of air [N/(m/s)^2]
const double dt = 0.005;			//[s]
const double gravity = 9.812;	//[m/s^2]
const double mass = 0.00467;		//[kg]

const int freq = 200;

const int n_stat = 6;
const int n_particle = 100;
}

using namespace ShuttleConst;

class Shuttle{
public:

	ros::NodeHandle nh;

	ros::Publisher marker_pub;
	visualization_msgs::Marker shuttle_particle;

	CvConDensation *cond = 0;
	CvMat *lowerBound = 0;
	CvMat *upperBound = 0;

	Shuttle(){

		marker_pub = nh.advertise<visualization_msgs::Marker>("shuttle_particles", 1);

		// Create Condensation class
		cond = cvCreateConDensation (n_stat, 0, n_particle);

		// Set upper and lower limits of each elements of the state vector
		lowerBound = cvCreateMat (n_stat, 1, CV_32FC1);
		upperBound = cvCreateMat (n_stat, 1, CV_32FC1);

		cvmSet (lowerBound, 0, 0, -10.0);
		cvmSet (lowerBound, 1, 0, -10.0);
		cvmSet (lowerBound, 2, 0, -10.0);
		cvmSet (lowerBound, 3, 0, -10.0);
		cvmSet (lowerBound, 4, 0, -10.0);
		cvmSet (lowerBound, 5, 0, -10.0);

		cvmSet (upperBound, 0, 0, 10.0);
		cvmSet (upperBound, 1, 0, 10.0);
		cvmSet (upperBound, 2, 0, 10.0);
		cvmSet (upperBound, 3, 0, 10.0);
		cvmSet (upperBound, 4, 0, 10.0);
		cvmSet (upperBound, 5, 0, 10.0);

		//Init Condensation class
		cvConDensInitSampleSet (cond, lowerBound, upperBound);

		// (7)ConDensationアルゴリズムにおける状態ベクトルのダイナミクスを指定する
		cond->DynamMatr[0] = 1.0;
		cond->DynamMatr[1] = 0.0;
		cond->DynamMatr[2] = 0.0;
		cond->DynamMatr[3] = 1.0;
		cond->DynamMatr[4] = 0.0;
		cond->DynamMatr[5] = 0.0;

		cond->DynamMatr[6] = 0.0;
		cond->DynamMatr[7] = 1.0;
		cond->DynamMatr[8] = 0.0;
		cond->DynamMatr[9] = 0.0;
		cond->DynamMatr[10] = 1.0;
		cond->DynamMatr[11] = 0.0;

		cond->DynamMatr[12] = 0.0;
		cond->DynamMatr[13] = 0.0;
		cond->DynamMatr[14] = 1.0;
		cond->DynamMatr[15] = 0.0;
		cond->DynamMatr[16] = 0.0;
		cond->DynamMatr[17] = 1.0;

		cond->DynamMatr[18] = 0.0;
		cond->DynamMatr[19] = 0.0;
		cond->DynamMatr[20] = 0.0;
		cond->DynamMatr[21] = 0.0;
		cond->DynamMatr[22] = 0.0;
		cond->DynamMatr[23] = 0.0;

		cond->DynamMatr[24] = 0.0;
		cond->DynamMatr[25] = 0.0;
		cond->DynamMatr[26] = 0.0;
		cond->DynamMatr[27] = 0.0;
		cond->DynamMatr[28] = 0.0;
		cond->DynamMatr[29] = 0.0;

		cond->DynamMatr[30] = 0.0;
		cond->DynamMatr[31] = 0.0;
		cond->DynamMatr[32] = 0.0;
		cond->DynamMatr[33] = 0.0;
		cond->DynamMatr[34] = 0.0;
		cond->DynamMatr[35] = 0.0;

		// Reconfigure parameters of noise
		cvRandInit (&(cond->RandS[0]), -25, 25, (int) cvGetTickCount ());
		cvRandInit (&(cond->RandS[1]), -25, 25, (int) cvGetTickCount ());
		cvRandInit (&(cond->RandS[2]), -5, 5, (int) cvGetTickCount ());
		cvRandInit (&(cond->RandS[3]), -5, 5, (int) cvGetTickCount ());
		cvRandInit (&(cond->RandS[4]), -5, 5, (int) cvGetTickCount ());
		cvRandInit (&(cond->RandS[5]), -5, 5, (int) cvGetTickCount ());
	}

	~Shuttle(){
		cvReleaseConDensation (&cond);
		cvReleaseMat (&lowerBound);
		cvReleaseMat (&upperBound);
	}

	float calc_likelihood (const geometry_msgs::Pose& pose, float *flSample)
	{
		float dist = 0.0, sigma = 50.0;

		dist = sqrt ( pow( pose.position.x - flSample[0] ,2) + pow( pose.position.y - flSample[1] ,2) + pow( pose.position.z - flSample[2] ,2) );

		return 1.0 / (sqrt (2.0 * CV_PI) * sigma) * expf (-dist * dist / (2.0 * sigma * sigma));
	}

	void update(const ros::Time time, const geometry_msgs::Pose& pose){
		// Calculate likelihood of every particle
		for (int i = 0; i < n_particle; i++) {
			float xx = (cond->flSamples[i][0]);
			float yy = (cond->flSamples[i][1]);
			float zz = (cond->flSamples[i][2]);

			if (xx < -10 || xx >= 10 || yy < -10 || yy >= 10 || zz < -10 || zz >= 10 ) {
				cond->flConfidence[i] = 0.0;
			}
			else {
				cond->flConfidence[i] = calc_likelihood(pose, cond->flSamples[i]);
			}
		}

	}
	void estimate(){
		// Estimate next state
		cvConDensUpdateByTime (cond);
	}

};

Shuttle *shuttle;


void pointsCallback(const geometry_msgs::PoseArray& posearray)
{
	if( !posearray.poses.empty() ){
		pthread_mutex_lock( &mutex );
		ROS_INFO("update.");
		shuttle->update(posearray.header.stamp,posearray.poses.at(0));
		pthread_mutex_unlock( &mutex );
	}
}

void update(int id)
{
	pthread_mutex_lock( &mutex );
	//ROS_INFO("estimate.");
	shuttle->estimate();
	pthread_mutex_unlock( &mutex );
}

int main (int argc, char **argv)
{
	ros::init(argc, argv, "shuttle_listener");

	shuttle = new Shuttle();

	ros::Subscriber subscriber = shuttle->nh.subscribe("shuttle_points", 100, pointsCallback);

	will_shutdown = false;

	pthread_t thread;
	pthread_create( &thread, NULL, (void* (*)(void*))thread_main, NULL );

	TimerManager t;
	t.addTimer(1000000 / freq, update);
	t.start();

	while( !will_shutdown ){
		sleep(10);
	}

	will_shutdown = true;
	pthread_join( thread, NULL );

	delete shuttle;

	return 0;
}

void thread_main(){
	ROS_INFO("New thread Created.");
	pthread_detach( pthread_self( ));

	ros::spin();
	will_shutdown = true;
}
