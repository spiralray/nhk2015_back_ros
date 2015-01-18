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

	CvConDensation *cond = 0;
	CvMat *lowerBound = 0;
	CvMat *upperBound = 0;

	ros::Time timeLastUpdate;

	geometry_msgs::Pose oldPose;

	bool updated;

	void MyConDensInitSampleSet( CvConDensation * conDens, CvMat * lowerBound, CvMat * upperBound ){
		int i, j;
		float *LBound;
		float *UBound;
		float Prob = 1.f / conDens->SamplesNum;

		if( !conDens || !lowerBound || !upperBound )
			CV_Error( CV_StsNullPtr, "" );

		if( CV_MAT_TYPE(lowerBound->type) != CV_32FC1 ||
				!CV_ARE_TYPES_EQ(lowerBound,upperBound) )
			CV_Error( CV_StsBadArg, "source  has not appropriate format" );

		if( (lowerBound->cols != 1) || (upperBound->cols != 1) )
			CV_Error( CV_StsBadArg, "source  has not appropriate size" );

		if( (lowerBound->rows != conDens->DP) || (upperBound->rows != conDens->DP) )
			CV_Error( CV_StsBadArg, "source  has not appropriate size" );

		LBound = lowerBound->data.fl;
		UBound = upperBound->data.fl;
		/* Initializing the structures to create initial Sample set */
		for( i = 0; i < conDens->DP; i++ )
		{
			cvRandInit( &(conDens->RandS[i]),
					LBound[i],
					UBound[i],
					i + (int)cvGetTickCount());
		}
		/* Generating the samples */
		for( j = 0; j < conDens->SamplesNum; j++ )
		{
			for( i = 0; i < conDens->DP; i++ )
			{
				cvbRand( conDens->RandS + i, conDens->flSamples[j] + i, 1 );
			}
			conDens->flConfidence[j] = Prob;
		}
		/* Reinitializes the structures to update samples randomly */
		for( i = 0; i < conDens->DP; i++ )
		{
			cvRandInit( &(conDens->RandS[i]),
					(LBound[i] - UBound[i]) / 5,
					(UBound[i] - LBound[i]) / 5,
					i + (int)cvGetTickCount());
		}
	}

	Shuttle(){

		updated = false;
		timeLastUpdate = ros::Time::now();

		marker_pub = nh.advertise<visualization_msgs::Marker>("shuttle_particles", 10);

		// Create Condensation class
		cond = cvCreateConDensation (n_stat, 0, n_particle);

		// Set upper and lower limits of each elements of the state vector
		lowerBound = cvCreateMat (n_stat, 1, CV_32FC1);
		upperBound = cvCreateMat (n_stat, 1, CV_32FC1);

		cvmSet (lowerBound, 0, 0, -5.0);
		cvmSet (lowerBound, 1, 0, -5.0);
		cvmSet (lowerBound, 2, 0, -1.0);
		cvmSet (lowerBound, 3, 0, -10.0);
		cvmSet (lowerBound, 4, 0, -10.0);
		cvmSet (lowerBound, 5, 0, -10.0);

		cvmSet (upperBound, 0, 0, 5.0);
		cvmSet (upperBound, 1, 0, 5.0);
		cvmSet (upperBound, 2, 0, 5.0);
		cvmSet (upperBound, 3, 0, 10.0);
		cvmSet (upperBound, 4, 0, 10.0);
		cvmSet (upperBound, 5, 0, 10.0);

		//Init Condensation class
		MyConDensInitSampleSet (cond, lowerBound, upperBound);

		// (7)ConDensationアルゴリズムにおける状態ベクトルのダイナミクスを指定する
		cond->DynamMatr[0] = 1.0;
		cond->DynamMatr[1] = 0.0;
		cond->DynamMatr[2] = 0.0;
		cond->DynamMatr[3] = 1.0/((float)freq);
		cond->DynamMatr[4] = 0.0;
		cond->DynamMatr[5] = 0.0;

		cond->DynamMatr[6] = 0.0;
		cond->DynamMatr[7] = 1.0;
		cond->DynamMatr[8] = 0.0;
		cond->DynamMatr[9] = 0.0;
		cond->DynamMatr[10] = 1.0/((float)freq);
		cond->DynamMatr[11] = 0.0;

		cond->DynamMatr[12] = 0.0;
		cond->DynamMatr[13] = 0.0;
		cond->DynamMatr[14] = 1.0;
		cond->DynamMatr[15] = 0.0;
		cond->DynamMatr[16] = 0.0;
		cond->DynamMatr[17] = 1.0/((float)freq);

		cond->DynamMatr[18] = 0.0;
		cond->DynamMatr[19] = 0.0;
		cond->DynamMatr[20] = 0.0;
		cond->DynamMatr[21] = 1.0;
		cond->DynamMatr[22] = 0.0;
		cond->DynamMatr[23] = 0.0;

		cond->DynamMatr[24] = 0.0;
		cond->DynamMatr[25] = 0.0;
		cond->DynamMatr[26] = 0.0;
		cond->DynamMatr[27] = 0.0;
		cond->DynamMatr[28] = 1.0;
		cond->DynamMatr[29] = 0.0;

		cond->DynamMatr[30] = 0.0;
		cond->DynamMatr[31] = 0.0;
		cond->DynamMatr[32] = 0.0;
		cond->DynamMatr[33] = 0.0;
		cond->DynamMatr[34] = 0.0;
		cond->DynamMatr[35] = 1.0;

		// Reconfigure parameters of noise
		cvRandInit (&(cond->RandS[0]), -0.1, 0.1, (int) cvGetTickCount ());
		cvRandInit (&(cond->RandS[1]), -0.1, 0.1, (int) cvGetTickCount ());
		cvRandInit (&(cond->RandS[2]), -0.05, 0.05, (int) cvGetTickCount ());
		cvRandInit (&(cond->RandS[3]), -0.05, 0.05, (int) cvGetTickCount ());
		cvRandInit (&(cond->RandS[4]), -0.05, 0.05, (int) cvGetTickCount ());
		cvRandInit (&(cond->RandS[5]), -0.05, 0.05, (int) cvGetTickCount ());

		//
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
	}

	~Shuttle(){
		cvReleaseConDensation (&cond);
		cvReleaseMat (&lowerBound);
		cvReleaseMat (&upperBound);
	}

	float calc_likelihood (double duration, const geometry_msgs::Pose& pose, const float* particle)
	{
		float dist = 0.0, sigma = 15.0;

		float vx = (pose.position.x - oldPose.position.x) / duration;
		float vy = (pose.position.y - oldPose.position.y) / duration;
		float vz = (pose.position.z - oldPose.position.z) / duration;

		dist = sqrt ( pow( pose.position.x - particle[0] ,2) + pow( pose.position.y - particle[1] ,2) + pow( pose.position.z - particle[2] ,2) );
		dist += sqrt ( pow( vx - particle[3] ,2) + pow( vy - particle[4] ,2) + pow( vz - particle[5] ,2) );

		return 1.0 / (sqrt (2.0 * CV_PI) * sigma) * expf (-dist * dist / (2.0 * sigma * sigma));
	}

	void update(const ros::Time time, const geometry_msgs::Pose& pose){

		double sec = time.toSec() - timeLastUpdate.toSec();

		if( sec > 0.5){
			//Resampling

			//Init Condensation class
			MyConDensInitSampleSet (cond, lowerBound, upperBound);

			// Reconfigure parameters of noise
			cvRandInit (&(cond->RandS[0]), -0.1, 0.1, (int) cvGetTickCount ());
			cvRandInit (&(cond->RandS[1]), -0.1, 0.1, (int) cvGetTickCount ());
			cvRandInit (&(cond->RandS[2]), -0.1, 0.1, (int) cvGetTickCount ());
			cvRandInit (&(cond->RandS[3]), -0.1, 0.1, (int) cvGetTickCount ());
			cvRandInit (&(cond->RandS[4]), -0.1, 0.1, (int) cvGetTickCount ());
			cvRandInit (&(cond->RandS[5]), -0.1, 0.1, (int) cvGetTickCount ());

			updated = false;
		}

		else{

			cond->DynamMatr[3] = sec;
			cond->DynamMatr[10] = sec;
			cond->DynamMatr[17] = sec;

			// Calculate likelihood of every particle
			for (int i = 0; i < n_particle; i++) {
				geometry_msgs::Pose tmp;
				tmp.position.x = cond->flSamples[i][0];
				tmp.position.y = cond->flSamples[i][1];
				tmp.position.z = cond->flSamples[i][2];

				cond->flConfidence[i] = calc_likelihood(sec, pose, cond->flSamples[i]);
			}

			updated = true;
		}

		oldPose = pose;
		timeLastUpdate = time;

	}
	void estimate(){

		if( updated ){
			// Estimate next state
			cvConDensUpdateByTime (cond);
		}
	}

	void publish(){
		shuttle_particle.points.clear();
		shuttle_particle.header.stamp = ros::Time::now();

		geometry_msgs::Point p;

		for (int i = 0; i < n_particle; i++) {
			p.x = cond->flSamples[i][0];
			p.y = cond->flSamples[i][1];
			p.z = cond->flSamples[i][2];
			shuttle_particle.points.push_back(p);
		}
		marker_pub.publish(shuttle_particle);
	}

};

Shuttle *shuttle;


void pointsCallback(const geometry_msgs::PoseArray& posearray)
{
	if( !posearray.poses.empty() ){
		//ROS_INFO("update.");
		shuttle->update(posearray.header.stamp,posearray.poses.at(0));
		shuttle->estimate();
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
