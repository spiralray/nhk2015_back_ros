#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
#include <visualization_msgs/Marker.h>

#include <cv.h>
#include <highgui.h>
#include <ctype.h>
#include <math.h>
#include <opencv2/legacy/legacy.hpp>

#include <pthread.h>

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

class Shuttle{
public:

	ros::NodeHandle nh;

	ros::Publisher marker_pub;
	visualization_msgs::Marker shuttle_line;

	ros::Time timeLastUpdate;

	geometry_msgs::PointStamped oldPoint;

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
		cvSetIdentity( kalman->measurement_noise_cov, cvRealScalar(1e-7) );
		cvSetIdentity( kalman->error_cov_post, cvRealScalar(1));


		float M[MP * DP] = {
				1.0,	0.0f,	0.0f,
				0.0,	0.0f,	0.0f,
				0.0,	0.0f,	0.0f,
				0.0,	1.0f,	0.0f,
				0.0,	0.0f,	0.0f,
				0.0,	0.0f,	0.0f,
				0.0,	0.0f,	1.0f,
				0.0,	0.0f,	0.0f,
				0.0,	0.0f,	0.0f
		};

		//cvZero( kalman->measurement_matrix );
		memcpy( kalman->measurement_matrix->data.fl, M, sizeof(M));

		ROS_INFO("measurement_matrix row:%d col:%d", kalman->measurement_matrix->rows, kalman->measurement_matrix->cols);

		cvZero( kalman->control_matrix );
		kalman->control_matrix->data.fl[ 7 ] = -gravity;

		ROS_INFO("control_matrix row:%d col:%d", kalman->control_matrix->rows, kalman->control_matrix->cols);

		marker_pub = nh.advertise<visualization_msgs::Marker>("/shuttle/kalman", 10);

		shuttle_line.header.frame_id = "/map";
		shuttle_line.header.stamp = ros::Time::now();
		shuttle_line.ns = "shuttle_line";
		shuttle_line.id = 0;
		shuttle_line.type = visualization_msgs::Marker::POINTS;
		shuttle_line.action = visualization_msgs::Marker::ADD;

		shuttle_line.scale.x = 0.1;

		shuttle_line.color.b = 1.0;
		shuttle_line.color.a = 1.0;
	}

	~Shuttle(){
		cvReleaseKalman(&kalman);
	}

	void update(const geometry_msgs::PointStamped& point){

		double sec = point.header.stamp.toSec() - timeLastUpdate.toSec();

		if( sec > 0.3 || updated == 0 ){
			//Resampling
			updated = 1;

			float A[DP] = {
					point.point.x, 0, 0,
					point.point.y, 0, 0,
					point.point.z, 0, 0
			};
			memcpy( kalman->state_post->data.fl, A, sizeof(A));
		}
		else if( updated == 1 ){
			updated = 2;

			float A[DP] = {
				point.point.x,	-(point.point.x - kalman->transition_matrix->data.fl[0])/sec,	0.0f,
				point.point.y,	-(point.point.y - kalman->transition_matrix->data.fl[3])/sec,	0.0f,
				point.point.z,	-(point.point.z - kalman->transition_matrix->data.fl[6])/sec,	0.0f
			};
			memcpy( kalman->state_post->data.fl, A, sizeof(A));
		}
		else{
			updated = 3;

			double v = sqrt( kalman->transition_matrix->data.fl[3]*kalman->transition_matrix->data.fl[3]+ kalman->transition_matrix->data.fl[4]*kalman->transition_matrix->data.fl[4] + kalman->transition_matrix->data.fl[5]*kalman->transition_matrix->data.fl[5]);	//speed of the shuttle

			double R = resist_coeff * v; //Air resistance

			float A[DP * DP] = {
					1.0,			 sec,	0.0f,	0.0f,			0.0f,	0.0f,	0.0f,			0.0f,	0.0f,
					0.0,			1.0f,	 sec,	0.0f,			0.0f,	0.0f,	0.0f,			0.0f,	0.0f,
					0.0,	 	 -R/mass,	0.0f,	0.0f,			0.0f,	0.0f,	0.0f,			0.0f,	0.0f,
					0.0,			0.0f,	0.0f,	1.0f,			 sec,	0.0f,	0.0f,			0.0f,	0.0f,
					0.0,			0.0f,	0.0f,	0.0f,			1.0f,	 sec,	0.0f,			0.0f,	0.0f,
					0.0,			0.0f,	0.0f,	0.0f,	 	 -R/mass,	0.0f,	0.0f,			0.0f,	0.0f,
					0.0,			0.0f,	0.0f,	0.0f,			0.0f,	0.0f,	1.0f,			 sec,	0.0f,
					0.0,			0.0f,	0.0f,	0.0f,			0.0f,	0.0f,	0.0f,			1.0f,	 sec,
					0.0,			0.0f,	0.0f,	0.0f,			0.0f,	0.0f,	0.0f,	 	 -R/mass,	0.0f
			};

			memcpy( kalman->transition_matrix->data.fl, A, sizeof(A));

			predict( sec );


			float m[MP] = {point.point.x, point.point.y ,point.point.z };
			CvMat measurement = cvMat(MP, 1, CV_32FC1, m);
			const CvMat *correction = cvKalmanCorrect(kalman, &measurement);

			ROS_INFO("%.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f",
					kalman->state_post->data.fl[0], kalman->state_post->data.fl[1], kalman->state_post->data.fl[2],
					kalman->state_post->data.fl[3], kalman->state_post->data.fl[4], kalman->state_post->data.fl[5],
					kalman->state_post->data.fl[6], kalman->state_post->data.fl[7], kalman->state_post->data.fl[8]);

		}

		oldPoint = point;
		timeLastUpdate = point.header.stamp;

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


		_last_point.x = kalmanOrbit->state_post->data.fl[0];
		_last_point.y = kalmanOrbit->state_post->data.fl[1];
		_last_point.z = kalmanOrbit->state_post->data.fl[2];


		for(double i=0; i < TIME_CALCULATE ; i+=dt){

			double v = sqrt( kalmanOrbit->transition_matrix->data.fl[3]*kalmanOrbit->transition_matrix->data.fl[3]+ kalmanOrbit->transition_matrix->data.fl[4]*kalmanOrbit->transition_matrix->data.fl[4] + kalmanOrbit->transition_matrix->data.fl[5]*kalmanOrbit->transition_matrix->data.fl[5]
			);	//speed of the shuttle

			double R = resist_coeff * v; //Air resistance

			float A[DP * DP] = {
					1.0,			  dt,	0.0f,	0.0f,			0.0f,	0.0f,	0.0f,			0.0f,	0.0f,
					0.0,			1.0f,	  dt,	0.0f,			0.0f,	0.0f,	0.0f,			0.0f,	0.0f,
					0.0,	     -R/mass,	0.0f,	0.0f,			0.0f,	0.0f,	0.0f,			0.0f,	0.0f,
					0.0,			0.0f,	0.0f,	1.0f,			  dt,	0.0f,	0.0f,			0.0f,	0.0f,
					0.0,			0.0f,	0.0f,	0.0f,			1.0f,	  dt,	0.0f,			0.0f,	0.0f,
					0.0,			0.0f,	0.0f,	0.0f,	     -R/mass,	0.0f,	0.0f,			0.0f,	0.0f,
					0.0,			0.0f,	0.0f,	0.0f,			0.0f,	0.0f,	1.0f,			  dt,	0.0f,
					0.0,			0.0f,	0.0f,	0.0f,			0.0f,	0.0f,	0.0f,			1.0f,	  dt,
					0.0,			0.0f,	0.0f,	0.0f,			0.0f,	0.0f,	0.0f,	     -R/mass,	0.0f
			};


			memcpy( kalmanOrbit->transition_matrix->data.fl, A, sizeof(A));

			float c[CP] = { dt };
			CvMat control = cvMat(CP, 1, CV_32FC1, c);
			const CvMat *prediction = cvKalmanPredict( kalmanOrbit, &control );

			_point.x = kalmanOrbit->state_post->data.fl[0];
			_point.y = kalmanOrbit->state_post->data.fl[3];
			_point.z = kalmanOrbit->state_post->data.fl[6];

			//shuttle_line.points.push_back(_last_point);
			shuttle_line.points.push_back(_point);
			_last_point = _point;
			//if( _point.z <= 0.0f ) break;
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


void pointsCallback(const geometry_msgs::PointStamped& point)
{

	shuttle->update(point);

	float *nowState = shuttle->getNowState();
	shuttle->calc(nowState);

	shuttle->publish();

}

int main (int argc, char **argv)
{
	ros::init(argc, argv, "shuttle_listener");

	shuttle = new Shuttle();

	ros::Subscriber subscriber = shuttle->nh.subscribe("/shuttle/point", 1, pointsCallback);

	ros::spin();

	delete shuttle;

	return 0;
}
