#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <visualization_msgs/Marker.h>

#include <math.h>

ros::Publisher marker_pub;
ros::Publisher speed_pub, accel_pub;

class Orbit{
public:
	static const double TIME_CALCULATE = 1.0;	//[s]

	static const double resist_coeff = 0.001075;//resistance coefficent of air [N/(m/s)^2]
	static const double dt = 0.005;			//[s]
	static const double gravity = 9.812;	//[m/s^2]
	static const double mass = 0.00467;		//[kg]

	visualization_msgs::Marker shuttle_line;
	geometry_msgs::PointStamped location;
	geometry_msgs::Point speed, last_speed;
	geometry_msgs::Point accel, last_accel;

	bool is_shuttle;

	Orbit(){

		speed.x = speed.y = speed.z = 0;
		last_speed.x = last_speed.y = last_speed.z = 0;

		accel.x = accel.y = accel.z = 0;
		last_accel.x = last_accel.y = last_accel.z = 0;

		location.point.x = location.point.y = location.point.z = 0.0;
		location.header.stamp.sec = 0;
		location.header.stamp.nsec = 0;

		shuttle_line.header.frame_id = "/laser";
		shuttle_line.header.stamp = ros::Time::now();
		shuttle_line.ns = "shuttle_line";
		shuttle_line.id = 0;
		shuttle_line.type = visualization_msgs::Marker::LINE_LIST;
		shuttle_line.action = visualization_msgs::Marker::ADD;

		shuttle_line.scale.x = 0.03;

		shuttle_line.color.g = 0.5;
		shuttle_line.color.a = 1.0;

		is_shuttle = false;

		//shuttle_line.lifetime = ros::Duration(0.2);

	}

	void update(const ros::Time time, const geometry_msgs::Pose& pose){
		double duration = time.toSec() - location.header.stamp.toSec();

		if(duration > 0){
			location.header.stamp = time;

			last_speed = speed;

			speed.x = (pose.position.x - location.point.x)/duration;
			speed.y = (pose.position.y - location.point.y)/duration;
			speed.z = (pose.position.z - location.point.z)/duration;
			location.point = pose.position;

			accel.x = (last_speed.x - speed.x) / duration;
			accel.y = (last_speed.y - speed.y) / duration;
			accel.z = (last_speed.z - speed.z) / duration;

			//ROS_INFO("%f %f %f %lf", accel.z, speed.z, last_speed.z, duration);

			is_shuttle = true;

			if(abs(speed.x) > 50){
				is_shuttle = false;
			}
			else if(abs(speed.y) > 50){
				is_shuttle = false;
			}
			else if(abs(speed.z) > 50){
				is_shuttle = false;
			}
#if 0
			if(abs(accel.x) > 50){
				is_shuttle = false;
			}
			else if(abs(accel.y) > 50){
				is_shuttle = false;
			}
			else if(abs(accel.z) > 50){
				is_shuttle = false;
			}
#endif

			//ROS_INFO("Update %f %f %f %f %f %f", location.point.x, location.point.y, location.point.z, speed.x, speed.y, speed.z);
		}
	}

	void calc(){

		//init
		shuttle_line.points.clear();
		shuttle_line.header.stamp = location.header.stamp;

		geometry_msgs::Point _last_point = location.point;
		geometry_msgs::Point _last_speed = speed;
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
};

Orbit *orbit;

void pointsCallback(const geometry_msgs::PoseArray& posearray)
{
	if( !posearray.poses.empty() ){
		orbit->update(posearray.header.stamp,posearray.poses.at(0));
		orbit->calc();

		if( orbit->is_shuttle ){
			marker_pub.publish(orbit->shuttle_line);

			speed_pub.publish(orbit->speed);
			accel_pub.publish(orbit->accel);
		}
	}
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "shuttle_listener");
  ros::NodeHandle n;

  orbit = new Orbit();
  marker_pub = n.advertise<visualization_msgs::Marker>("shuttle_line", 10);
  speed_pub = n.advertise<geometry_msgs::Point>("shuttle/speed", 10);
  accel_pub = n.advertise<geometry_msgs::Point>("shuttle/accel", 10);

  ros::Subscriber subscriber = n.subscribe("shuttle_points", 100, pointsCallback);

  ros::spin();

  return 0;
}
