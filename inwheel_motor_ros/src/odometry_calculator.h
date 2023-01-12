// odometry_calculator.h
#include<ros/ros.h>
#include<nav_msgs/Odometry.h>
#include<tf/transform_broadcaster.h>
//#include<geometry_msgs/TransformStamped.h>

#ifndef _ODOMETRY_CALCULATOR_H_
#define _ODOMETRY_CALCULATOR_H_

class OdometryCalculator
{
private:
	ros::NodeHandle nh;
	nav_msgs::Odometry odom;
	tf::TransformBroadcaster tf_caster;
	ros::Publisher pub_odom;

	ros::Time current_time;
	ros::Time last_time;
	double cur_x;
	double cur_y;
	double cur_th;
	double vel_x;
	double vel_th;

	double track_width;

	void publish_odom();
	void publish_tf();
	void make_odom_msg();
	double normalize_angle(double angle);

public:
	OdometryCalculator() = delete;
	OdometryCalculator(double tw);
	void init();
	void set_track_width(double tw);
	void update(double dr, double dl);
};

#endif
