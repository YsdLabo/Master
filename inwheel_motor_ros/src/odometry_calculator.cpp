// odometry_calculator.cpp

#include "odometry_calculator.h"

#define  ABS(x)  x>0?(x):-(x)

OdometryCalculator::OdometryCalculator()
{
	pub_odom = nh.advertise<nav_msgs::Odometry>("odom", 10);
	set_track_width(0.0);
	init();
}

OdometryCalculator::OdometryCalculator(double tw)
{
	pub_odom = nh.advertise<nav_msgs::Odometry>("odom", 10);
	set_track_width(tw);
	init();
}

void OdometryCalculator::init()
{
	last_time = ros::Time::now();
	cur_x = 0.0;
	cur_y = 0.0;
	cur_th = 0.0;
	vel_x = 0.0;
	vel_th = 0.0;
}

void OdometryCalculator::set_track_width(double tw)
{
	if(tw > 0.0)	track_width = tw;
}

void OdometryCalculator::update(double dr, double dl)
{
	// odometry
	current_time = ros::Time::now();
	double d_time = (current_time - last_time).toSec();

	double d_dist = (dr + dl) / 2.0;
	double d_theta = 0.0;
	if(ABS(dr - dl) < 0.000001)
	{
		d_theta = 0.0;
		cur_x += d_dist * cos(cur_th);
		cur_y += d_dist * sin(cur_th);
	}
	else
	{
		d_theta = (dr - dl) / track_width;
		double r = d_dist / d_theta;
		double tmp = 2.0 * r * sin(d_theta * 0.5);
		cur_x += tmp * cos(cur_th + d_theta * 0.5);
		cur_y += tmp * sin(cur_th + d_theta * 0.5);
		cur_th = normalize_angle(cur_th + d_theta);
	}

	if(d_time < 0.000001)
	{
		vel_x = 0.0;
		vel_th = 0.0;
	}
	else
	{
		vel_x = d_dist / d_time;
		vel_th = d_theta / d_time;
	}

	make_odom_msg();	
	publish_odom();
	publish_tf();
}

void OdometryCalculator::make_odom_msg()
{
	odom.header.stamp = current_time;
	odom.header.frame_id = "odom";
	odom.child_frame_id = "base_footprint";

	odom.pose.pose.position.x = cur_x;
	odom.pose.pose.position.y = cur_y;
	odom.pose.pose.position.z = 0.0;
	odom.pose.pose.orientation = tf::createQuaternionMsgFromYaw(cur_th);

	odom.pose.covariance[0] = 0.01;
	odom.pose.covariance[7] = 0.01;
	odom.pose.covariance[14] = FLT_MAX;
	odom.pose.covariance[21] = FLT_MAX;
	odom.pose.covariance[28] = FLT_MAX;
	odom.pose.covariance[35] = 0.01;

	odom.twist.twist.linear.x = vel_x;
	odom.twist.twist.linear.y = 0.0;
	odom.twist.twist.angular.z = vel_th;

	odom.twist.covariance[0] = 0.01;
	odom.twist.covariance[7] = 0.01;
	odom.twist.covariance[14] = FLT_MAX;
	odom.twist.covariance[21] = FLT_MAX;
	odom.twist.covariance[28] = FLT_MAX;
	odom.twist.covariance[35] = 0.01;	
}

void OdometryCalculator::publish_odom()
{
	pub_odom.publish(odom);
}

void OdometryCalculator::publish_tf()
{
	geometry_msgs::TransformStamped  tf_trans;

	tf_trans.header.stamp = odom.header.stamp;
	tf_trans.header.frame_id = odom.header.frame_id;
	tf_trans.child_frame_id = odom.child_frame_id;
	tf_trans.transform.translation.x = odom.pose.pose.position.x;
	tf_trans.transform.translation.y = odom.pose.pose.position.y;
	tf_trans.transform.translation.z = odom.pose.pose.position.z;
	tf_trans.transform.rotation = odom.pose.pose.orientation;

	tf_caster.sendTransform(tf_trans);
}

double OdometryCalculator::normalize_angle(double angle)
{
	while(angle > M_PI) angle -= 2.0 * M_PI;
	while(angle < -M_PI) angle += 2.0 * M_PI;
	return angle;
}
