// odometry_calculator.h
#include<ros/ros.h>
#include<nav_msgs/Odometry.h>
#include<sensor_msgs/JointState.h>
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
	ros::Subscriber sub_joint_states;
	ros::Publisher pub_odom;

	int first_call;

	ros::Time cur_time;
	ros::Time last_time;
	double d_time;
	double cur_x;
	double cur_y;
	double cur_th;
	double cur_v_x;
	double cur_v_th;

	double Lr, Ll;
	double d_Lr, d_Ll;
	double Lr_o, Ll_o;

	double track_width;
	double wheel_radius;

	void init();
	void update();
	void publish_odom();
	void publish_tf();
	double normalize_angle(double angle);
	void joint_states_callback(const sensor_msgs::JointState::ConstPtr&);

public:
	OdometryCalculator();
	void run();
};

#endif
