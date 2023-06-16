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

	ros::Time cur_time;	// 現在の時間
	ros::Time last_time;	// 前回の時間
	double d_time;		// 微小時間
	double cur_x;		// 現在のX座標値
	double cur_y;		// 現在のY座標値
	double cur_th;		// 現在のYaw角
	double cur_v_x;		// 現在の並進速度
	double cur_v_th;	// 現在の旋回角速度

	double Lr, Ll;		// 右車輪の移動距離，左車輪の移動距離
	double d_Lr, d_Ll;	// 右車輪の微小移動距離，左車輪の微小移動距離
	double Lr_o, Ll_o;

	double track_width;	// トラック幅
	double wheel_radius;	// 車輪半径

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
