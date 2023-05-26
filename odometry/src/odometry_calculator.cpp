// odometry_calculator.cpp

#include "odometry_calculator.h"

OdometryCalculator::OdometryCalculator()
{
	first_call = 1;
}

void OdometryCalculator::run()
{
	sub_joint_states = nh.subscribe("joint_states", 10, &OdometryCalculator::joint_states_callback, this);
	pub_odom = nh.advertise<nav_msgs::Odometry>("odom", 10);

	ros::NodeHandle pnh("~");
	if(pnh.getParam("~track_width", track_width))
	{
		ROS_INFO("Set track width: %lf", track_width);
	}
	else {
		track_width = 1.0;
		ROS_WARN("Set DEFAULT value for track width: %lf", track_width);
	}

	if(pnh.getParam("~wheel_radius", wheel_radius))
	{
		ROS_INFO("Set wheel radius: %lf", wheel_radius);
	}
	else {
		wheel_radius = 0.1;
		ROS_WARN("Set DEFAULT value for wheel radius: %lf", wheel_radius);
	}
}

void OdometryCalculator::init()
{
	cur_x = 0.0;
	cur_y = 0.0;
	cur_th = 0.0;
	cur_v_x = 0.0;
	cur_v_th = 0.0;
}

void OdometryCalculator::joint_states_callback(const sensor_msgs::JointState::ConstPtr& msg)
{
	cur_time = msg->header.stamp;
	d_time = (cur_time - last_time).toSec();
	last_time = cur_time;

	for(int i=0;i<msg->name.size();i++) {
		if(msg->name[i] == "wheel_right_angle") Lr = wheel_radius * msg->position[i];
		else if(msg->name[i] == "wheel_left_angle") Ll = wheel_radius * msg->position[i];
	}

	d_Lr = Lr - Lr_o;
	d_Ll = Ll - Ll_o;
	Lr_o = Lr;
	Ll_o = Ll;

	if(first_call == 1) {
		init();
		first_call = 0;
	}
	else {
		update();
	}
}

void OdometryCalculator::update()
{
	double d_L = (d_Lr + d_Ll) / 2.0;
	double d_theta = 0.0;

	if(std::fabs(d_Lr - d_Ll) < 0.000001)
	{
		d_theta = 0.0;
		cur_x += d_L * cos(cur_th);
		cur_y += d_L * sin(cur_th);
	}
	else
	{
		d_theta = (d_Lr - d_Ll) / track_width;
		double rho = d_L / d_theta;
		double d_Lp = 2.0 * rho * sin(d_theta * 0.5);
		cur_x += d_Lp * cos(cur_th + d_theta * 0.5);
		cur_y += d_Lp * sin(cur_th + d_theta * 0.5);
		cur_th = normalize_angle(cur_th + d_theta);
	}

	if(d_time < 0.000001)
	{
		cur_v_x = 0.0;
		cur_v_th = 0.0;
	}
	else
	{
		cur_v_x = d_L / d_time;
		cur_v_th = d_theta / d_time;
	}

	publish_odom();
	publish_tf();
}

void OdometryCalculator::publish_odom()
{
	odom.header.stamp = cur_time;
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

	odom.twist.twist.linear.x = cur_v_x;
	odom.twist.twist.linear.y = 0.0;
	odom.twist.twist.angular.z = cur_v_th;

	odom.twist.covariance[0] = 0.01;
	odom.twist.covariance[7] = 0.01;
	odom.twist.covariance[14] = FLT_MAX;
	odom.twist.covariance[21] = FLT_MAX;
	odom.twist.covariance[28] = FLT_MAX;
	odom.twist.covariance[35] = 0.01;

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
