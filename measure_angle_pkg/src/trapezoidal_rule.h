#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Float64.h>

#ifndef _INCLUDED_TRAPEZOIDAL_RULE_H_
#define _INCLUDED_TRAPEZOIDAL_RULE_H_

class TrapezoidalRule
{
private:
	ros::NodeHandle nh;
	ros::Subscriber sub_imu;
	ros::Publisher  pub_angle;
	sensor_msgs::Imu imu_raw;

	double angle_z;
	double last_angular_z;
	
	ros::Time  last_stamp;
	bool first_time;
	
public:
	TrapezoidalRule();
	void init();
	void run();
	void imu_callback(const sensor_msgs::Imu::ConstPtr& msg);
	void publication();
};

#endif
