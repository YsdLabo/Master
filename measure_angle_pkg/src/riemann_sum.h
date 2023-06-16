#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Float64.h>

#ifndef _INCLUDED_RIEMANN_SUM_H_
#define _INCLUDED_RIEMANN_SUM_H_

class RiemannSum
{
private:
	ros::NodeHandle nh;
	ros::Subscriber sub_imu;
	ros::Publisher  pub_angle;
	sensor_msgs::Imu imu_raw;

	double angle_z;
	
	ros::Time  last_stamp;
	bool first_time;
	
public:
	RiemannSum();
	void init();
	void run();
	void imu_callback(const sensor_msgs::Imu::ConstPtr& msg);
	void publication();
};

#endif
